# Raspberry Pi Controller Code - Single Threaded with Temperature Measurement
# This program controls a robot's movement based on voltage readings and measures temperature

# ============== IMPORTS ==============
import usbtmc
import numpy as np
import pandas as pd
import time
import math
import os
import glob
import sys
from datetime import datetime
import spidev
from dataclasses import dataclass
from typing import Callable, List, Optional
from usbtmc.usbtmc import UsbtmcException

# ============== CONFIGURATION ==============
# Hardware Configuration
KEITHLEY_VENDOR_ID = 0x05E6
KEITHLEY_PRODUCT_ID = 0x2450
MAX_VOLTAGE = 10.0  # Maximum voltage limit
MAX_CURRENT = 2.0  # Maximum current limit
MAX_TEMP = 100.0   # Maximum temperature limit
MIN_TEMP = 0.0     # Minimum temperature limit
INITIAL_WAIT_TIME = 20.0  # Initial wait time before threshold detection in seconds
TEST_MODE = True  # Set to True to skip threshold detection

# Temperature Control Configuration
VOLTAGE_TO_TEMP_SLOPE = 0.1  # Temperature change per volt
VOLTAGE_TO_TEMP_OFFSET = 5.0  # Base voltage at 0°C

# ============== SIMULATION CLASSES ==============
class TemperatureField:
    """Simulates a temperature field in 2D space."""
    def __init__(self, func: Callable[[float, float], float], size: float = 100.0):
        self.func = func
        self.size = size

    def temperature(self, x: float, y: float) -> float:
        return self.func(x, y)

    @staticmethod
    def linear(size: float = 100.0, base: float = 25.0, grad_x: float = 0.1, grad_y: float = 0.1):
        return TemperatureField(lambda x, y: base + grad_x * x + grad_y * y, size)

@dataclass
class RobotState:
    """Represents the robot's position and orientation."""
    x: float
    y: float
    heading: float  # degrees

@dataclass
class Robot:
    """Controls robot movement and state."""
    state: RobotState
    step_length: float = 1.0
    arc_angle_degrees: float = 15.0
    turn_angle: float = 15.0
    initial_state: RobotState = None

    def __post_init__(self):
        if self.initial_state is None:
            self.initial_state = RobotState(self.state.x, self.state.y, self.state.heading)

    def _forward(self):
        """Move robot forward in current heading direction."""
        heading_rad = np.deg2rad(self.state.heading)
        self.state.x += self.step_length * np.cos(heading_rad)
        self.state.y += self.step_length * np.sin(heading_rad)

    def arc(self):
        """Move forward and turn slightly."""
        self._forward()
        self.state.heading += self.arc_angle_degrees
        self.state.heading = self.state.heading % 360

    def turn(self):
        """Change heading without moving."""
        self.state.heading += self.turn_angle
        self.state.heading = self.state.heading % 360

    def reset(self):
        """Reset robot to initial state."""
        self.state = RobotState(self.initial_state.x, self.initial_state.y, self.initial_state.heading)

# ============== HARDWARE INTERFACE ==============
def setup_temperature_sensor():
    """Initialize the temperature sensor and return the device file path."""
    os.system('modprobe w1-gpio')
    os.system('modprobe w1-therm')
    
    base_dir = '/sys/bus/w1/devices/'
    device_folder = glob.glob(base_dir + '28*')[0]
    device_file = device_folder + '/w1_slave'
    return device_file

def read_temperature(device_file):
    """Read temperature from sensor."""
    def read_raw():
        with open(device_file, 'r') as f:
            return f.readlines()
    
    lines = read_raw()
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_raw()
    
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        return float(temp_string) / 1000.0

def initialize_keithley():
    """Initialize and configure the Keithley instrument."""
    try:
        keithley = usbtmc.Instrument(KEITHLEY_VENDOR_ID, KEITHLEY_PRODUCT_ID)
        keithley.timeout = 2000  # 2 second timeout
        
        # Configure Keithley
        keithley.write("*RST")  # Reset to default settings
        keithley.write(":OUTP OFF")  # Turn off output
        keithley.write(":SOUR:VOLT 0")  # Set voltage to 0
        keithley.write(f":SENS:VOLT:PROT {MAX_VOLTAGE}")  # Set voltage protection to max voltage
        keithley.write(f":SENS:CURR:PROT {MAX_CURRENT}")  # Set current protection to max current
        
        return keithley
    except Exception as e:
        print(f"Error initializing Keithley: {e}")
        sys.exit(1)

def collect_parameters() -> dict:
    """Collect simulation parameters from user"""
    print("\nEnter simulation parameters (press Enter for defaults):")
    
    # Get field parameters
    field_size = float(input("Field size (default 100.0): ") or "100.0")
    base_temp = float(input("Base temperature (default 25.0): ") or "25.0")
    grad_x = float(input("Temperature gradient in x (default 0.1): ") or "0.1")
    grad_y = float(input("Temperature gradient in y (default 0.1): ") or "0.1")
    
    # Get robot parameters
    initial_x = float(input("Initial x position (default 0.0): ") or "0.0")
    initial_y = float(input("Initial y position (default 0.0): ") or "0.0")
    initial_heading = float(input("Initial heading in degrees (default 0.0): ") or "0.0")
    step_distance = float(input("Step distance (default 1.0): ") or "1.0")
    arc_angle = float(input("Arc angle in degrees (default 15.0): ") or "15.0")
    turn_angle = float(input("Turn angle in degrees (default 15.0): ") or "15.0")
    
    # Get control parameters
    num_steps = int(input("Number of steps (default 10): ") or "10")
    temp_tolerance = float(input("Temperature tolerance in °C (default 1.0): ") or "1.0")
    arc_duration = float(input("Arc movement duration in seconds (default 2.0): ") or "2.0")
    turn_arc_duration = float(input("Turn+arc movement duration in seconds (default 3.0): ") or "3.0")
    initial_duration = float(input("Initial waiting duration in seconds (default 5.0): ") or "5.0")
    
    return {
        'field_size': field_size,
        'base_temp': base_temp,
        'grad_x': grad_x,
        'grad_y': grad_y,
        'initial_x': initial_x,
        'initial_y': initial_y,
        'initial_heading': initial_heading,
        'step_distance': step_distance,
        'arc_angle': arc_angle,
        'turn_angle': turn_angle,
        'num_steps': num_steps,
        'temp_tolerance': temp_tolerance,
        'arc_duration': arc_duration,
        'turn_arc_duration': turn_arc_duration,
        'initial_duration': initial_duration
    }

def log_step_data(log_data: List[dict], step: int, voltage: float, action: str,
                  start_temp: float, end_temp: float, start_pos: tuple, end_pos: tuple,
                  start_heading: float, end_heading: float, actual_temp: float) -> None:
    """Log data for each step"""
    log_data.append({
        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'),
        'step': step,
        'voltage': voltage,
        'action': action,
        'start_temp': start_temp,
        'end_temp': end_temp,
        'actual_temp': actual_temp,
        'start_x': start_pos[0],
        'start_y': start_pos[1],
        'end_x': end_pos[0],
        'end_y': end_pos[1],
        'start_heading': start_heading,
        'end_heading': end_heading
    })

def print_step_details(step: int, voltage: float, action: str,
                      start_temp: float, end_temp: float, start_pos: tuple, end_pos: tuple,
                      start_heading: float, end_heading: float, actual_temp: float,
                      loop_time: float = None) -> None:
    """Print detailed information about each step"""
    print("\n" + "="*50)
    print(f"Step {step} Statistics at {datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}")
    print("="*50)
    
    # Voltage and Action Information
    print("\nVoltage & Action:")
    print(f"Voltage Reading: {voltage:.3f}V")
    print(f"Action Taken: {action}")
    
    # Temperature Information
    print("\nTemperature Information:")
    print(f"Start Temperature: {start_temp:.1f}°C")
    print(f"End Temperature: {end_temp:.1f}°C")
    print(f"Actual Temperature: {actual_temp:.1f}°C")
    print(f"Temperature Difference: {abs(end_temp - actual_temp):.1f}°C")
    
    # Position Information
    print("\nPosition Information:")
    print(f"Start Position: ({start_pos[0]:.1f}, {start_pos[1]:.1f})")
    print(f"End Position: ({end_pos[0]:.1f}, {end_pos[1]:.1f})")
    print(f"Distance Traveled: {math.sqrt((end_pos[0]-start_pos[0])**2 + (end_pos[1]-start_pos[1])**2):.1f} units")
    
    # Heading Information
    print("\nHeading Information:")
    print(f"Start Heading: {start_heading:.1f}°")
    print(f"End Heading: {end_heading:.1f}°")
    print(f"Heading Change: {(end_heading - start_heading) % 360:.1f}°")
    
    # Timing Information
    if loop_time is not None:
        print("\nTiming Information:")
        print(f"Control Loop Time: {loop_time:.3f} seconds")
    
    print("="*50)

def wait_for_initial_threshold(keithley: usbtmc.Instrument) -> float:
    """Wait for the first voltage threshold crossing and return the voltage.
    
    Args:
        keithley: The Keithley instrument instance
        
    Returns:
        float: The voltage reading that triggered the threshold crossing
    """
    print("\n" + "="*50)
    print(f"Waiting {INITIAL_WAIT_TIME} seconds before starting threshold detection...")
    print("="*50)
    time.sleep(INITIAL_WAIT_TIME)
    
    print("\n" + "="*50)
    print("Starting threshold detection...")
    print("="*50)
    
    last_voltage = None
    start_time = time.time()
    readings_count = 0
    
    while True:
        try:
            voltage = float(keithley.ask(":MEAS:VOLT:DC?"))
            readings_count += 1
            
            if last_voltage is not None:
                # Check for state change (crossing 0.5V threshold)
                if (last_voltage <= 0.5 and voltage > 0.5) or (last_voltage > 0.5 and voltage <= 0.5):
                    elapsed_time = time.time() - start_time
                    print("\nThreshold Detection Statistics:")
                    print(f"Initial voltage: {last_voltage:.3f}V")
                    print(f"Trigger voltage: {voltage:.3f}V")
                    print(f"Time to detect: {elapsed_time:.1f} seconds")
                    print(f"Total readings: {readings_count}")
                    print(f"Average reading rate: {readings_count/elapsed_time:.1f} Hz")
                    print("="*50)
                    return voltage
                    
            last_voltage = voltage
            time.sleep(0.1)  # Check every 100ms
            
        except UsbtmcException as e:
            print(f"Error reading voltage: {e}")
            time.sleep(1)
        except KeyboardInterrupt:
            print("\nProgram stopped by user")
            raise

def temperature_to_voltage(temp: float) -> float:
    """Convert temperature to voltage using the mapping: voltage = slope * temp + offset"""
    return VOLTAGE_TO_TEMP_SLOPE * temp + VOLTAGE_TO_TEMP_OFFSET

def main_control_loop(robot: Robot, temp_field: TemperatureField, keithley: usbtmc.Instrument,
                     params: dict, log_data: List[dict], device_file: str) -> None:
    """Main control loop for robot movement and data collection"""
    try:
        # Record initial state
        initial_temp = temp_field.temperature(robot.state.x, robot.state.y)
        initial_actual_temp = read_temperature(device_file)
        log_step_data(log_data, 0, 0.0, "Initial",
                     initial_temp, initial_temp,
                     (robot.state.x, robot.state.y), (robot.state.x, robot.state.y),
                     robot.state.heading, robot.state.heading,
                     initial_actual_temp)
        print_step_details(0, 0.0, "Initial",
                          initial_temp, initial_temp,
                          (robot.state.x, robot.state.y), (robot.state.x, robot.state.y),
                          robot.state.heading, robot.state.heading,
                          initial_actual_temp)

        if not TEST_MODE:
            # Wait for initial threshold crossing
            try:
                initial_voltage = wait_for_initial_threshold(keithley)
                
                # Wait for initial duration after threshold detection
                print("\n" + "="*50)
                print(f"Threshold detected! Waiting for initial duration of {params['initial_duration']} seconds...")
                print("="*50)
                time.sleep(params['initial_duration'])
                
            except KeyboardInterrupt:
                return
        else:
            print("\n" + "="*50)
            print("TEST MODE: Skipping threshold detection")
            print("="*50)
            time.sleep(1)  # Brief pause for visibility

        # Main control loop
        for step in range(1, params['num_steps'] + 1):
            loop_start_time = time.time()
            
            # Read voltage from Keithley
            try:
                voltage = float(keithley.ask(":MEAS:VOLT:DC?"))
            except UsbtmcException as e:
                print(f"Error reading voltage: {e}")
                break

            # Record state before movement
            start_temp = temp_field.temperature(robot.state.x, robot.state.y)
            start_pos = (robot.state.x, robot.state.y)
            start_heading = robot.state.heading

            # Determine and execute movement
            if voltage > 0.5:
                robot.arc()
                print(f"\nExecuting arc movement (waiting {params['arc_duration']} seconds)...")
                time.sleep(params['arc_duration'])
            else:
                robot.turn()
                robot.arc()
                print(f"\nExecuting turn+arc movement (waiting {params['turn_arc_duration']} seconds)...")
                time.sleep(params['turn_arc_duration'])

            # Record state after movement
            end_temp = temp_field.temperature(robot.state.x, robot.state.y)
            end_pos = (robot.state.x, robot.state.y)
            end_heading = robot.state.heading
            actual_temp = read_temperature(device_file)

            # Set heater voltage based on field temperature
            heater_voltage = 5.0 + 0.1 * end_temp
            try:
                keithley.write(f":SOUR:VOLT {heater_voltage}")
                keithley.write(":OUTP ON")
                print(f"\nTemperature Control:")
                print(f"Current Temperature: {actual_temp:.1f}°C")
                print(f"Field Temperature: {end_temp:.1f}°C")
                print(f"Heater Voltage: {heater_voltage:.2f}V")
            except UsbtmcException as e:
                print(f"Error controlling heater: {e}")

            # Log and print step data
            action = "arc" if voltage > 0.5 else "turn+arc"
            log_step_data(log_data, step, voltage, action,
                         start_temp, end_temp, start_pos, end_pos,
                         start_heading, end_heading, actual_temp)
            
            # Calculate loop time
            loop_end_time = time.time()
            loop_time = loop_end_time - loop_start_time
            
            print_step_details(step, voltage, action,
                             start_temp, end_temp, start_pos, end_pos,
                             start_heading, end_heading, actual_temp,
                             loop_time)

            # Check temperature tolerance
            temp_diff = abs(end_temp - actual_temp)
            if temp_diff > params['temp_tolerance']:
                print(f"\nTemperature difference {temp_diff:.1f}°C exceeds tolerance {params['temp_tolerance']}°C")
                break

    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    except Exception as e:
        print(f"\nError in main control loop: {e}")
    finally:
        # Turn off Keithley output
        keithley.write(":OUTP OFF")
        # Save data to CSV files
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Save steps data
        steps_df = pd.DataFrame(log_data)
        steps_df.to_csv(f'robot_steps_{timestamp}.csv', index=False)
        print(f"\nSteps data saved to robot_steps_{timestamp}.csv")
        
        # Save parameters data
        params_df = pd.DataFrame([{
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'),
            'field_size': params['field_size'],
            'base_temp': params['base_temp'],
            'grad_x': params['grad_x'],
            'grad_y': params['grad_y'],
            'initial_x': params['initial_x'],
            'initial_y': params['initial_y'],
            'initial_heading': params['initial_heading'],
            'step_distance': params['step_distance'],
            'arc_angle': params['arc_angle'],
            'turn_angle': params['turn_angle'],
            'num_steps': params['num_steps'],
            'temp_tolerance': params['temp_tolerance'],
            'arc_duration': params['arc_duration'],
            'turn_arc_duration': params['turn_arc_duration'],
            'initial_duration': params['initial_duration'],
            'test_mode': TEST_MODE
        }])
        params_df.to_csv(f'robot_params_{timestamp}.csv', index=False)
        print(f"Parameters saved to robot_params_{timestamp}.csv")

def main():
    """Main program entry point."""
    try:
        # Initialize hardware
        keithley = initialize_keithley()
        device_file = setup_temperature_sensor()  # Setup temperature sensor once
        
        # Collect parameters
        params = collect_parameters()
        
        # Initialize simulation
        temp_field = TemperatureField.linear(
            size=params['field_size'],
            base=params['base_temp'],
            grad_x=params['grad_x'],
            grad_y=params['grad_y']
        )
        
        robot = Robot(
            state=RobotState(
                x=params['initial_x'],
                y=params['initial_y'],
                heading=params['initial_heading']
            ),
            step_length=params['step_distance'],
            arc_angle_degrees=params['arc_angle'],
            turn_angle=params['turn_angle']
        )
        
        # Initialize data logging
        log_data = []
        
        # Run main control loop
        main_control_loop(robot, temp_field, keithley, params, log_data, device_file)
        
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    except Exception as e:
        print(f"\nError in main program: {e}")
    finally:
        print("\nProgram terminated")

if __name__ == "__main__":
    main()



