# Raspberry Pi Controller Code - Multithreaded with PID Temperature Control
# This program controls a robot's movement based on voltage readings and maintains temperature using PID control

# ============== IMPORTS ==============
import usbtmc
import time
import numpy as np
import pandas as pd
import sys
import threading
import queue
from dataclasses import dataclass
from typing import Callable, List, Optional
from usbtmc.usbtmc import UsbtmcException
from simple_pid import PID
import os
import glob

# ============== TEMPERATURE SENSOR SETUP ==============
def setup_temperature_sensor():
    """Initialize the DS18B20 temperature sensor."""
    os.system('modprobe w1-gpio')
    os.system('modprobe w1-therm')
    
    # Find the DS18B20 sensor
    base_dir = '/sys/bus/w1/devices/'
    device_folder = glob.glob(base_dir + '28*')[0]
    device_file = device_folder + '/w1_slave'
    return device_file

def read_temperature(device_file):
    """Read temperature from DS18B20 sensor."""
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

# ============== CONFIGURATION ==============
# Hardware Configuration
KEITHLEY_VENDOR_ID = 0x05E6
KEITHLEY_PRODUCT_ID = 0x2450
ACTION_VOLTAGE_THRESHOLD = 0.5  # Voltage threshold for robot action
STEP_INTERVAL = 4.0  # Time between steps if no threshold crossing
TOTAL_STEPS = None  # Set to None for infinite operation

# PID Control Configuration
PID_SAMPLE_TIME = 0.1  # Control loop timing
PID_KP = 1.2  # Proportional gain
PID_KI = 0.01  # Integral gain
PID_KD = 0.005  # Derivative gain
PID_OUTPUT_LIMITS = (0, 5)  # Output voltage limits for heater

# ============== THREAD-SAFE DATA STRUCTURES ==============
voltage_queue = queue.Queue()
temperature_queue = queue.Queue()
control_queue = queue.Queue()
stop_event = threading.Event()
threshold_crossed_event = threading.Event()
last_action = None
log_lock = threading.Lock()
log_data = []

# ============== SIMULATION CLASSES ==============
class TemperatureField:
    """Simulates a temperature field in 2D space."""
    def __init__(self, func: Callable[[float, float], float], size: float = 100.0):
        self.func = func
        self.size = size

    def temperature(self, x: float, y: float) -> float:
        return self.func(x, y)

    @staticmethod
    def linear(size: float = 100.0, base: float = 0.0, grad_x: float = 0.0, grad_y: float = 0.0):
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
    arc_angle_degrees: float = 10.0
    turn_angle: float = 1.0
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

# ============== THREAD FUNCTIONS ==============
def voltage_measurement_thread(keithley, stop_event):
    """Continuously measures voltage and detects threshold crossings."""
    global last_action
    consecutive_errors = 0
    max_consecutive_errors = 3
    
    try:
        while not stop_event.is_set():
            try:
                # Read voltage with minimal timeout
                keithley.timeout = 50  # 0.05 second timeout
                voltage = float(keithley.ask(":MEAS:VOLT?"))
                voltage_queue.put(voltage)
                
                # Reset error counter on successful read
                consecutive_errors = 0
                
                # Check for threshold crossing
                current_action = 1 if voltage > ACTION_VOLTAGE_THRESHOLD else 0
                if last_action is not None and current_action != last_action:
                    threshold_crossed_event.set()
                last_action = current_action
                
                time.sleep(0.01)  # Minimal delay
                
            except (UsbtmcException, IOError) as e:
                consecutive_errors += 1
                print(f"Error reading voltage (attempt {consecutive_errors}/{max_consecutive_errors}): {e}")
                
                if consecutive_errors >= max_consecutive_errors:
                    print("Too many consecutive errors. Attempting to reconnect...")
                    try:
                        # Try to reset the connection
                        keithley.write("*RST")
                        keithley.write(":OUTP OFF")
                        time.sleep(1)  # Give it time to reset
                        
                        # Test the connection
                        keithley.ask("*IDN?")
                        print("Successfully reconnected to Keithley")
                        consecutive_errors = 0
                    except Exception as reconnect_error:
                        print(f"Failed to reconnect: {reconnect_error}")
                        time.sleep(2)  # Longer delay before next attempt
                else:
                    time.sleep(0.5)  # Short delay before retry
                    
            except Exception as e:
                print(f"Unexpected error in voltage measurement: {e}")
                time.sleep(0.5)
                
    except Exception as e:
        print(f"Fatal error in voltage measurement thread: {e}")

def temperature_control_thread(keithley, stop_event):
    """Maintains temperature using PID control."""
    try:
        while not stop_event.is_set():
            try:
                # Read and process temperature
                current_temp = read_temperature(device_file)
                temperature_queue.put(current_temp)
                
                # Calculate and apply control signal
                control_signal = pid(current_temp)
                keithley.timeout = 50
                keithley.write(f":SOUR:VOLT {control_signal}")
                keithley.write(":OUTP ON" if control_signal > 0 else ":OUTP OFF")
                
                control_queue.put(control_signal)
                time.sleep(PID_SAMPLE_TIME)
            except UsbtmcException as e:
                print(f"Error in temperature control: {e}")
                try:
                    keithley.write("*RST")
                    keithley.write(":OUTP OFF")
                except:
                    pass
                time.sleep(0.5)
            except Exception as e:
                print(f"Unexpected error in temperature control: {e}")
                time.sleep(0.5)
    except Exception as e:
        print(f"Fatal error in temperature control thread: {e}")

def set_simulation_steps(num_steps: Optional[int]):
    """Set the number of steps for the simulation.
    
    Args:
        num_steps: Number of steps to run. Set to None for infinite operation.
    """
    global TOTAL_STEPS
    TOTAL_STEPS = num_steps
    print(f"Simulation steps set to: {'Infinite' if num_steps is None else num_steps}")

def main_control_loop(keithley, robot, field, stop_event):
    """Main control loop coordinating robot movement and data logging."""
    step = 0
    start_time_of_step = time.time()
    start_simulated_temperature = field.temperature(robot.state.x, robot.state.y)
    last_voltage = 0.0  # Keep track of last valid voltage reading

    try:
        while not stop_event.is_set():
            try:
                # Wait for threshold crossing or step interval
                threshold_crossed = threshold_crossed_event.wait(timeout=STEP_INTERVAL)
                threshold_crossed_event.clear()
                
                step += 1
                print(f"\n=== Step {step} ===")
                print(f"Step triggered by: {'Threshold crossing' if threshold_crossed else 'Time interval'}")

                # Get voltage and determine action
                voltage = last_voltage  # Default to last valid reading
                if not voltage_queue.empty():
                    try:
                        voltage = voltage_queue.get()
                        last_voltage = voltage  # Update last valid reading
                    except queue.Empty:
                        print("Warning: Using last valid voltage reading")
                
                current_action = 1 if voltage > ACTION_VOLTAGE_THRESHOLD else 0
                print(f"Measured voltage: {voltage:.4f} V -> Action: {'arc' if current_action == 1 else 'turn and arc'}")

                # Record initial state and temperature BEFORE movement
                start_pos_x, start_pos_y = robot.state.x, robot.state.y
                start_heading = robot.state.heading
                start_simulated_temperature = field.temperature(robot.state.x, robot.state.y)
                print(f"Start temperature at position ({start_pos_x:.2f}, {start_pos_y:.2f}): {start_simulated_temperature:.2f}°C")

                # Update robot position
                if current_action == 1:
                    robot.arc()
                else:
                    robot.turn()
                    robot.arc()

                # Record final state and temperature AFTER movement
                end_pos_x, end_pos_y = robot.state.x, robot.state.y
                end_heading = robot.state.heading
                end_simulated_temperature = field.temperature(robot.state.x, robot.state.y)
                print(f"End temperature at position ({end_pos_x:.2f}, {end_pos_y:.2f}): {end_simulated_temperature:.2f}°C")

                # Update temperature control with new target temperature
                pid.setpoint = end_simulated_temperature
                
                # Get current Peltier temperature
                current_temp = read_temperature(device_file)
                print(f"Current Peltier temperature: {current_temp:.2f}°C")
                
                control_signal = control_queue.get() if not control_queue.empty() else 0.0

                # Print step details
                print_step_details(step, start_time_of_step, voltage, current_action,
                                 start_simulated_temperature, end_simulated_temperature,
                                 current_temp, start_pos_x, start_pos_y, start_heading,
                                 end_pos_x, end_pos_y, end_heading, control_signal,
                                 threshold_crossed)

                # Log data
                log_step_data(step, start_time_of_step, voltage, current_action,
                            start_simulated_temperature, end_simulated_temperature,
                            current_temp, start_pos_x, start_pos_y, start_heading,
                            end_pos_x, end_pos_y, end_heading, control_signal,
                            threshold_crossed)

                # Update for next step
                start_time_of_step = time.time()

                # Check step limit
                if TOTAL_STEPS is not None and step >= TOTAL_STEPS:
                    print(f"\nReached total steps ({TOTAL_STEPS}). Stopping simulation.")
                    stop_event.set()
                    break

            except Exception as e:
                print(f"Error in main control loop step: {e}")
                time.sleep(1)

    except Exception as e:
        print(f"Fatal error in main control loop: {e}")
        stop_event.set()

def print_step_details(step, start_time, voltage, action, start_temp, end_temp,
                      current_temp, start_x, start_y, start_heading,
                      end_x, end_y, end_heading, control_signal, threshold_crossed):
    """Print detailed information about the current step."""
    print("\nStep Details:")
    print(f"Time: {time.time():.2f}")
    print(f"Time At Start of Step: {start_time:.2f}")
    print(f"Voltage Reading: {voltage:.4f} V")
    print(f"Action Taken: {'arc' if action == 1 else 'turn and arc'}")
    print(f"Start Temperature: {start_temp:.2f}°C")
    print(f"End Temperature: {end_temp:.2f}°C")
    print(f"Current Peltier Temperature: {current_temp:.2f}°C")
    print(f"Start Position: ({start_x:.2f}, {start_y:.2f})")
    print(f"Start Heading: {start_heading:.2f}°")
    print(f"End Position: ({end_x:.2f}, {end_y:.2f})")
    print(f"End Heading: {end_heading:.2f}°")
    print(f"Heater Output Value: {control_signal:.2f} V")
    print(f"Step Interval: {STEP_INTERVAL:.1f} s")
    print(f"Voltage Threshold: {ACTION_VOLTAGE_THRESHOLD:.2f} V")
    print(f"Triggered By: {'Threshold crossing' if threshold_crossed else 'Time interval'}")

def log_step_data(step, start_time, voltage, action, start_temp, end_temp,
                 current_temp, start_x, start_y, start_heading,
                 end_x, end_y, end_heading, control_signal, threshold_crossed):
    """Log data for the current step."""
    with log_lock:
        log_entry = {
            'Time': time.time(),
            'Step': step,
            'Time At Start of Step': start_time,
            'Voltage Reading': voltage,
            'Action Taken': action,
            'Start Temperature': start_temp,
            'End Temperature': end_temp,
            'Current Peltier Temperature': current_temp,
            'Start Position X': start_x,
            'Start Position Y': start_y,
            'Start Heading': start_heading,
            'End Position X': end_x,
            'End Position Y': end_y,
            'End Heading': end_heading,
            'Heater Output Value': control_signal,
            'STEP_INTERVAL (Exp)': STEP_INTERVAL,
            'ACTION_VOLTAGE_THRESHOLD (Exp)': ACTION_VOLTAGE_THRESHOLD,
            'Triggered By': 'Threshold crossing' if threshold_crossed else 'Time interval'
        }
        log_data.append(log_entry)

# ============== MAIN PROGRAM ==============
def main():
    print("Initializing hardware and simulation...")
    
    try:
        # Set number of steps (can be changed before running)
        set_simulation_steps(100)  # Set to None for infinite operation
        print("Step count initialized")
        
        # Initialize temperature sensor
        global device_file
        try:
            device_file = setup_temperature_sensor()
            print("Temperature sensor initialized")
        except Exception as e:
            print(f"Error initializing temperature sensor: {e}")
            sys.exit(1)
        
        # Initialize Keithley
        keithley = None
        try:
            print("Attempting to connect to Keithley instrument...")
            keithley = usbtmc.Instrument(KEITHLEY_VENDOR_ID, KEITHLEY_PRODUCT_ID)
            keithley.timeout = 2000
            print(f"Keithley instrument connected (VID: {hex(KEITHLEY_VENDOR_ID)}, PID: {hex(KEITHLEY_PRODUCT_ID)}).")
            
            # Test communication with Keithley
            print("Testing Keithley communication...")
            try:
                idn = keithley.ask("*IDN?")
                print(f"Instrument Identification: {idn.strip()}")
            except Exception as e:
                print(f"Warning: Could not query instrument ID: {e}")
            
            # Initialize instrument
            print("Initializing Keithley settings...")
            keithley.write("*RST")
            keithley.write(":OUTP OFF")
            keithley.write(":SOUR:VOLT 0")
            keithley.write(":SENS:VOLT:PROT 5")
            keithley.write(":SENS:CURR:PROT 1")
            print("Keithley settings initialized")
                
        except Exception as e:
            print(f"Error initializing Keithley: {e}")
            sys.exit(1)

        # Initialize simulation
        print("Initializing simulation...")
        try:
            field = TemperatureField.linear(size=100, base=20, grad_x=1, grad_y=0)
            initial_state = RobotState(x=10.0, y=10.0, heading=0.0)
            robot = Robot(state=initial_state, step_length=1, arc_angle_degrees=1, turn_angle=45)
            print("Robot and field initialized")
        except Exception as e:
            print(f"Error initializing simulation: {e}")
            sys.exit(1)
        
        # Initialize PID controller
        print("Initializing PID controller...")
        try:
            global pid
            pid = PID(Kp=PID_KP, Ki=PID_KI, Kd=PID_KD)
            pid.output_limits = PID_OUTPUT_LIMITS
            pid.setpoint = field.temperature(robot.state.x, robot.state.y)
            print("PID controller initialized")
        except Exception as e:
            print(f"Error initializing PID controller: {e}")
            sys.exit(1)
        
        print("All systems initialized successfully.")
        print("Starting main control loop...")

        # Create and start threads
        threads = []
        try:
            # Start voltage measurement thread
            print("Starting voltage measurement thread...")
            voltage_thread = threading.Thread(
                target=voltage_measurement_thread,
                args=(keithley, stop_event)
            )
            threads.append(voltage_thread)
            voltage_thread.start()
            print("Voltage measurement thread started")

            # Start temperature control thread
            print("Starting temperature control thread...")
            temp_thread = threading.Thread(
                target=temperature_control_thread,
                args=(keithley, stop_event)
            )
            threads.append(temp_thread)
            temp_thread.start()
            print("Temperature control thread started")

            # Run main control loop
            print("Entering main control loop...")
            main_control_loop(keithley, robot, field, stop_event)

        except KeyboardInterrupt:
            print("\nProgram interrupted by user.")
            stop_event.set()
        except Exception as e:
            print(f"Fatal error in main program: {e}")
            stop_event.set()
        finally:
            # Wait for threads to finish
            print("Waiting for threads to finish...")
            for thread in threads:
                thread.join(timeout=5.0)

            # Cleanup
            print("Cleaning up...")
            try:
                if keithley is not None:
                    keithley.write(":OUTP OFF")
                    print("Keithley output turned off.")
            except Exception as e:
                print(f"Error during cleanup: {e}")

            # Save logged data
            if log_data:
                try:
                    log_df = pd.DataFrame(log_data)
                    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
                    filename = f"robot_simulation_log_usbtmc_{timestamp_str}.csv"
                    log_df.to_csv(filename, index=False)
                    print(f"Data saved to {filename}")
                except Exception as e:
                    print(f"Error saving log data: {e}")
            else:
                print("No data to save.")

            print("Program finished.")
    except Exception as e:
        print(f"Critical error in main program: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()



