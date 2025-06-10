# Raspberry Pi Controller Code - Multithreaded with PID Temperature Control - Using usbtmc

# --- Imports and Setup ---
import usbtmc # Import usbtmc
import time
import numpy as np
import pandas as pd # For logging data
import sys # To handle graceful exit
import threading
import queue
from dataclasses import dataclass
from typing import Callable, List, Optional
from usbtmc.usbtmc import UsbtmcException
from simple_pid import PID
import os
import glob

# DS18B20 Temperature Sensor Setup
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

# Find the DS18B20 sensor
base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

def read_temp_raw():
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

def read_temp():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        return temp_c

# --- Configuration ---
# Replace with your actual Keithley Vendor ID and Product ID (find using lsusb)
KEITHLEY_VENDOR_ID = 0x05E6
KEITHLEY_PRODUCT_ID = 0x2450

# Define the voltage threshold for robot action (adjust based on your robot's output)
ACTION_VOLTAGE_THRESHOLD = 0.5 # Example threshold

# Define the desired interval between steps (in seconds)
STEP_INTERVAL = 4.0 # Example: Aim for a step every 4 seconds

# Define the total number of steps to run (Set to None for indefinite loop)
TOTAL_STEPS = 100 # Example: Run for 100 steps, or None for infinite

# PID Control Parameters
TARGET_TEMPERATURE = 30.0  # Target temperature in Celsius
PID_SAMPLE_TIME = 0.5  # PID control loop sample time in seconds
PID_KP = 1.2  # Proportional gain
PID_KI = 0.01  # Integral gain
PID_KD = 0.005  # Derivative gain
PID_OUTPUT_LIMITS = (0, 0.5)  # Output voltage limits for the heater

# Thread-safe data structures
voltage_queue = queue.Queue()  # Queue for voltage measurements
temperature_queue = queue.Queue()  # Queue for temperature measurements
control_queue = queue.Queue()  # Queue for control signals
stop_event = threading.Event()  # Event to signal threads to stop

# Create global PID controller
pid = PID(Kp=PID_KP, Ki=PID_KI, Kd=PID_KD, setpoint=TARGET_TEMPERATURE)
pid.output_limits = PID_OUTPUT_LIMITS

# --- Data Logging Setup ---
log_data = [] # List to store data for logging
log_lock = threading.Lock()  # Lock for thread-safe logging

# -------------------------------------------------
# Temperature Field (from original code)
# -------------------------------------------------
class TemperatureField:
    """Continuous temperature field on a square [0,size]×[0,size] domain."""

    def __init__(self, func: Callable[[float, float], float], size: float = 100.0):
        self.func = func
        self.size = size

    def temperature(self, x: float, y: float) -> float:
        return self.func(x, y)

    @staticmethod
    def linear(size: float = 100.0, base: float = 0.0, grad_x: float = 0.0, grad_y: float = 0.0):
        return TemperatureField(lambda x, y: base + grad_x * x + grad_y * y, size)

# -------------------------------------------------
# Robot (from original code)
# -------------------------------------------------
@dataclass
class RobotState:
    x: float
    y: float
    heading: float  # degrees

@dataclass
class Robot:
    state: RobotState
    step_length: float = 1.0
    arc_angle_degrees: float = 10.0  # degrees
    turn_angle: float = 1.0  # degrees
    initial_state: RobotState = None

    def __post_init__(self):
        if self.initial_state is None:
            self.initial_state = RobotState(self.state.x, self.state.y, self.state.heading)

    def _forward(self):
        heading_rad = np.deg2rad(self.state.heading)
        self.state.x += self.step_length * np.cos(heading_rad)
        self.state.y += self.step_length * np.sin(heading_rad)

    def arc(self):
        self._forward()
        self.state.heading += self.arc_angle_degrees
        self.state.heading = self.state.heading % 360

    def turn(self):
        self.state.heading += self.turn_angle
        self.state.heading = self.state.heading % 360

    def reset(self):
        self.state = RobotState(self.initial_state.x, self.initial_state.y, self.initial_state.heading)

# -------------------------------------------------
# Thread Functions
# -------------------------------------------------
def voltage_measurement_thread(keithley, stop_event):
    """Thread for continuous voltage measurements."""
    try:
        while not stop_event.is_set():
            try:
                # Try to read voltage with timeout
                keithley.timeout = 2000  # 2 second timeout
                voltage = float(keithley.ask(":MEAS:VOLT?"))
                voltage_queue.put(voltage)
                time.sleep(0.1)  # Small delay to prevent overwhelming the instrument
            except UsbtmcException as e:
                print(f"Error reading voltage: {e}")
                # Try to reset the connection
                try:
                    keithley.write("*RST")
                    keithley.write(":OUTP OFF")
                except:
                    pass
                time.sleep(1)  # Longer delay on error
            except Exception as e:
                print(f"Unexpected error in voltage measurement: {e}")
                time.sleep(1)
    except Exception as e:
        print(f"Fatal error in voltage measurement thread: {e}")

def temperature_control_thread(keithley, stop_event):
    """Thread for PID temperature control."""
    try:
        while not stop_event.is_set():
            try:
                # Read current temperature from DS18B20
                current_temp = read_temp()
                temperature_queue.put(current_temp)
                
                # Calculate control signal
                control_signal = pid(current_temp)
                
                # Apply control signal to heater with timeout
                keithley.timeout = 2000  # 2 second timeout
                keithley.write(f":SOUR:VOLT {control_signal}")
                if control_signal > 0:
                    keithley.write(":OUTP ON")
                else:
                    keithley.write(":OUTP OFF")
                
                # Put control signal in queue for logging
                control_queue.put(control_signal)
                
                time.sleep(PID_SAMPLE_TIME)
            except UsbtmcException as e:
                print(f"Error in temperature control: {e}")
                # Try to reset the connection
                try:
                    keithley.write("*RST")
                    keithley.write(":OUTP OFF")
                except:
                    pass
                time.sleep(1)
            except Exception as e:
                print(f"Unexpected error in temperature control: {e}")
                time.sleep(1)
    except Exception as e:
        print(f"Fatal error in temperature control thread: {e}")

def main_control_loop(keithley, robot, field, stop_event):
    """Main control loop for robot movement and simulation."""
    step = 0
    start_time_of_step = time.time()
    start_simulated_temperature = field.temperature(robot.state.x, robot.state.y)

    try:
        while not stop_event.is_set():
            try:
                step += 1
                print(f"\n--- Step {step} ---")

                # Timing control
                end_time_of_previous_step = time.time()
                elapsed_time = end_time_of_previous_step - start_time_of_step
                time_to_wait = max(0, STEP_INTERVAL - elapsed_time)
                
                print(f"Elapsed time in previous step: {elapsed_time:.2f} s")
                print(f"Waiting for {time_to_wait:.2f} s before starting step {step}...")
                time.sleep(time_to_wait)
                start_time_of_step = time.time()

                # Capture initial state
                start_pos_x = robot.state.x
                start_pos_y = robot.state.y
                start_heading = robot.state.heading

                # Get voltage measurement from queue
                if not voltage_queue.empty():
                    voltage = voltage_queue.get()
                    current_action = 1 if voltage > ACTION_VOLTAGE_THRESHOLD else 0
                    print(f"Measured voltage: {voltage:.4f} V -> Action: {'arc' if current_action == 1 else 'turn and arc'}")
                else:
                    print("No voltage measurement available")
                    continue

                # Update robot state
                if current_action == 1:
                    robot.arc()
                else:
                    robot.turn()
                    robot.arc()

                # Get end state
                end_pos_x = robot.state.x
                end_pos_y = robot.state.y
                end_heading = robot.state.heading
                end_simulated_temperature = field.temperature(robot.state.x, robot.state.y)

                # Update PID setpoint to match simulated temperature
                pid.setpoint = end_simulated_temperature

                # Get current temperature and control signal for logging
                current_temp = read_temp()
                control_signal = 0.0
                if not control_queue.empty():
                    control_signal = control_queue.get()

                # Log data
                with log_lock:
                    log_entry = {
                        'Time': time.time(),
                        'Step': step,
                        'Time At Start of Step': start_time_of_step,
                        'Voltage Reading': voltage,
                        'Action Taken': current_action,
                        'Start Temperature': start_simulated_temperature,
                        'End Temperature': end_simulated_temperature,
                        'Current Peltier Temperature': current_temp,
                        'Start Position X': start_pos_x,
                        'Start Position Y': start_pos_y,
                        'Start Heading': start_heading,
                        'End Position X': end_pos_x,
                        'End Position Y': end_pos_y,
                        'End Heading': end_heading,
                        'Heater Output Value': control_signal,
                        'STEP_INTERVAL (Exp)': STEP_INTERVAL,
                        'ACTION_VOLTAGE_THRESHOLD (Exp)': ACTION_VOLTAGE_THRESHOLD,
                    }
                    log_data.append(log_entry)

                # Update temperature for next step
                start_simulated_temperature = end_simulated_temperature

                # Check step limit
                if TOTAL_STEPS is not None and step >= TOTAL_STEPS:
                    print("Total steps reached.")
                    stop_event.set()
                    break

            except Exception as e:
                print(f"Error in main control loop step: {e}")
                time.sleep(1)

    except Exception as e:
        print(f"Fatal error in main control loop: {e}")
        stop_event.set()

# -------------------------------------------------
# Main Program
# -------------------------------------------------
def main():
    print("Initializing hardware and simulation...")
    
    # Initialize Keithley
    keithley = None
    try:
        keithley = usbtmc.Instrument(KEITHLEY_VENDOR_ID, KEITHLEY_PRODUCT_ID)
        keithley.timeout = 2000  # 2 second timeout
        print(f"Keithley instrument connected (VID: {hex(KEITHLEY_VENDOR_ID)}, PID: {hex(KEITHLEY_PRODUCT_ID)}).")
        
        # Query identity with retry
        max_retries = 3
        for attempt in range(max_retries):
            try:
                idn = keithley.ask("*IDN?")
                print(f"Instrument Identification: {idn.strip()}")
                break
            except Exception as e:
                if attempt == max_retries - 1:
                    print(f"Error querying instrument ID after {max_retries} attempts: {e}")
                else:
                    print(f"Retrying instrument ID query... (Attempt {attempt + 1}/{max_retries})")
                    time.sleep(1)
        
        # Initialize instrument
        keithley.write("*RST")  # Reset to default settings
        keithley.write(":OUTP OFF")  # Ensure output is off initially
        keithley.write(":SOUR:VOLT 0")  # Set initial voltage to 0
        keithley.write(":SENS:VOLT:PROT 5")  # Set voltage protection limit to 5V
        keithley.write(":SENS:CURR:PROT 1")  # Set current protection limit to 1A
            
    except Exception as e:
        print(f"Error initializing Keithley: {e}")
        sys.exit(1)

    # Initialize simulation
    field = TemperatureField.linear(size=100, base=20, grad_x=1, grad_y=0)
    initial_state = RobotState(x=10.0, y=10.0, heading=0.0)
    robot = Robot(state=initial_state, step_length=1, arc_angle_degrees=1, turn_angle=45)
    
    # Set initial PID setpoint based on starting position
    initial_temp = field.temperature(robot.state.x, robot.state.y)
    pid.setpoint = initial_temp
    print(f"Initial target temperature set to {initial_temp:.1f}°C based on robot position")
    
    print("Simulation initialized.")

    # Create and start threads
    threads = []
    try:
        # Start voltage measurement thread
        voltage_thread = threading.Thread(
            target=voltage_measurement_thread,
            args=(keithley, stop_event)
        )
        threads.append(voltage_thread)
        voltage_thread.start()

        # Start temperature control thread
        temp_thread = threading.Thread(
            target=temperature_control_thread,
            args=(keithley, stop_event)
        )
        threads.append(temp_thread)
        temp_thread.start()

        # Run main control loop
        main_control_loop(keithley, robot, field, stop_event)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
        stop_event.set()
    except Exception as e:
        print(f"Fatal error in main program: {e}")
        stop_event.set()
    finally:
        # Wait for threads to finish
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

if __name__ == "__main__":
    main()
