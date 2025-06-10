
# Raspberry Pi Controller Code - Single Threaded with Timed Steps (No Continuous Temp Control) - Using usbtmc (Updated)

# %%
# --- Imports and Setup ---
import usbtmc # Import usbtmc
import time
import numpy as np
import pandas as pd # For logging data
import sys # To handle graceful exit
# import matplotlib.pyplot as plt # Optional: For plotting
# import matplotlib.cm as cm # Optional: For plotting

from dataclasses import dataclass
from typing import Callable, List, Optional

# Import the specific exception class for usbtmc errors
from usbtmc.usbtmc import UsbtmcException

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

# --- Data Logging Setup ---
log_data = [] # List to store data for logging

# %%
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
    arc_angle_degrees: float = 1.0  # degrees - New attribute for arc turn angle
    turn_angle: float = 1.0  # degrees
    initial_state: RobotState = None

    def __post_init__(self):
        if self.initial_state is None:
            # Ensure initial_state heading is also in degrees
            self.initial_state = RobotState(self.state.x, self.state.y, self.state.heading)

    # --- motion primitives --------------------------------------------------
    def _forward(self):
        # Convert heading from degrees to radians for trigonometric functions
        heading_rad = np.deg2rad(self.state.heading)
        self.state.x += self.step_length * np.cos(heading_rad)
        self.state.y += self.step_length * np.sin(heading_rad)

    def arc(self):
        self._forward() #moves forward before the turn
        # Directly add the defined arc_angle_degrees to the current heading
        self.state.heading += self.arc_angle_degrees
        # Ensure heading stays within [0, 360) range
        self.state.heading = self.state.heading % 360

    def turn(self):
        # Add turn_angle (in degrees) to the current heading (in degrees)
        self.state.heading += self.turn_angle
        # Ensure heading stays within [0, 360) range (optional but good practice)
        self.state.heading = self.state.heading % 360


    def reset(self):
        # Ensure state heading is reset in degrees
        self.state = RobotState(self.initial_state.x, self.initial_state.y, self.initial_state.heading)


# %%
# -------------------------------------------------
# Hardware Initialization
# -------------------------------------------------
print("Initializing hardware using usbtmc...")
keithley = None # Initialize keithley to None
try:
    # Initialize a single instrument object for the Keithley using VID and PID
    keithley = usbtmc.Instrument(KEITHLEY_VENDOR_ID, KEITHLEY_PRODUCT_ID)
    print(f"Keithley instrument connected (VID: {hex(KEITHLEY_VENDOR_ID)}, PID: {hex(KEITHLEY_PRODUCT_ID)}).")

    # Optional: Query identity to confirm communication
    try:
        idn = keithley.ask("*IDN?")
        print(f"Instrument Identification: {idn.strip()}")
    except UsbtmcException as e: # Catch the correct usbtmc exception
        print(f"usbtmc error querying *IDN?: {e}")
    except Exception as e:
        print(f"An unexpected error occurred querying *IDN?: {e}")

    # You might want to add initial Keithley configuration here
    # Example: Set to measure DC Voltage
    # keithley.write(':SENSE:FUNCTION "VOLTAGE:DC"')

except UsbtmcException as e: # Catch the correct usbtmc exception during initialization
    print(f"Error connecting to Keithley using usbtmc: {e}")
    print("Please ensure:")
    print("1. python-usbtmc is installed.")
    print("2. The Keithley is connected via USB.")
    print(f"3. The Vendor ID ({hex(KEITHLEY_VENDOR_ID)}) and Product ID ({hex(KEITHLEY_PRODUCT_ID)}) in the script are correct.")
    print("   You can find VID/PID using the 'lsusb' command in the terminal.")
    print("4. You have permissions to access the USB device (check udev rules).")
    sys.exit(1)
except Exception as e: # Catch other potential initialization errors
    print(f"An unexpected error occurred during hardware initialization: {e}")
    sys.exit(1)


# %%
# -------------------------------------------------
# Simulation Initialization
# -------------------------------------------------
print("Initializing simulation...")
# Create instances of your simulation objects
field = TemperatureField.linear(size=100, base=20, grad_x=1, grad_y=0) # Adjust parameters as needed
initial_state = RobotState(x=10.0, y=10.0, heading=0.0) # Set initial state
robot = Robot(state=initial_state, step_length=1, arc_angle_degrees=1, turn_angle=45) # Set robot parameters

print("Simulation initialized.")

# Capture initial state and other experiment parameters for logging (optional, can do at the end)
# initial_pos_x = robot.initial_state.x
# initial_pos_y = robot.initial_state.y
# initial_heading = robot.initial_state.heading
# initial_simulated_temp = field.temperature(initial_pos_x, initial_pos_y)

# Capture temperature field parameters for logging (optional)
# field_size = field.size
# field_base_temp = field.func(0, 0)
# field_grad_x = 1
# field_grad_y = 0

# Capture robot parameters for logging (optional)
# robot_step_length = robot.step_length
# robot_arc_angle_degrees = robot.arc_angle_degrees
# robot_turn_angle = robot.turn_angle

# %%
# -------------------------------------------------
# Main Control Loop (Single Threaded with Timed Steps)
# -------------------------------------------------
print(f"Starting main control loop with a step interval of {STEP_INTERVAL} seconds. Press Ctrl+C to exit early.")

# Keep track of the temperature at the *start* of the current step
start_simulated_temperature_this_step = field.temperature(robot.state.x, robot.state.y)

# Initialize variables for timing and action history
step = 0
start_time_of_step = time.time() # Record the start time of the first step
previous_action = -1 # To detect action changes (0 to 1)

# Use a while True loop to run continuously until interrupted or total steps reached
try: # Wrap the main loop in a try block for cleanup
    while True:
        try: # Inner try for step-specific errors
            step += 1
            print(f"\n--- Step {step} ---")

            # --- Timing Control ---
            # Calculate how long the previous step took and how long to wait
            end_time_of_previous_step = time.time()
            elapsed_time_in_previous_step = end_time_of_previous_step - start_time_of_step
            time_to_wait = max(0, STEP_INTERVAL - elapsed_time_in_previous_step)

            print(f"Elapsed time in previous step: {elapsed_time_in_previous_step:.2f} s")
            print(f"Waiting for {time_to_wait:.2f} seconds before starting step {step} actions...")
            time.sleep(time_to_wait) # This is the blocking delay
            start_time_of_step = time.time() # Record the start time of the current step
            print(f"Starting step {step} actions.")


            # Capture state at the start of the step *after* the wait
            start_pos_x_this_step = robot.state.x
            start_pos_y_this_step = robot.state.y
            start_heading_this_step = robot.state.heading
            # start_simulated_temperature_this_step is carried over from the end of the previous step


            # 1. Read action from physical robot (via the single Keithley)
            # This happens after the calculated delay for the step interval
            try:
                # Configure Keithley for voltage measurement (if necessary)
                # keithley.write(':SENSE:FUNCTION "VOLTAGE:DC"')

                voltage = float(keithley.ask(":MEAS:VOLT?"))

                # Determine current action based on voltage
                if voltage > ACTION_VOLTAGE_THRESHOLD:
                    current_action = 1 # arc
                else:
                    current_action = 0 # turn and arc

                print(f"Measured voltage: {voltage:.4f} V -> Robot action: {'arc' if current_action == 1 else 'turn and arc'}")

            except UsbtmcException as e: # Catch the correct usbtmc exception
                 print(f"Step {step}: usbtmc error reading voltage from Keithley: {e}")
                 # Decide how to handle this error: log, skip this step, break
                 # For now, let's log the error and skip the rest of this step's actions
                 log_entry = {
                     'Time': time.time(),
                     'Step': step,
                     'Time At Start of Step': start_time_of_step,
                     'Voltage Reading': 'Error',
                     'Action Taken': 'Skipped due to error',
                     'Start Temperature': start_simulated_temperature_this_step,
                     'End Temperature': 'N/A', # Temperature update skipped
                     'Start Position X': start_pos_x_this_step,
                     'Start Position Y': start_pos_y_this_step,
                     'Start Heading': start_heading_this_step,
                     'End Position X': start_pos_x_this_step, # Positions didn't change
                     'End Position Y': start_pos_y_this_step,
                     'End Heading': start_heading_this_step,
                     'Heater Output Value': 'N/A', # Heater control skipped
                     # Log experiment parameters if needed
                 }
                 log_data.append(log_entry)
                 continue # Skip the rest of the loop body and go to the next iteration
            except Exception as e: # Catch other potential errors during voltage read
                 print(f"Step {step}: An unexpected error occurred during voltage read: {e}")
                 log_entry = {
                     'Time': time.time(),
                     'Step': step,
                     'Time At Start of Step': start_time_of_step,
                     'Voltage Reading': 'Unexpected Error',
                     'Action Taken': 'Skipped due to error',
                     'Start Temperature': start_simulated_temperature_this_step,
                     'End Temperature': 'N/A', # Temperature update skipped
                     'Start Position X': start_pos_x_this_step,
                     'Start Position Y': start_pos_y_this_step,
                     'Start Heading': start_heading_this_step,
                     'End Position X': start_pos_x_this_step, # Positions didn't change
                     'End Position Y': start_pos_y_this_step,
                     'End Heading': start_heading_this_step,
                     'Heater Output Value': 'N/A', # Heater control skipped
                     # Log experiment parameters if needed
                 }
                 log_data.append(log_entry)
                 continue # Skip the rest of the loop body and go to the next iteration


            # 2. Update robot state in the simulation based on the action
            # This happens after the voltage is read
            action_taken = current_action # Log the action determined by voltage
            if current_action == 1:
                robot.arc()
            else:
                # Assuming 'turn and arc' is a turn followed by an arc
                robot.turn()
                robot.arc()

            # Capture state at the end of the step
            end_pos_x_this_step = robot.state.x
            end_pos_y_this_step = robot.state.y
            end_heading_this_step = robot.state.heading

            # print(f"Simulated position after action: ({robot.state.x:.2f}, {robot.state.y:.2f}), Heading: {robot.state.heading:.2f}")


            # 3. Get simulated temperature at the *end* position
            end_simulated_temperature_this_step = field.temperature(robot.state.x, robot.state.y)
            # print(f"Simulated temperature: {end_simulated_temperature_this_step:.2f}°C")


            # 4. Heater Control (Simple example - not a continuous loop)
            # This happens once per step
            try:
                # Implement a simple heater control logic here
                # Example: Turn on heater if simulated temperature is below a threshold
                HEATER_ON_THRESHOLD = 110.0 # Example threshold
                HEATER_VOLTAGE = 3.0 # Example voltage when heater is on

                heater_output_value = 0.0 # Default to off
                if end_simulated_temperature_this_step < HEATER_ON_THRESHOLD:
                     heater_output_value = HEATER_VOLTAGE
                     print(f"Simulated temperature ({end_simulated_temperature_this_step:.2f}°C) below threshold. Turning heater ON.")
                else:
                     print(f"Simulated temperature ({end_simulated_temperature_this_step:.2f}°C) above threshold. Turning heater OFF.")


                # Configure Keithley for voltage sourcing
                # keithley.write(':SOURCE:FUNCTION "VOLTAGE"')
                keithley.write(f":SOUR:VOLT {heater_output_value}")
                if heater_output_value > 0:
                     keithley.write(":OUTP ON")
                else:
                     keithley.write(":OUTP OFF")


            except UsbtmcException as e: # Catch the correct usbtmc exception
                 print(f"Step {step}: usbtmc error controlling heater with Keithley: {e}")
                 # Attempt to turn off heater output on error
                 try:
                      if keithley is not None: # Add check if keithley object exists
                           keithley.write(":OUTP OFF")
                 except Exception as e_off:
                       print(f"Error turning off Keithley output after heater control error: {e_off}")
            except Exception as e: # Catch other potential errors during heater control
                print(f"Step {step}: An unexpected error occurred during heater control: {e}")
                # Attempt to turn off heater output on error
                try:
                    if keley is not None: # Add check if keithley object exists
                         keithley.write(":OUTP OFF")
                except Exception as e_off:
                     print(f"Error turning off Keithley output after unexpected heater control error: {e_off}")


            # 5. Log Data
            log_entry = {
                'Time': time.time(), # Time of logging
                'Step': step,
                'Time At Start of Step': start_time_of_step, # Log the actual start time of the step
                'Voltage Reading': voltage,
                'Action Taken': action_taken,
                'Start Temperature': start_simulated_temperature_this_step,
                'End Temperature': end_simulated_temperature_this_step,
                'Start Position X': start_pos_x_this_step,
                'Start Position Y': start_pos_y_this_step, # Fix typo here
                'Start Heading': start_heading_this_step,
                'End Position X': end_pos_x_this_step,
                'End Position Y': end_pos_y_this_step,
                'End Heading': end_heading_this_step,
                'Heater Output Value': heater_output_value,
                 # Include experiment parameters here as well
                 'STEP_INTERVAL (Exp)': STEP_INTERVAL,
                 'ACTION_VOLTAGE_THRESHOLD (Exp)': ACTION_VOLTAGE_THRESHOLD,
                 # Add other relevant experiment parameters for logging
            }
            log_data.append(log_entry)

            # Prepare for the next step: the end temperature of this step is the start temperature of the next
            start_simulated_temperature_this_step = end_simulated_temperature_this_step

            # Check for the total number of steps limit
            if TOTAL_STEPS is not None and step >= TOTAL_STEPS:
                print("Total steps reached.")
                break # Exit the inner while loop

        except KeyboardInterrupt:
            print("\nControl loop interrupted by user.")
            break # Exit the inner while loop

        except Exception as e: # Catch any other unexpected errors within the step
            print(f"\nStep {step}: An unexpected error occurred within the step: {e}")
            # Decide how to handle this error - for now, break the outer loop
            break # Exit the inner while loop

except Exception as outer_e: # Catch any errors that escaped the inner try/except
     print(f"\nAn unhandled error occurred outside the step loop: {outer_e}")


finally:
    # -------------------------------------------------
    # Data Logging and Cleanup (Outside the Loop)
    # -------------------------------------------------
    print("Cleaning up and saving data...")

    # Turn off Keithley output and close the connection
    try:
        if keithley is not None: # Check if keithley object was successfully created
            keithley.write(":OUTP OFF")
            print("Keithley output turned off.")
            # Note: usbtmc.Instrument objects don't typically have a .close() method
            # in the same way pyvisa resources do for basic USB-TMC.
            # If your usbtmc library version requires explicit closing, add it here.
            # Otherwise, the object will be garbage collected.
    except Exception as e:
        print(f"Error during Keithley cleanup: {e}")

    # Save logged data to a CSV file
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
