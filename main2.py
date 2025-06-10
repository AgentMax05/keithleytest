# --- Imports and Setup ---
import usbtmc
import time
import sys

# --- Configuration ---
# Replace with your actual Keithley USB address (find this using usbtmc.list_devices() or by checking /dev/)
KEITHLEY_ADDRESS = "/dev/usbtmc0" # Example - REPLACE THIS with your instrument's address

# Define the voltage threshold for triggering an action (adjust as needed)
ACTION_VOLTAGE_THRESHOLD = 0.5 # Example threshold

# Define the desired interval between steps (in seconds)
STEP_INTERVAL = 4.0 # Example: Aim for a step every 4 seconds

# Define the total number of steps to run (Set to None for indefinite loop)
TOTAL_STEPS = 10 # Example: Run for 10 steps, or None for infinite


# -------------------------------------------------
# Hardware Initialization
# -------------------------------------------------
print("Initializing hardware using usbtmc...")
keithley = None # Initialize keithley to None
try:
    # Initialize a single instrument object for the Keithley
    keithley = usbtmc.Instrument(KEITHLEY_ADDRESS)
    print(f"Keithley instrument connected at {KEITHLEY_ADDRESS}.")

    # Optional: Query identity to confirm communication
    try:
        idn = keithley.ask("*IDN?")
        print(f"Instrument Identification: {idn.strip()}")
    except usbtmc.TMCDeviceError as e:
        print(f"usbtmc error querying *IDN?: {e}")
    except Exception as e:
        print(f"An unexpected error occurred querying *IDN?: {e}")


    # You might want to add initial Keithley configuration here
    # Example: Set to measure DC Voltage
    # keithley.write(':SENSE:FUNCTION "VOLTAGE:DC"')

except usbtmc.TMCDeviceError as e:
    print(f"Error connecting to Keithley using usbtmc: {e}")
    print("Please ensure python-usbtmc is installed, the Keithley is connected,")
    print(f"and the device file ({KEITHLEY_ADDRESS}) exists and you have permissions to access it.")
    sys.exit(1)
except Exception as e: # Catch other potential initialization errors
    print(f"An unexpected error occurred during hardware initialization: {e}")
    sys.exit(1)


# -------------------------------------------------
# Main Control Loop (Minimal Single Threaded with Timed Steps)
# -------------------------------------------------
print(f"\nStarting minimal control loop with a step interval of {STEP_INTERVAL} seconds. Press Ctrl+C to exit early.")

# Initialize variables for timing
step = 0
start_time_of_step = time.time() # Record the start time of the first step


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


            # 1. Read voltage from physical Keithley
            try:
                # Configure Keithley for voltage measurement (if necessary)
                # keithley.write(':SENSE:FUNCTION "VOLTAGE:DC"')

                voltage = float(keithley.ask(":MEAS:VOLT?"))

                # Determine and print action based on voltage
                if voltage > ACTION_VOLTAGE_THRESHOLD:
                    action_message = "Action: Arc"
                else:
                    action_message = "Action: Turn and Arc"

                print(f"Measured voltage: {voltage:.4f} V -> {action_message}")

            except usbtmc.TMCDeviceError as e:
                 print(f"Step {step}: usbtmc error reading voltage from Keithley: {e}")
                 # Decide how to handle this error: log, skip this step, break
                 # For now, just print the error and continue to the next timed step
                 continue # Skip the rest of the loop body for this iteration
            except Exception as e: # Catch other potential errors during voltage read
                 print(f"Step {step}: An unexpected error occurred during voltage read: {e}")
                 continue # Skip the rest of the loop body for this iteration


            # --- Minimal Robot Action (Replace with your actual robot movement) ---
            # In a real application, you would control your robot here based on 'current_action'
            print("Performing minimal robot action (simulated).")
            # Example: You might send a command to a motor controller or GPIO


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
    # Cleanup (Outside the Loop)
    # -------------------------------------------------
    print("\nCleaning up...")

    # Turn off Keithley output (if applicable and safe)
    # Be cautious turning off output if the minimal code doesn't explicitly turn it on.
    # Only uncomment and use this if you add code to turn on Keithley output in the loop.
    # try:
    #     if keithley is not None:
    #         keithley.write(":OUTP OFF")
    #         print("Keithley output turned off (attempted).")
    # except Exception as e:
    #     print(f"Error during Keithley cleanup: {e}")

    print("Program finished.")
