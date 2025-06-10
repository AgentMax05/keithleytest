# --- Imports and Setup ---
import usbtmc
import time
import sys
# Import the specific exception class
from usbtmc.usbtmc import UsbtmcException

# --- Configuration ---
# Your actual Keithley Vendor ID and Product ID from lsusb
KEITHLEY_VENDOR_ID = 0x05E6
KEITHLEY_PRODUCT_ID = 0x2450

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
    # Initialize a single instrument object for the Keithley using YOUR VID and PID
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
    print(f"3. You have permissions to access the USB device (check udev rules for VID: {hex(KEITHLEY_VENDOR_ID)}, PID: {hex(KEITHLEY_PRODUCT_ID)}).")
    sys.exit(1)
except Exception as e: # Catch other potential initialization errors
    print(f"An unexpected error occurred during hardware initialization: {e}")
    sys.exit(1)

# ... rest of the minimal code (main loop, finally block) ...
