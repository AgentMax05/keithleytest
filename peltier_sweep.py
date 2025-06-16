import usbtmc
import time
import pandas as pd
from datetime import datetime

import board
import digitalio
import adafruit_max31856

# ============== CONFIGURATION ==============
KEITHLEY_VENDOR_ID = 0x05E6
KEITHLEY_PRODUCT_ID = 0x2450
MAX_CURRENT = 1.0  # Set max current to 1A
WAIT_TIME_AFTER_SET_VOLTAGE = 15.0  # Wait time in seconds after setting voltage

# Temperature sensor configuration (from thermo_test.py)
# Create sensor object, communicating over the board's default SPI bus
try:
    spi = board.SPI()
except NotImplementedError:
    print("SPI not available on this platform. Temperature sensor will not work.")
    spi = None

# allocate a CS pin and set the direction
try:
    cs = digitalio.DigitalInOut(board.D5)
    cs.direction = digitalio.Direction.OUTPUT
except NotImplementedError:
    print("Digital pins not available on this platform. Temperature sensor will not work.")
    cs = None

# create a thermocouple object with the above
thermocouple = None
if spi and cs:
    try:
        thermocouple = adafruit_max31856.MAX31856(spi, cs)
    except RuntimeError as e:
        print(f"Error initializing MAX31856 sensor: {e}")
        thermocouple = None

# ============== HARDWARE INTERFACE ==============
def initialize_keithley():
    """Initialize and configure the Keithley instrument."""
    try:
        keithley = usbtmc.Instrument(KEITHLEY_VENDOR_ID, KEITHLEY_PRODUCT_ID)
        keithley.timeout = 2000  # 2 second timeout

        # Configure Keithley
        keithley.write("*RST")  # Reset to default settings
        keithley.write(":OUTP OFF")  # Turn off output
        keithley.write(":SOUR:VOLT 0")  # Set voltage to 0
        keithley.write(f":SENS:CURR:PROT {MAX_CURRENT}")  # Set current protection to max current

        return keithley
    except Exception as e:
        print(f"Error initializing Keithley: {e}")
        return None

def read_temperature_from_max31856():
    """Read temperature from the MAX31856 sensor."""
    if thermocouple:
        try:
            return thermocouple.temperature
        except RuntimeError as e:
            print(f"Error reading temperature from MAX31856: {e}")
            return None
    else:
        print("MAX31856 sensor not initialized.")
        return None

def main():
    """Main program entry point."""
    keithley = None
    log_data = []

    try:
        # Initialize Keithley
        keithley = initialize_keithley()
        if keithley is None:
            return

        # Define voltage range
        voltage_range = [float(i) / 10.0 for i in range(-5, 6, 1)]
        # voltage_range = [round(v, 2) for v in range(-5, 6, 1)] # -1V to 1V in 0.1V increments

        print("Starting voltage sweep and temperature measurement...")

        # Sweep through voltage range
        for voltage in voltage_range:
            print(f"\nSetting Keithley voltage to: {voltage:.2f}V")
            try:
                keithley.write(f":SOUR:VOLT {voltage}")
                keithley.write(":OUTP ON")

                # Wait for the specified time
                print(f"Waiting for {WAIT_TIME_AFTER_SET_VOLTAGE} seconds...")
                time.sleep(WAIT_TIME_AFTER_SET_VOLTAGE)

                # Read temperature from MAX31856 sensor
                current_temp = read_temperature_from_max31856()

                if current_temp is not None:
                    print(f"Measured Temperature: {current_temp:.2f}°C")

                    # Log data
                    log_data.append({
                        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'),
                        'set_voltage': voltage,
                        'measured_temperature': current_temp
                    })
                else:
                    print("Could not read temperature.")

            except usbtmc.usbtmc.UsbtmcException as e:
                print(f"Error communicating with Keithley: {e}")
                break
            except Exception as e:
                print(f"An unexpected error occurred: {e}")
                break

    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    finally:
        # Turn off Keithley output if initialized
        if keithley:
            try:
                keithley.write(":OUTP OFF")
                print("\nKeithley output turned OFF.")
            except usbtmc.usbtmc.UsbtmcException as e:
                print(f"Error turning off Keithley output: {e}")

        # Save data to CSV
        if log_data:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            df = pd.DataFrame(log_data)
            df.to_csv(f'voltage_temperature_mapping_{timestamp}.csv', index=False)
            print(f"\nData saved to voltage_temperature_mapping_{timestamp}.csv")
        else:
            print("\nNo data was collected.")
        print("\nProgram terminated")

if __name__ == "__main__":
    main()
