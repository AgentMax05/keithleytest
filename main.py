import pyvisa
import time

# --------------------
# Initialization
# --------------------
try:
    # Create VISA resource manager
    rm = pyvisa.ResourceManager()

    # List available instruments
    resources = rm.list_resources()
    if not resources:
        raise RuntimeError("No instruments found. Is the Keithley 2450 connected via USB?")
    
    print("Connected instruments:", resources)
    keithley = rm.open_resource(resources[0])  # Use first found device

    # Optional: print identity
    print("Instrument ID:", keithley.query('*IDN?'))

    # Clear any previous state
    keithley.write('*RST')
    keithley.write('*CLS')

    # --------------------
    # Configure 2450
    # --------------------
    voltage_to_source = 2.0  # Set desired voltage in volts
    current_limit = 0.01     # Compliance current in amps (e.g., 10 mA)

    keithley.write('SOUR:FUNC VOLT')                        # Set to voltage source mode
    keithley.write(f'SOUR:VOLT {voltage_to_source}')        # Set output voltage
    keithley.write(f'SENS:CURR:PROT {current_limit}')       # Set current limit (compliance)
    keithley.write('SENS:FUNC "CURR"')                      # Set sense function to current

    # --------------------
    # Enable Output
    # --------------------
    keithley.write('OUTP ON')
    print(f"Output enabled: sourcing {voltage_to_source} V")

    # Wait for system to stabilize
    time.sleep(1)

    # --------------------
    # Measure Current
    # --------------------
    current = float(keithley.query('MEAS:CURR?').strip())
    print(f"Measured current: {current:.6f} A")

    # --------------------
    # Cleanup
    # --------------------
    keithley.write('OUTP OFF')
    print("Output disabled.")
    keithley.close()

except Exception as e:
    print(f"Error: {e}")
