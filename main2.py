from pymeasure.instruments.keithley import Keithley2450
import pyvisa
import time

# Force use of pyvisa-py backend
rm = pyvisa.ResourceManager('@py')

# Get VISA address
resources = rm.list_resources()
if not resources:
    raise RuntimeError("No VISA devices found.")

print("Connected devices:", resources)
keithley = Keithley2450(rm.open_resource(resources[0]))

# Clear and identify
print("ID:", keithley.id)

# Configure source/measure
keithley.reset()
keithley.use_front_terminals()  # Optional: ensure correct terminals

# Source voltage mode
keithley.source_mode = "voltage"
keithley.voltage = 1.5          # Set output voltage to 1.5 V
keithley.compliance_current = 0.01  # 10 mA limit
keithley.enable_output()

# Wait to stabilize
time.sleep(1)

# Measure
v = keithley.voltage
i = keithley.current
print(f"Measured Voltage: {v:.6f} V")
print(f"Measured Current: {i:.6f} A")

# Disable output
keithley.disable_output()
keithley.shutdown()
