import time

usbtmc_device = "/dev/usbtmc0"

def write_cmd(cmd):
    with open(usbtmc_device, 'w') as f:
        f.write(cmd + '\n')

def read_resp():
    with open(usbtmc_device, 'r') as f:
        return f.read()

# --- Initialization ---
write_cmd("*RST")
write_cmd("*CLS")

# --- Configure source sweep ---
write_cmd("SOUR:FUNC VOLT")
write_cmd("SOUR:VOLT:MODE SWE")
write_cmd("SOUR:VOLT:STAR 0")
write_cmd("SOUR:VOLT:STOP 5")
write_cmd("SOUR:VOLT:STEP 0.5")
write_cmd("SOUR:SWE:SPAC LIN")
write_cmd("SOUR:SWE:RANG AUTO")

# --- Configure measurement ---
write_cmd("SENS:FUNC 'CURR'")
write_cmd("SENS:CURR:PROT 0.1")
write_cmd("SENS:CURR:RANG:AUTO ON")

# --- Configure trigger ---
points = int((5 - 0) / 0.5) + 1
write_cmd(f"TRIG:COUN {points}")
write_cmd("TRIG:SOUR IMM")

# --- Trace buffer setup ---
write_cmd("TRAC:CLE")
write_cmd(f"TRAC:POIN {points}")
write_cmd("TRAC:FEED SENS")
write_cmd("TRAC:FEED:CONT NEXT")

# --- Start sweep and time it ---
write_cmd("OUTP ON")
start = time.time()

write_cmd("INIT")

# Wait for sweep to finish: conservative wait or poll *OPC?
time.sleep(2)  # Or replace with polling logic if needed

write_cmd("TRAC:DATA?")
data = read_resp()

end = time.time()
duration = end - start

# --- Process results ---
currents = [float(x) for x in data.strip().split(',')]
voltages = [0 + i * 0.5 for i in range(points)]

print("Voltage (V) | Current (A)")
for v, c in zip(voltages, currents):
    print(f"{v:.2f}        | {c:.6e}")

print(f"\nSweep duration: {duration:.3f} seconds")

# --- Cleanup ---
write_cmd("OUTP OFF")
