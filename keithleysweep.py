import time

usbtmc_device = "/dev/usbtmc0"

def query_cmd(cmd):
    """Send a query and read the result back correctly."""
    with open(usbtmc_device, 'w') as f_w, open(usbtmc_device, 'r') as f_r:
        f_w.write(cmd + '\n')
        f_w.flush()
        time.sleep(0.1)  # Short pause to let it execute
        return f_r.read().strip()

def write_cmd(cmd):
    with open(usbtmc_device, 'w') as f:
        f.write(cmd + '\n')
    print(f"wrote: {cmd}")
    #time.sleep(1)

def read_resp():
    with open(usbtmc_device, 'r') as f:
        return f.read()

# --- Calculate number of steps ---
v_start = 0.0
v_stop = 1.0
v_step = 0.1
delay_per_step = 0.5  # seconds
points = int((v_stop - v_start) / v_step) + 1

# --- Instrument setup ---
write_cmd("*RST")
write_cmd("*CLS")

# Source sweep config
write_cmd("SOUR:FUNC VOLT")

# Measurement config
write_cmd("SENS:FUNC \"CURR\"")
write_cmd("SOUR:VOLT:ILIM 1")
write_cmd("SOUR:VOLT:RANG 20")
# write_cmd("SENS:CURR:RANG:AUTO ON")

write_cmd(f"SOUR:SWE:VOLT:LIN:STEP {v_start}, {v_stop}, {v_step}, {delay_per_step}, 1, FIX")

# Trace buffer config

# Trigger config

write_cmd(":TRIG:CONT OFF")
write_cmd(":INIT:IMM")

# --- Start sweep and time it ---
write_cmd("OUTP ON")
start = time.time()
write_cmd("INIT")

# Wait for sweep completion using *OPC? (optional)
time.sleep(points * delay_per_step)

query_cmd("*OPC?")

write_cmd(":READ? \"SOUR\"")
print(f"read voltage: {read_resp()}")

end = time.time()
duration = end - start

# --- Retrieve and display data ---
print(f"\nTotal sweep duration: {duration:.1f} seconds")

# --- Cleanup ---
write_cmd("OUTP OFF")
