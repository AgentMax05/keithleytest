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
#write_cmd("SOUR:VOLT:MODE SWE")
#write_cmd(f"SOUR:VOLT:STAR {v_start}")
#write_cmd(f"SOUR:VOLT:STOP {v_stop}")
#write_cmd(f"SOUR:VOLT:STEP {v_step}")
#write_cmd("SOUR:SWE:SPAC LIN")
#write_cmd("SOUR:SWE:RANG AUTO")
#write_cmd(f"SOUR:SWE:DEL {delay_per_step}")

# Measurement config
write_cmd("SENS:FUNC \"CURR\"")
write_cmd("SOUR:VOLT:ILIM 1")
write_cmd("SOUR:VOLT:RANG 20")
# write_cmd("SENS:CURR:RANG:AUTO ON")

write_cmd(f"SOUR:SWE:VOLT:LIN:STEP {v_start}, {v_stop}, {v_step}, {delay_per_step}, 1, FIX")

# Trace buffer config

#write_cmd("TRAC:CLE")
#write_cmd(f"TRAC:POIN {points}")
#write_cmd("TRAC:FEED SENS")
#write_cmd("TRAC:FEED:CONT NEXT")

# Trigger config

#write_cmd(f"TRIG:COUN {points}")
#write_cmd("TRIG:SOUR IMM")  # Start immediately

write_cmd(":TRIG:CONT OFF")
write_cmd(":INIT:IMM")

#write_cmd(":TRIG:COUN INF") # Use number of sweep points if you want finite count
#write_cmd(":TRIG:SEQ:ADV AUTO")

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
