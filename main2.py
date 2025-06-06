cat > keithley_idn.py << 'EOF'
#!/usr/bin/env python3
import sys

# Open the USBTMC device
try:
    smu = open("/dev/usbtmc0", "rb+", buffering=0)
except Exception as e:
    print("ERROR opening /dev/usbtmc0:", e)
    sys.exit(1)

def write(cmd):
    # Append newline and send
    smu.write((cmd + "\n").encode())

def query(cmd):
    write(cmd)
    return smu.read(1024).decode().strip()

# Send *IDN? and print response
idn = query("*IDN?")
print("Instrument ID:", idn)

# Close file
smu.close()
EOF
