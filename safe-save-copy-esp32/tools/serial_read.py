#!/usr/bin/env python3
"""Capture ESP32 serial output for a fixed time, non-interactively.

usage: python3 tools/serial_read.py /dev/ttyACM0 [seconds=20] [baud=115200] [--reset]

--reset pulses EN via RTS (works on USB-UART bridge ports with the auto-reset
circuit, and on the ESP32-S3 native USB-Serial/JTAG port) so boot output is captured.
Without --reset, DTR/RTS are held inactive so opening the port does not reset the chip.
"""
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: sudo apt install python3-serial  (or pip install --user pyserial)")

args = [a for a in sys.argv[1:] if not a.startswith("--")]
if not args:
    sys.exit(__doc__)
port = args[0]
secs = float(args[1]) if len(args) > 1 else 20.0
baud = int(args[2]) if len(args) > 2 else 115200

s = serial.Serial()
s.port, s.baudrate, s.timeout = port, baud, 0.2
s.dtr = False
s.rts = False
s.open()
if "--reset" in sys.argv:
    s.rts = True          # EN low
    time.sleep(0.1)
    s.rts = False         # EN high -> boot
end = time.time() + secs
try:
    while time.time() < end:
        line = s.readline()
        if line:
            sys.stdout.write(line.decode(errors="replace"))
            sys.stdout.flush()
finally:
    s.close()
