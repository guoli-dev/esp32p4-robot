import serial
import time

try:
    s = serial.Serial("COM6", 115200, timeout=3)
    print("Port opened successfully!")
    time.sleep(4)
    data = s.read(2000)
    print(f"Read {len(data)} bytes:")
    print(data.decode("utf-8", errors="replace"))
    s.close()
except Exception as e:
    print(f"Error: {e}")
