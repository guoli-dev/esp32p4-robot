"""Reset ESP32-P4 via RTS and capture boot log."""
import serial
import time

ser = serial.Serial("COM7", 115200, timeout=0.1)

# Toggle RTS to reset (ESP32-P4 EN pin connected to RTS via CH343)
ser.rts = True
time.sleep(0.05)
ser.rts = False
time.sleep(0.2)
ser.rts = True

# Read boot log
start = time.time()
while time.time() - start < 8:
    data = ser.read(4096)
    if data:
        print(data.decode("utf-8", errors="replace"), end="", flush=True)
    time.sleep(0.02)

ser.close()
print("\n=== DONE ===")
