"""
File: bridge.py
Description: Runs on Odroid C4. Bridges UDP packets to Serial (ESP32).
"""

import socket
import serial
import time
import sys

# --- Configuration ---
UDP_IP = "0.0.0.0"
UDP_PORT = 4210
SERIAL_PORT = '/dev/ttyUSB0' # Adjust based on connected port
BAUD_RATE = 115200

# --- Setup ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except Exception as e:
    print(f"Serial Error: {e}")
    sys.exit()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("Bridge Started. Press Ctrl+C to stop.")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        message = data.decode('utf-8').strip()
        
        # Validate format <Throttle,Steering>
        if message.startswith('<') and message.endswith('>'):
            ser.write(message.encode())

except KeyboardInterrupt:
    print("\nStopping Bridge...")

finally:
    # --- Failsafe: Send Stop Command to ESP32 before exiting ---
    print("Sending Emergency Stop...")
    ser.write(b"<1500,1500>") 
    time.sleep(0.5)
    ser.close()
    sock.close()