import socket
import serial
import time
import glob
import sys
import select

# ================= CONFIGURATION =================
TARGET_PC_IP = "192.168.1.96"  # IP ของคอมพิวเตอร์ที่รับค่า

UDP_PORT_RX = 4210  # Port ฟังคำสั่ง (PC -> ESP32)
UDP_PORT_TX = 4211  # Port ส่งกลับ (ESP32 -> PC)
BAUD_RATE = 921600      

# ================= SETUP =================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT_RX))
sock.setblocking(False) 

ser = None
last_serial_check = 0

def connect_serial():
    global ser, last_serial_check
    now = time.time()
    if ser and ser.is_open: return True
    if now - last_serial_check < 1.0: return False
    last_serial_check = now
    
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    ports.sort()
    
    if ports:
        try:
            ser = serial.Serial(ports[0], BAUD_RATE, timeout=0, write_timeout=0)
            print(f"[SYSTEM] Connected: {ports[0]}")
            return True
        except:
            return False
    return False

print(f"[SYSTEM] Bridge Started.")

# ================= MAIN LOOP =================
try:
    buffer_serial = ""
    
    while True:
        # 1. Maintain Serial Connection
        if not (ser and ser.is_open):
            connect_serial()
            time.sleep(0.1)
            continue

        # 2. UDP -> Serial (Flush Buffer & Send Last)
        last_data = None
        while True:
            ready, _, _ = select.select([sock], [], [], 0)
            if ready:
                try:
                    data, _ = sock.recvfrom(64)
                    last_data = data
                except:
                    break
            else:
                break
        
        if last_data:
            try:
                ser.write(last_data.strip() + b'\n')
            except:
                pass

        # 3. Serial -> UDP (Forward Speed Data)
        try:
            bytes_waiting = ser.in_waiting
            if bytes_waiting > 0:
                raw = ser.read(bytes_waiting).decode('utf-8', errors='ignore')
                buffer_serial += raw
                
                while '\n' in buffer_serial:
                    line, buffer_serial = buffer_serial.split('\n', 1)
                    line = line.strip()
                    
                    if line.startswith('<SPD:') and line.endswith('>'):
                        sock.sendto(line.encode(), (TARGET_PC_IP, UDP_PORT_TX))
                        # print(line) # Uncomment ถ้าอยากดูค่า
                        
        except OSError:
            print("[ERR] Serial Lost")
            ser.close()
            ser = None
        except:
            pass

except KeyboardInterrupt:
    if ser: ser.close()
    sock.close()