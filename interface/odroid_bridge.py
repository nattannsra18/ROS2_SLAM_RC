"""
File: odroid_bridge.py
"""

import socket
import serial
import time
import glob
import sys
import select

# ================= CONFIGURATION =================
TARGET_PC_IP = "192.168.0.44" 

UDP_PORT_RX = 4210  # Port ฟังคำสั่ง (จาก PC)
UDP_PORT_TX = 4211  # Port ส่งกลับ (Speed ไป PC)

# Serial Config (ต้องตรงกับ ESP32)
BAUD_RATE = 921600      
SERIAL_TIMEOUT = 0  # Non-blocking

# ================= SETUP =================
# 1. Setup UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT_RX))
sock.setblocking(False) # สำคัญมาก: ห้ามรอข้อมูล

# 2. Setup Serial Helper
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
            # ปรับแต่ง: write_timeout=0 เพื่อไม่ให้รอตอนส่ง
            ser = serial.Serial(ports[0], BAUD_RATE, timeout=0, write_timeout=0)
            print(f"\n[SYSTEM] ✅ ESP32 Connected: {ports[0]} @ {BAUD_RATE}")
            return True
        except Exception as e:
            print(f"[ERR] Serial: {e}")
    return False

print(f"[SYSTEM] 🚀 Ultra-Low Latency Bridge Started.")
print(f"[SYSTEM] 📡 Target PC: {TARGET_PC_IP}:{UDP_PORT_TX}")

# ================= MAIN LOOP =================
try:
    buffer_serial = ""
    last_print = 0
    
    while True:
        # Check Serial Connection
        if not (ser and ser.is_open):
            connect_serial()
            time.sleep(0.1) # พักนิดหน่อยถ้ายิงหาไม่เจอ
            continue

        # ----------------------------------------
        # PART 1: UDP (PC) -> Serial (ESP32)
        # ----------------------------------------
        # ใช้ select ตรวจสอบว่ามีข้อมูลเข้ามาไหม (ลด CPU Load ได้ดีกว่าวนลูปเปล่าๆ)
        # แต่ตั้ง timeout เป็น 0 เพื่อให้เป็น Non-blocking
        ready_to_read, _, _ = select.select([sock], [], [], 0)
        
        if ready_to_read:
            try:
                data, _ = sock.recvfrom(64) # รับแค่ 64 bytes ก็พอ (คำสั่งสั้น)
                msg = data.strip()
                if msg:
                    # ยิงลง Serial ทันที (Raw bytes)
                    ser.write(msg + b'\n')
            except OSError:
                pass

        # ----------------------------------------
        # PART 2: Serial (ESP32) -> UDP (PC)
        # ----------------------------------------
        try:
            # ตรวจสอบว่ามีข้อมูลใน Serial Buffer ไหม
            bytes_waiting = ser.in_waiting
            if bytes_waiting > 0:
                # อ่านรวดเดียวหมด Buffer (เพื่อเคลียร์ Lag)
                raw_data = ser.read(bytes_waiting).decode('utf-8', errors='ignore')
                buffer_serial += raw_data
                
                # ถ้ามีขึ้นบรรทัดใหม่ แปลว่าจบ Packet
                while '\n' in buffer_serial:
                    line, buffer_serial = buffer_serial.split('\n', 1)
                    line = line.strip()
                    
                    # กรองเอาเฉพาะข้อมูล Speed (<SPD:xx.xx>)
                    if line.startswith('<SPD:') and line.endswith('>'):
                        # ส่ง UDP กลับไป PC ทันที
                        sock.sendto(line.encode(), (TARGET_PC_IP, UDP_PORT_TX))
                        
                        # Debug: ปริ้นท์แค่บางครั้งเพื่อไม่ให้หน่วง (ทุก 0.2 วิ)
                        if time.time() - last_print > 0.2:
                            print(f"[FWD] {line}      ", end='\r')
                            last_print = time.time()
                            
        except OSError:
            print("\n[ERR] Serial Lost!")
            ser.close()
            ser = None
        except Exception:
            pass

        # Loop นี้ไม่มี sleep() เพื่อความไวสูงสุด
        # Python จะกิน CPU 1 Core เกือบ 100% ซึ่งปกติสำหรับงาน Real-time

except KeyboardInterrupt:
    print("\n[SYSTEM] Stopping...")
    if ser: ser.close()
    sock.close()