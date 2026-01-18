import socket
import pygame
import time
import threading

# ==========================================
# CONFIGURATION SETTINGS
# ==========================================

# Network Configuration
TARGET_IP = "odroid"    # IP หรือ Hostname ของ Odroid ปลายทาง
TX_PORT   = 4210        # Port สำหรับส่งข้อมูลไปหา Odroid
RX_PORT   = 4211        # Port สำหรับรอรับข้อมูลจาก Odroid

# Joystick Mapping (Linux/KDE Standard)
AXIS_STEER  = 0         # แกนหมุนเลี้ยว (Left Stick X-Axis)
AXIS_ACCEL  = 5         # คันเร่ง (RT - Right Trigger)
AXIS_BRAKE  = 2         # เบรก/ถอยหลัง (LT - Left Trigger)
BTN_GEAR_UP = 3         # ปุ่มเพิ่มเกียร์ (Y Button)
BTN_GEAR_DN = 0         # ปุ่มลดเกียร์ (A Button)

# Control Logic Config
GYRO_DEADZONE = 5.0     # ค่า Deadzone ของ Gyro เพื่อตัด Noise

# Global Variables (Telemetry)
current_speed = 0.0
current_gyro  = 0.0
turn_status   = "--- STRAIGHT ---"

# ==========================================
# NETWORK SETUP
# ==========================================
sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rx.bind(("0.0.0.0", RX_PORT)) 
sock_rx.settimeout(0.005)  # Timeout สั้นมากเพื่อป้องกัน RX บล็อกการทำงานหลัก

# ==========================================
# PYGAME & JOYSTICK SETUP
# ==========================================
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Error: No Joystick Found")
    exit()

joy = pygame.joystick.Joystick(0)
joy.init()
print(f" Connected: {joy.get_name()}")

# ==========================================
# THREAD: RECEIVE DATA (RX)
# ==========================================
def rx_thread():
    """
    Thread สำหรับรอรับข้อมูล Telemetry จาก Odroid
    Format ที่คาดหวัง: <SPD:speed,gyro>
    """
    global current_speed, current_gyro, turn_status
    while True:
        try:
            data, _ = sock_rx.recvfrom(128)
            msg = data.decode('utf-8', errors='ignore').strip()
            
            # ตรวจสอบ Header และ Footer ของข้อมูล
            if msg.startswith("<SPD:") and msg.endswith(">"):
                # ตัดส่วน <SPD: และ > ออก แล้วแยกค่าด้วย comma
                val_str = msg[5:-1]
                parts = val_str.split(',')
                
                if len(parts) >= 2:
                    current_speed = float(parts[0])
                    current_gyro  = float(parts[1])
                    
                    # Update สถานะการเลี้ยวตามค่า Gyro
                    if current_gyro > GYRO_DEADZONE:
                        turn_status = "TURNING LEFT  <<<<"
                    elif current_gyro < -GYRO_DEADZONE:
                        turn_status = "TURNING RIGHT >>>>"
                    else:
                        turn_status = "--- STRAIGHT ---"

        except socket.timeout:
            continue  # ไม่มีข้อมูลมา ให้วนลูปต่อเงียบๆ
        except Exception:
            pass      # ข้าม Error อื่นๆ เพื่อกันโปรแกรมหลุด

# Start RX Thread
threading.Thread(target=rx_thread, daemon=True).start()

# ==========================================
# HELPER FUNCTIONS
# ==========================================
def get_linux_trigger(axis_idx):
    """
    แปลงค่า Trigger ของ Linux Joystick
    Input: -1.0 (ปล่อย) ถึง 1.0 (กดสุด)
    Output: 0.0 (ปล่อย) ถึง 1.0 (กดสุด)
    """
    raw = joy.get_axis(axis_idx)
    val = (raw + 1.0) / 2.0
    
    # Deadzone เล็กน้อยเพื่อกันค่าแกว่งตอนปล่อยปุ่ม
    if val < 0.05: return 0.0
    return val

# ==========================================
# MAIN LOOP
# ==========================================
try:
    # Local State Variables
    current_gear = 1
    max_gear     = 5
    prev_gear_up = 0
    prev_gear_dn = 0
    
    print(f" Ready! Linking to Odroid @ {TARGET_IP}")

    while True:
        pygame.event.pump() # อัปเดต Event ของ Pygame
        
        # -----------------------------
        # 1. Gear Logic (Rising Edge)
        # -----------------------------
        g_up = joy.get_button(BTN_GEAR_UP)
        g_dn = joy.get_button(BTN_GEAR_DN)
        
        # เพิ่มเกียร์เมื่อกดครั้งแรก (และไม่เกิน max)
        if g_up and not prev_gear_up and current_gear < max_gear: 
            current_gear += 1
        # ลดเกียร์เมื่อกดครั้งแรก (และไม่ต่ำกว่า 1)
        if g_dn and not prev_gear_dn and current_gear > 1: 
            current_gear -= 1
            
        prev_gear_up, prev_gear_dn = g_up, g_dn

        # -----------------------------
        # 2. Throttle Logic
        # -----------------------------
        rt = get_linux_trigger(AXIS_ACCEL)
        lt = get_linux_trigger(AXIS_BRAKE)
        
        throttle_pwm = 1500 # ค่ากลาง (Neutral)
        speed_factor = (current_gear / max_gear) # อัตราทดตามเกียร์
        
        if rt > 0.0:
            # เดินหน้า: 1500 + (แรงกด * 500 * อัตราทด)
            throttle_pwm = 1500 + int(rt * 500 * speed_factor)
        elif lt > 0.0:
            # ถอยหลัง/เบรก: 1500 - (แรงกด * 500 * อัตราทด)
            throttle_pwm = 1500 - int(lt * 500 * speed_factor)
            
        # -----------------------------
        # 3. Steering Logic
        # -----------------------------
        steer_axis = joy.get_axis(AXIS_STEER)
        # แปลง Axis (-1..1) เป็น PWM offset (1500 +/- 500)
        steer_pwm = 1500 - int(steer_axis * 500) 

        # -----------------------------
        # 4. Network Send (UDP)
        # -----------------------------
        cmd = f"<{throttle_pwm},{steer_pwm}>"
        sock_tx.sendto(cmd.encode(), (TARGET_IP, TX_PORT))

        # -----------------------------
        # 5. Dashboard Display
        # -----------------------------
        print(f"G:{current_gear} | PWM:{throttle_pwm} | SPD:{current_speed:5.2f} m/s | {turn_status}      ", end='\r')
        
        # 100Hz Update Rate (Control Loop Frequency)
        time.sleep(0.01) 

except KeyboardInterrupt:
    print("\n Stopping...")
    # ส่งค่า Neutral ก่อนปิดโปรแกรมเพื่อความปลอดภัย
    sock_tx.sendto(b"<1500,1500>", (TARGET_IP, TX_PORT))
    sock_tx.close()
    sock_rx.close()
    print("Disconnected.")