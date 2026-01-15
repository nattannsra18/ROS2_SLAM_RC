"""
File: pc_controller_v3.py
Description: RC Controller with Tuned Gears & Debug Mode
Features:
  - Gear System (1-5)
  - Tuned Reverse Limit (Max 1300)
  - Debug RX for Speed Telemetry
"""

import socket
import pygame
import time
import threading

# --- Network Configuration ---
TARGET_IP = "odroid" 
TX_PORT = 4210
RX_PORT = 4211

# --- Hardware Mapping (Xbox Controller) ---
AXIS_RT = 5  # Gas
AXIS_LT = 4  # Brake/Reverse
BUTTON_X = 2 # Gear Down
BUTTON_Y = 3 # Gear Up

# --- Gearbox Configuration ---
GEAR_MAX = 5
PWM_NEUTRAL = 1500

# จุดเริ่มทำงาน (Start PWM)
PWM_FWD_START = 1580 
PWM_REV_START = 1430 

# --- ปรับจูนความเร็วสูงสุดที่นี่ ---
PWM_FWD_LIMIT = 2000
PWM_REV_LIMIT = 1300  

# --- Smoothing & Kick-back ---
RAMP_STEP = 150      
STEER_TRIM = 0        
STEER_DEADZONE = 20   
KICK_ENABLE = True    
KICK_FORCE = 60       
KICK_DURATION = 8     

# --- State Variables ---
current_gear = 1      
current_pwm_throttle = 1500 
current_speed_display = 0.0
last_steer_side = 0   
kick_counter = 0      
prev_btn_x = 0
prev_btn_y = 0

# --- Setup Network ---
sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rx.bind(("0.0.0.0", RX_PORT)) # รอรับทุก IP
sock_rx.settimeout(0.01)

# --- Setup Pygame ---
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("❌ No joystick detected!")
    exit()
joy = pygame.joystick.Joystick(0)
joy.init()
print(f"🎮 Connected: {joy.get_name()}")

# ================= RX THREAD (DEBUG MODE) =================
def udp_receive_thread():
    global current_speed_display
    while True:
        try:
            data, addr = sock_rx.recvfrom(128)
            msg = data.decode('utf-8', errors='ignore').strip()
            
            if msg.startswith("<SPD:") and msg.endswith(">"):
                try:
                    speed_str = msg[5:-1]
                    current_speed_display = float(speed_str)
                except ValueError:
                    pass
        except socket.timeout:
            continue
        except Exception as e:
            print(f"RX Error: {e}")

# Start RX Thread
threading.Thread(target=udp_receive_thread, daemon=True).start()

# ================= HELPER FUNCTIONS =================
def smooth_approach(current, target, step):
    if abs(current - target) < step: return target
    if current < target: return current + step
    else: return current - step

def map_gear_speed(trigger_val, is_forward):
    """คำนวณ PWM ตามเกียร์"""
    gear_ratio = current_gear / GEAR_MAX 
    
    if is_forward:
        available_range = PWM_FWD_LIMIT - PWM_FWD_START
        current_range_limit = available_range * gear_ratio
        return PWM_FWD_START + (trigger_val * current_range_limit)
    else:
        # Logic ถอยหลัง: 1430 ลงไปหา 1300
        # Range เต็ม = 1430 - 1300 = 130
        available_range = PWM_REV_START - PWM_REV_LIMIT 
        current_range_limit = available_range * gear_ratio
        
        # ยิ่งกดเยอะ ค่า PWM ยิ่งลดลง (เข้าใกล้ 1300)
        return PWM_REV_START - (trigger_val * current_range_limit)

def get_trigger(axis):
    val = joy.get_axis(axis)
    if abs(val) < 0.01: return 0.0
    val = (val + 1) / 2
    return 0.0 if val < 0.05 else val

# ================= MAIN LOOP =================
try:
    last_print_time = 0
    print("🚀 Controller Ready. Press Ctrl+C to stop.")

    while True:
        pygame.event.pump()
        
        # --- Gear Shift ---
        btn_x = joy.get_button(BUTTON_X)
        btn_y = joy.get_button(BUTTON_Y)
        if btn_x and not prev_btn_x and current_gear > 1: current_gear -= 1
        if btn_y and not prev_btn_y and current_gear < GEAR_MAX: current_gear += 1
        prev_btn_x, prev_btn_y = btn_x, btn_y

        # --- Throttle & Brake ---
        gas = get_trigger(AXIS_RT)   
        brake = get_trigger(AXIS_LT) 
        
        target = 1500
        if gas > 0.01: target = map_gear_speed(gas, True)
        elif brake > 0.01: target = map_gear_speed(brake, False)

        current_pwm_throttle = smooth_approach(current_pwm_throttle, target, RAMP_STEP)
        pwm_throttle = int(current_pwm_throttle)

        # --- Steering ---
        steer = joy.get_axis(0)
        raw_steer = (steer * -500) + 1500 + STEER_TRIM
        
        if raw_steer < 1450: last_steer_side = 1
        elif raw_steer > 1550: last_steer_side = -1

        if abs(raw_steer - 1500) < STEER_DEADZONE:
            if KICK_ENABLE and last_steer_side == 1:
                kick_counter = KICK_DURATION
                last_steer_side = 0 
            
            if kick_counter > 0:
                pwm_steering = 1500 + KICK_FORCE
                kick_counter -= 1
            else:
                pwm_steering = 1500
        else:
            kick_counter = 0
            pwm_steering = int(raw_steer)

        # --- Send UDP ---
        msg = f"<{pwm_throttle},{pwm_steering}>"
        sock_tx.sendto(msg.encode(), (TARGET_IP, TX_PORT))
        
        # --- Display ---
        now = time.time()
        if now - last_print_time > 0.1:
            # คำนวณความเร็ว Max ของเกียร์ปัจจุบันเพื่อโชว์
            max_rev_gear = PWM_REV_START - ((PWM_REV_START - PWM_REV_LIMIT) * (current_gear/GEAR_MAX))
            
            print(f"GEAR: {current_gear} | "
                  f"PWM: {pwm_throttle:4d} (RevLim: {int(max_rev_gear)}) | "
                  f"SPD: {current_speed_display:5.2f} m/s", end='\r')
            last_print_time = now

        time.sleep(0.001)

except KeyboardInterrupt:
    print("\nStopping...")
    for _ in range(5):
        sock_tx.sendto(b"<1500,1500>", (TARGET_IP, TX_PORT))
        time.sleep(0.05)
    sock_tx.close()
    sock_rx.close()