"""
File: controller.py
Description: PC-side controller script for RC Car.
Features: 
  - Xbox Controller Input (Corrected Axis Mapping)
  - Kick-back logic for steering friction compensation
  - Smoothing logic for throttle control
  - UDP Communication to Odroid
"""

import socket
import pygame
import time

# --- Network Configuration ---
TARGET_IP = "192.168.1.XXX"  # IP of Odroid
TARGET_PORT = 4210

# --- Hardware Mapping (Windows 11 Xbox Controller) ---
AXIS_RT = 5  # Gas
AXIS_LT = 4  # Brake/Reverse

# --- Tuning Parameters ---
MAX_FWD_SCALE = 500   
MAX_REV_SCALE = 300   # Increased scale for better reverse power
THROTTLE_DEADZONE = 20 
STEER_TRIM = 0        
STEER_DEADZONE = 20   

# --- Kick-back System Configuration ---
KICK_ENABLE = True    
KICK_FORCE = 60       # Force to apply (PWM units)
KICK_DURATION = 8     # Duration (cycles)

# --- Smoothing Configuration ---
RAMP_STEP = 150       # Lower = Smoother, Higher = More responsive

# --- State Variables ---
current_limit = 0.6   # Start speed limit at 60%
kick_counter = 0      
last_steer_side = 0   # 1=Right, -1=Left
current_pwm_throttle = 1500 

# --- Setup ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pygame.init()
pygame.joystick.init()
joy = pygame.joystick.Joystick(0)
joy.init()

def smooth_approach(current, target, step):
    """Gradually moves current value towards target value."""
    if abs(current - target) < step: return target
    if current < target: return current + step
    else: return current - step

def get_trigger(axis_index):
    """Reads trigger value and handles initial 0.0 bug."""
    val = joy.get_axis(axis_index)
    if abs(val) < 0.01: return 0.0 
    mapped_val = (val + 1) / 2
    if mapped_val < 0.05: return 0.0
    return mapped_val

try:
    while True:
        pygame.event.pump()
        
        # --- 1. Speed Limiter (X/Y Buttons) ---
        if joy.get_button(2): current_limit = max(0.1, current_limit - 0.1) # X
        if joy.get_button(3): current_limit = min(1.0, current_limit + 0.1) # Y

        # --- 2. Input Processing ---
        gas = get_trigger(AXIS_RT)   
        brake = get_trigger(AXIS_LT) 
        steer = joy.get_axis(0)

        # --- 3. Throttle Calculation with Smoothing ---
        val_fwd = gas * MAX_FWD_SCALE * current_limit
        val_rev = brake * MAX_REV_SCALE * current_limit
        target_throttle = 1500 + val_fwd - val_rev

        if abs(target_throttle - 1500) < THROTTLE_DEADZONE:
            target_throttle = 1500

        current_pwm_throttle = smooth_approach(current_pwm_throttle, target_throttle, RAMP_STEP)
        pwm_throttle = int(current_pwm_throttle)

        # --- 4. Steering with Kick-back Logic ---
        # Inverted logic: Right(1) -> 1000 | Left(-1) -> 2000
        raw_steer = (steer * -500) + 1500 + STEER_TRIM
        
        # Track last direction
        if raw_steer < 1450: last_steer_side = 1    # Right side
        elif raw_steer > 1550: last_steer_side = -1 # Left side

        if abs(raw_steer - 1500) < STEER_DEADZONE:
            # Apply kick-back if returning from Right side
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

        # --- 5. Data Transmission ---
        # Clamp values for safety
        pwm_throttle = max(1000, min(2000, pwm_throttle))
        pwm_steering = max(1000, min(2000, pwm_steering))

        msg = f"<{pwm_throttle},{pwm_steering}>"
        sock.sendto(msg.encode(), (TARGET_IP, TARGET_PORT))
        
        time.sleep(0.01) # 100Hz Loop

except KeyboardInterrupt:
    # Send stop command on exit
    for i in range(5):
        sock.sendto(b"<1500,1500>", (TARGET_IP, TARGET_PORT))
        time.sleep(0.05)
    sock.close()