/**
 * File: esp32_firmware.ino
 * Description: Low-level control for ESC and Servo.
 * Features: Hybrid Coast/Brake logic, Watchdog Failsafe, Anti-glitch.
 */

#include <ESP32Servo.h>

// --- Pin Definitions ---
const int PIN_THROTTLE = 13;
const int PIN_STEERING = 12;

// --- Config ---
const int COAST_MIN = 1490;
const int COAST_MAX = 1510;
const unsigned long FAILSAFE_TIMEOUT = 500; // Stop if no data for 0.5s

Servo servoThrottle;
Servo servoSteering;

unsigned long lastPacketTime = 0;
String inputString = "";
boolean stringComplete = false;
int lastDirection = 1; // 1=Forward, -1=Reverse

void setup() {
    Serial.begin(115200);
    
    servoThrottle.setPeriodHertz(50);
    servoSteering.setPeriodHertz(50);
    
    // Arming Sequence: Send 1500 to initialize ESC
    servoThrottle.attach(PIN_THROTTLE, 1000, 2000);
    servoSteering.attach(PIN_STEERING, 1000, 2000);
    servoThrottle.writeMicroseconds(1500);
    servoSteering.writeMicroseconds(1500);
    
    delay(2000); // Wait for ESC initialization beep

    inputString.reserve(200);
    lastPacketTime = millis();
}

// --- Hybrid Throttle Logic ---
void handleThrottle(int pwm) {
    // Active State (Moving)
    if (pwm < COAST_MIN || pwm > COAST_MAX) {
        // Track Direction
        if (pwm > 1500) lastDirection = 1;  
        else lastDirection = -1;            

        if (!servoThrottle.attached()) {
            servoThrottle.attach(PIN_THROTTLE, 1000, 2000);
        }
        servoThrottle.writeMicroseconds(pwm);
    }
    // Idle State (Released Joystick)
    else {
        // Case 1: Returning from Forward -> Enable Coasting
        if (lastDirection == 1) {
            if (servoThrottle.attached()) {
                servoThrottle.detach();
                // Anti-Glitch: Pull pin LOW to prevent floating signal noise
                pinMode(PIN_THROTTLE, OUTPUT);
                digitalWrite(PIN_THROTTLE, LOW); 
            }
        }
        // Case 2: Returning from Reverse -> Active Stop (Prevent Revving)
        else {
             if (!servoThrottle.attached()) {
                servoThrottle.attach(PIN_THROTTLE, 1000, 2000);
             }
             servoThrottle.writeMicroseconds(1500);
        }
    }
}

void emergencyStop() {
    // Re-attach and force neutral
    if (!servoThrottle.attached()) servoThrottle.attach(PIN_THROTTLE, 1000, 2000);
    servoThrottle.writeMicroseconds(1500);
    servoSteering.writeMicroseconds(1500);
}

void processPacket(String data) {
    if (!data.startsWith("<") || !data.endsWith(">")) return;
    
    data = data.substring(1, data.length() - 1); 
    int commaIndex = data.indexOf(',');
    if (commaIndex == -1) return;

    int pwmThrottle = data.substring(0, commaIndex).toInt();
    int pwmSteer    = data.substring(commaIndex + 1).toInt();

    // Clamp values
    pwmThrottle = constrain(pwmThrottle, 1000, 2000);
    pwmSteer    = constrain(pwmSteer, 1000, 2000);

    lastPacketTime = millis(); // Reset Failsafe Timer
    
    handleThrottle(pwmThrottle);
    servoSteering.writeMicroseconds(pwmSteer);
}

void loop() {
    // Non-blocking Serial Read
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        inputString += inChar;
        if (inChar == '>') stringComplete = true;
    }

    if (stringComplete) {
        inputString.trim();
        processPacket(inputString);
        inputString = "";
        stringComplete = false;
    }

    // Failsafe Check
    if (millis() - lastPacketTime > FAILSAFE_TIMEOUT) {
        emergencyStop();
    }
}