/**
 * File: esp32_firmware_v2.ino
 * Description: RC Control + Improved Hall Sensor (Interval Method + Zero Timeout)
 */

#include <ESP32Servo.h>

/* ================== PIN DEFINITIONS ================== */
const int PIN_THROTTLE = 13;
const int PIN_STEERING = 12;
const int PIN_HALL     = 25;

/* ================== CONFIG ================== */
const int COAST_MIN = 1490;
const int COAST_MAX = 1510;
const unsigned long FAILSAFE_TIMEOUT = 500; // ms

// ตั้งค่าความถี่ในการส่งข้อมูลกลับ Odroid (ms)
// 50ms = 20Hz (กำลังดี ไม่รก Serial)
const unsigned long SEND_INTERVAL = 50; 

/* ================== WHEEL CONFIG ================== */
const int   pulsesPerRev = 2;        // แม่เหล็ก 2 ตัว
const float wheelDiameter = 0.0715;  // meter
const float wheelCircumference = 3.1415926 * wheelDiameter;

/* ================== FILTER CONFIG ================== */
#define FILTER_SIZE 5
float speedBuffer[FILTER_SIZE];
int   filterIndex = 0;

/* ================== OBJECTS ================== */
Servo servoThrottle;
Servo servoSteering;

/* ================== RC STATE ================== */
unsigned long lastPacketTime = 0;
String inputString = "";
bool stringComplete = false;
int lastDirection = 1;

/* ================== SENSOR STATE ================== */
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool newPulse = false;

float currentSpeedMps = 0.0;
unsigned long lastSendTime = 0;

/* ================== ISR (INTERRUPT) ================== */
void IRAM_ATTR hallISR() {
    unsigned long now = micros();
    // คำนวณเวลาที่ผ่านไปจาก Pulse รอบที่แล้ว
    pulseInterval = now - lastPulseTime;
    lastPulseTime = now;
    newPulse = true;
}

/* ================== SETUP ================== */
void setup() {
    Serial.begin(921600); // High Baudrate

    /* ---- Servo ---- */
    servoThrottle.setPeriodHertz(50);
    servoSteering.setPeriodHertz(50);
    servoThrottle.attach(PIN_THROTTLE, 1000, 2000);
    servoSteering.attach(PIN_STEERING, 1000, 2000);
    servoThrottle.writeMicroseconds(1500);
    servoSteering.writeMicroseconds(1500);
    
    delay(2000); // ESC Arming delay

    /* ---- Hall Sensor ---- */
    pinMode(PIN_HALL, INPUT_PULLUP);
    // ใช้ RISING ตามโค้ดใหม่ที่คุณให้มา
    attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, RISING);

    // Init Filter Buffer
    for (int i = 0; i < FILTER_SIZE; i++) {
        speedBuffer[i] = 0.0;
    }

    inputString.reserve(200);
    lastPacketTime = millis();
    lastPulseTime = micros(); 

    Serial.println("ESP32 Firmware V2 Ready");
}

/* ================== CONTROL LOGIC (เดิม) ================== */
void handleThrottle(int pwm) {
    if (pwm < COAST_MIN || pwm > COAST_MAX) {
        if (pwm > 1500) lastDirection = 1;
        else            lastDirection = -1;

        if (!servoThrottle.attached()) servoThrottle.attach(PIN_THROTTLE, 1000, 2000);
        servoThrottle.writeMicroseconds(pwm);
    } else {
        if (lastDirection == 1) {
            if (servoThrottle.attached()) {
                servoThrottle.detach();
                pinMode(PIN_THROTTLE, OUTPUT);
                digitalWrite(PIN_THROTTLE, LOW);
            }
        } else {
            if (!servoThrottle.attached()) servoThrottle.attach(PIN_THROTTLE, 1000, 2000);
            servoThrottle.writeMicroseconds(1500);
        }
    }
}

void emergencyStop() {
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

    lastPacketTime = millis();
    handleThrottle(constrain(pwmThrottle, 1000, 2000));
    servoSteering.writeMicroseconds(constrain(pwmSteer, 1000, 2000));
}

/* ================== SPEED CALCULATION ================== */
void updateSpeedSensor() {
    
    // 1. กรณีมี Pulse เข้ามาใหม่ (ล้อหมุน)
    if (newPulse) {
        noInterrupts();
        unsigned long interval = pulseInterval;
        newPulse = false;
        interrupts();

        if (interval > 0) {
            // คำนวณ RPM
            float rpm = (60.0 * 1000000.0) / (interval * pulsesPerRev);
            
            // คำนวณ m/s
            float rawSpeed = (wheelCircumference * rpm) / 60.0;

            // --- Moving Average Filter ---
            speedBuffer[filterIndex] = rawSpeed;
            filterIndex = (filterIndex + 1) % FILTER_SIZE;

            float sum = 0.0;
            for (int i = 0; i < FILTER_SIZE; i++) {
                sum += speedBuffer[i];
            }
            currentSpeedMps = sum / FILTER_SIZE;
        }
    }

    // 2. กรณี Timeout (ล้อหยุดหมุน)
    // ถ้าเวลาผ่านไปเกิน 0.5 วินาที (500,000 micros) โดยไม่มี pulse ใหม่
    if (micros() - lastPulseTime > 300000) { 
        currentSpeedMps = 0.0;
        
        // Reset Buffer เพื่อให้ตอนออกตัวใหม่ค่าค่อยๆ ไต่ขึ้น ไม่กระโดด
        // (หรือถ้าอยากให้ออกตัวไว ไม่ต้อง reset ก็ได้ แต่ reset จะนุ่มนวลกว่า)
        /*
        for (int i = 0; i < FILTER_SIZE; i++) {
            speedBuffer[i] = 0.0;
        }
        */
    }
}

/* ================== SEND TELEMETRY ================== */
void sendTelemetry() {
    unsigned long now = millis();
    if (now - lastSendTime >= SEND_INTERVAL) {
        // ส่ง Format <SPD:xx.xx> ตามเดิม
        Serial.print("<SPD:");
        Serial.print(currentSpeedMps, 3); // ทศนิยม 3 ตำแหน่งเพื่อความละเอียด
        Serial.println(">");
        
        lastSendTime = now;
    }
}

/* ================== MAIN LOOP ================== */
void loop() {
    // 1. รับคำสั่ง Serial
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

    // 2. อัปเดตเซนเซอร์
    updateSpeedSensor();

    // 3. ส่งข้อมูลกลับ (ทุกๆ 50ms)
    sendTelemetry();

    // 4. Failsafe
    if (millis() - lastPacketTime > FAILSAFE_TIMEOUT) {
        emergencyStop();
    }
}