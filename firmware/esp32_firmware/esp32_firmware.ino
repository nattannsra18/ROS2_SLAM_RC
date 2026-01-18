#include <Wire.h>
#include <ESP32Servo.h>

// ==========================================
// 1. HARDWARE CONFIGURATION
// ==========================================
#define SERIAL_BAUD      921600   // High speed serial for low latency
#define PIN_THROTTLE     13       // ESC Signal Pin
#define PIN_STEERING     12       // Servo Signal Pin
#define PIN_HALL         25       // Hall Effect Sensor (Speed)

// ==========================================
// 2. DRIVING DYNAMICS TUNING
// ==========================================
// Throttle Smoothing
#define RAMP_INTERVAL_MS 2        // Update throttle every X ms
#define THROTTLE_STEP    50       // Max change per step (Higher = More aggressive)

// Deadzone & Neutral
#define DEADZONE_MIN     1480
#define DEADZONE_MAX     1520
#define USE_SIGNAL_CUT   true     // true = Enable Coasting (Detach Servo)

// ==========================================
// 3. SPEEDOMETER CALIBRATION
// ==========================================
// Physics
#define TIRE_DIAMETER_M  0.070    // Wheel Diameter: 7 cm
#define PULSES_PER_REV   2.0      // Magnets per wheel revolution

// Signal Filtering (Crucial for clean speed)
#define DEBOUNCE_MS      15       // Ignore pulses closer than 15ms (Noise filter)
#define WINDOW_SIZE      5        // Number of samples for Moving Average
#define UPDATE_RATE_MS   50       // Speed calc frequency (20Hz)

// ==========================================
// GLOBAL VARIABLES
// ==========================================
Servo servoThrottle;
Servo servoSteering;

// --- Speed Calculation ---
volatile unsigned long pulseCountISR = 0;    // Pulses from Interrupt
volatile unsigned long lastDebounceTime = 0; // Timestamp for Debounce
unsigned long pulseHistory[WINDOW_SIZE];     // Rolling window buffer
byte historyIdx = 0;
float currentSpeedMs = 0.0;

// --- IMU (Gyro) ---
#define MPU_ADDR 0x68
float gyroZ_offset = 0;

// --- Control State ---
int targetThrottle  = 1500;
int currentThrottle = 1500;  // Actual PWM sent to motor
int targetSteering  = 1500;
unsigned long lastCmdTime = 0;
unsigned long lastRampTime = 0;

// --- Drive Logic Flags ---
bool isCoasting      = false;
bool isForwardAction = true; // Track intended direction

// --- Serial Buffer ---
const byte numChars = 64;
char receivedChars[numChars];

// ==========================================
// INTERRUPT SERVICE ROUTINE (ISR)
// ==========================================
void IRAM_ATTR hallISR() {
  unsigned long now = millis();
  
  // Debounce Logic:
  // ถ้าระยะเวลาห่างจาก pulse ล่าสุด น้อยกว่า DEBOUNCE_MS -> ถือว่าเป็น Noise
  if (now - lastDebounceTime > DEBOUNCE_MS) {
    pulseCountISR++;
    lastDebounceTime = now;
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  // 1. Init Serial
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(1);

  // 2. Init Servos
  servoThrottle.setPeriodHertz(50);
  servoSteering.setPeriodHertz(50);
  servoThrottle.attach(PIN_THROTTLE, 1000, 2000);
  servoSteering.attach(PIN_STEERING, 1000, 2000);

  // 3. ESC Arming Sequence (Safety)
  // ส่งค่า Neutral แช่ไว้ 3 วินาที เพื่อให้ ESC พร้อมทำงาน
  servoThrottle.writeMicroseconds(1500);
  servoSteering.writeMicroseconds(1500);
  delay(3000);

  // 4. Init Hall Sensor
  pinMode(PIN_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, FALLING);

  // 5. Init IMU (MPU6050)
  Wire.begin();
  Wire.setClock(400000); // Fast Mode
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power Management 1
  Wire.write(0);    // Wake up
  Wire.endTransmission(true);

  // 6. Calibrate Gyro (Zeroing)
  delay(500);
  long sum = 0;
  for(int i=0; i<100; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); // Register 0x47 (GYRO_ZOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2);
    if(Wire.available() >= 2) {
      int16_t reading = (Wire.read() << 8) | Wire.read();
      sum += reading;
    }
    delay(2);
  }
  gyroZ_offset = sum / 100.0;

  // 7. Reset History
  for(int i=0; i<WINDOW_SIZE; i++) pulseHistory[i] = 0;
  lastCmdTime = millis();
}

// ==========================================
// HELPER: PARSE COMMAND
// ==========================================
void parseInternal() {
  // Expected format: <throttle,steering>
  char * strtokIndx; 
  
  strtokIndx = strtok(receivedChars, ",");
  if(strtokIndx == NULL) return;
  int thr = atoi(strtokIndx);
  
  strtokIndx = strtok(NULL, ",");
  if(strtokIndx == NULL) return;
  int str = atoi(strtokIndx);

  targetThrottle = constrain(thr, 1000, 2000);
  targetSteering = constrain(str, 1000, 2000);
  lastCmdTime = millis();
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  // -----------------------------
  // 1. SERIAL READ (Non-blocking)
  // -----------------------------
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) ndx = numChars - 1;
      } else {
        receivedChars[ndx] = '\0'; // Terminate string
        recvInProgress = false;
        ndx = 0;
        parseInternal();
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }

  // -----------------------------
  // 2. MOTOR CONTROL LOGIC
  // -----------------------------
  servoSteering.writeMicroseconds(targetSteering);

  // --- CASE A: NEUTRAL / DEADZONE ---
  if (targetThrottle > DEADZONE_MIN && targetThrottle < DEADZONE_MAX) {
      if (USE_SIGNAL_CUT && isForwardAction) {
          // Coasting Mode: ตัดสัญญาณเพื่อให้รถไหลฟรี (เลียนแบบคลัตช์)
          if (!isCoasting) {
             servoThrottle.detach();
             isCoasting = true;
             currentThrottle = 1500;
          }
      } else {
          // Normal Mode: สั่งเบรก/หยุดมอเตอร์
          if (isCoasting) {
             servoThrottle.attach(PIN_THROTTLE, 1000, 2000);
             isCoasting = false;
          }
          servoThrottle.writeMicroseconds(1500);
          currentThrottle = 1500;
      }
  } 
  // --- CASE B: ACTIVE DRIVING ---
  else {
      // ตรวจสอบทิศทางที่ตั้งใจขับ (เพื่อใช้ตัดสินใจตอนปล่อยคันเร่ง)
      if (targetThrottle > 1520) isForwardAction = true;
      else if (targetThrottle < 1480) isForwardAction = false;

      // Re-attach servo if returning from coasting
      if (isCoasting) {
          servoThrottle.attach(PIN_THROTTLE, 1000, 2000);
          isCoasting = false;
      }

      // Smooth Ramping (ป้องกันกระชาก)
      if (millis() - lastRampTime >= RAMP_INTERVAL_MS) {
        if (currentThrottle != targetThrottle) {
            if (currentThrottle < targetThrottle) {
              currentThrottle += THROTTLE_STEP;
              if (currentThrottle > targetThrottle) currentThrottle = targetThrottle;
            } else {
              currentThrottle -= THROTTLE_STEP;
              if (currentThrottle < targetThrottle) currentThrottle = targetThrottle;
            }
            servoThrottle.writeMicroseconds(currentThrottle);
        }
        lastRampTime = millis();
      }
  }

  // --- FAILSAFE (Safety Stop) ---
  // ถ้าขาดการติดต่อนานเกิน 0.5 วินาที ให้หยุดรถ
  if (millis() - lastCmdTime > 500) {
    targetThrottle = 1500;
    targetSteering = 1500;
    if (!isCoasting) servoThrottle.writeMicroseconds(1500);
  }

  // -----------------------------
  // 3. TELEMETRY & SPEED CALC
  // -----------------------------
  static unsigned long lastCalc = 0;
  
  if (millis() - lastCalc >= UPDATE_RATE_MS) {
    
    // A. ดึงค่าจาก ISR อย่างปลอดภัย (Critical Section)
    noInterrupts();
    unsigned long pulsesNow = pulseCountISR;
    pulseCountISR = 0;
    interrupts();

    // B. เก็บลง Rolling Window Buffer
    pulseHistory[historyIdx] = pulsesNow;
    historyIdx = (historyIdx + 1) % WINDOW_SIZE;

    // C. รวมผล Pulse ย้อนหลัง (Smoothing)
    unsigned long totalPulses = 0;
    for(int i=0; i<WINDOW_SIZE; i++) {
        totalPulses += pulseHistory[i];
    }

    // D. คำนวณความเร็ว (m/s)
    // Formula: (TotalPulses / TimeWindow) / PPR * Circumference
    float totalTimeSec = (UPDATE_RATE_MS * WINDOW_SIZE) / 1000.0; // Window time (0.25s)
    
    if (totalTimeSec > 0 && totalPulses > 0) {
        float rps = ((float)totalPulses / totalTimeSec) / PULSES_PER_REV;
        currentSpeedMs = rps * 3.14159 * TIRE_DIAMETER_M;
    } else {
        currentSpeedMs = 0.0;
    }

    // E. อ่านค่า Gyro (Z-Axis)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2);
    int16_t rawZ = (Wire.read() << 8) | Wire.read();
    float gyroZ = ((float)rawZ - gyroZ_offset) / 131.0;

    // F. ส่งข้อมูลกลับ PC <SPD:speed,gyro,thr,str>
    // Format: <SPD:1.25,0.05,1550,1400>
    char msg[64];
    sprintf(msg, "<SPD:%.2f,%.2f,%d,%d>", currentSpeedMs, gyroZ, currentThrottle, targetSteering);
    Serial.println(msg);
    
    lastCalc = millis();
  }
}