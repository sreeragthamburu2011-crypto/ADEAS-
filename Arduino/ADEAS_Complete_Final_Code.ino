/*
 * ═══════════════════════════════════════════════════════════════════════════
 * PROJECT: ADEAS - ACCIDENT DETECTION & EMERGENCY ALERT SYSTEM
 * VEHICLE: Bluetooth Controlled 4-Wheel RC Car with Arduino UNO R3
 * VERSION: Complete Implementation (Phase 1 + Phase 2)
 * 
 * DESCRIPTION:
 * This is the complete Arduino code for a Bluetooth-controlled RC car with
 * accident detection capability. The system uses:
 * - HC-05 Bluetooth module for wireless control via MIT App Inventor app
 * - L298N motor driver for 4 independent motors
 * - SW-18020P vibration sensor for immediate collision detection
 * - MPU6050 gyroscope/accelerometer for advanced accident detection
 * - Buzzer alarm system for emergency alerts
 * - LED status indicator
 * 
 * AUTHOR: sreeragthamburu2011-crypto
 * DATE: 2026
 * LICENSE: Open Source
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>

// ═══════════════════════════════════════════════════════════════════════════
// HARDWARE PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════

// Bluetooth Module Pins (SoftwareSerial)
#define BT_RX 10              // HC-05 TXD pin
#define BT_TX 11              // HC-05 RXD pin
SoftwareSerial BT(BT_RX, BT_TX);

// Motor Control Pins (L298N Motor Driver)
#define MOTOR_LEFT_FORWARD 4   // IN1 - Left motor forward control
#define MOTOR_LEFT_BACKWARD 5  // IN2 - Left motor backward control
#define MOTOR_RIGHT_FORWARD 6  // IN3 - Right motor forward control
#define MOTOR_RIGHT_BACKWARD 7 // IN4 - Right motor backward control

// Output Device Pins
#define BUZZER_PIN 3           // Piezo buzzer for alarm
#define STATUS_LED 8           // LED for status indication

// Sensor Pins
#define VIBRATION_SENSOR 2     // SW-18020P vibration/collision sensor
#define MPU_INTERRUPT 9        // MPU6050 interrupt pin (optional)

// ═══════════════════════════════════════════════════════════════════════════
// SYSTEM CONSTANTS & CONFIGURATION
// ════════════════════════════════��══════════════════════════════════════════

// Accident Detection Thresholds
const int VIBRATION_THRESHOLD = HIGH;      // Vibration sensor trigger level
const int ACCEL_THRESHOLD = 2500;          // Acceleration threshold (mg)
const int GYRO_THRESHOLD = 20000;          // Gyro threshold (deg/s * 131 LSB/deg/s)

// Safety Timeouts (milliseconds)
const unsigned long EMERGENCY_ALERT_DELAY = 20000;  // 20 second countdown before auto-alert
const unsigned long DEBOUNCE_DELAY = 100;           // Debounce time for sensors
const unsigned long SERIAL_TIMEOUT = 5000;          // Serial communication timeout

// Buzzer Alarm Patterns
const int ALARM_FREQUENCY_HIGH = 1000;   // High tone frequency (Hz)
const int ALARM_FREQUENCY_LOW = 500;     // Low tone frequency (Hz)
const int ALARM_DURATION = 200;          // Alarm beep duration (ms)

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL VARIABLES & STATE MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════

// MPU6050 Sensor Object
MPU6050 mpu;

// System State Variables
boolean isSystemLocked = false;        // Car is locked after accident
boolean accidentDetected = false;      // Accident has been detected
boolean emergencyAlertSent = false;    // Emergency alert already sent
unsigned long accidentTimestamp = 0;   // Time when accident occurred

// Motor Control State
volatile char lastCommand = 'S';       // Last received command
volatile char currentMovement = 'S';   // Current movement state

// Sensor Reading Variables
int16_t accelX = 0, accelY = 0, accelZ = 0;
int16_t gyroX = 0, gyroY = 0, gyroZ = 0;
int16_t tempRaw = 0;

// Communication Variables
String incomingCommand = "";
unsigned long lastSerialData = 0;

// LED Blink State
unsigned long lastLEDToggle = 0;
boolean ledState = false;
boolean blinkMode = false;             // true = emergency blinking, false = normal

// ═══════════════════════════════════════════════════════════════════════════
// SETUP FUNCTION - Runs once at startup
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  // Initialize Serial Communication for Debugging
  Serial.begin(9600);
  delay(500);
  
  // Print startup banner
  Serial.println("\n\n");
  Serial.println("╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║           ADEAS - RC CAR CONTROL SYSTEM                   ║");
  Serial.println("║    Accident Detection & Emergency Alert System            ║");
  Serial.println("║              Arduino UNO R3 Initialization                ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize Bluetooth Module
  Serial.println("[INIT] Starting Bluetooth Module (HC-05)...");
  BT.begin(9600);
  Serial.println("[INIT] ✓ Bluetooth Module initialized (9600 baud)");
  
  // Configure Motor Control Pins
  Serial.println("[INIT] Configuring motor control pins...");
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  Serial.println("[INIT] ✓ Motor pins configured");
  
  // Configure Output Device Pins
  Serial.println("[INIT] Configuring output devices...");
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  Serial.println("[INIT] ✓ Output devices configured");
  
  // Configure Sensor Pins
  Serial.println("[INIT] Configuring sensors...");
  pinMode(VIBRATION_SENSOR, INPUT);
  pinMode(MPU_INTERRUPT, INPUT);
  Serial.println("[INIT] ✓ Sensor pins configured");
  
  // Initialize I2C Communication for MPU6050
  Serial.println("[INIT] Initializing I2C communication...");
  Wire.begin();
  Serial.println("[INIT] ✓ I2C initialized");
  
  // Initialize MPU6050 Sensor
  Serial.println("[INIT] Initializing MPU6050 sensor...");
  mpu.initialize();
  
  // Test MPU6050 Connection
  if (!mpu.testConnection()) {
    Serial.println("[ERROR] ✗ MPU6050 connection FAILED!");
    Serial.println("[ERROR] Check I2C connections (SDA=A4, SCL=A5)");
    performErrorBlink();
  } else {
    Serial.println("[INIT] ✓ MPU6050 connection successful");
    
    // Configure MPU6050
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);
    mpu.setIntEnabled(true);
    Serial.println("[INIT] ✓ MPU6050 configured");
  }
  
  // Stop all motors on startup (Safety)
  Serial.println("[INIT] Stopping all motors...");
  stopMotors();
  Serial.println("[INIT] ✓ All motors stopped");
  
  // Perform startup sequence
  Serial.println("[INIT] Performing startup sequence...");
  digitalWrite(STATUS_LED, HIGH);
  delay(300);
  digitalWrite(STATUS_LED, LOW);
  delay(200);
  digitalWrite(STATUS_LED, HIGH);
  delay(300);
  digitalWrite(STATUS_LED, LOW);
  Serial.println("[INIT] ✓ Startup sequence complete");
  
  // System Ready
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║              ✓ SYSTEM READY FOR OPERATION                 ║");
  Serial.println("║         Waiting for Bluetooth connection...               ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  F = Forward    B = Backward   L = Left      R = Right");
  Serial.println("  S = Stop       r = Reset      W = LED ON    w = LED OFF");
  Serial.println("  V = Buzzer ON  v = Buzzer OFF");
  Serial.println();
  
  // Send ready signal to app
  BT.println("ADEAS_READY");
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP - Runs continuously
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  // Check for accidents continuously
  checkForAccident();
  
  // Handle LED status updates
  updateLEDStatus();
  
  // SYSTEM LOCKED STATE (After Accident)
  if (isSystemLocked) {
    if (BT.available()) {
      char cmd = BT.read();
      if (cmd == 'r') {
        systemReset();
      }
    }
    
    // Check if emergency alert timeout has passed
    if (!emergencyAlertSent) {
      unsigned long timeSinceAccident = millis() - accidentTimestamp;
      if (timeSinceAccident >= EMERGENCY_ALERT_DELAY) {
        sendAutoEmergencyAlert();
      }
    }
    return;
  }
  
  // NORMAL OPERATION - Process Bluetooth Commands
  if (BT.available()) {
    char cmd = BT.read();
    
    Serial.print("[BT] Command: ");
    Serial.println(cmd);
    
    lastSerialData = millis();
    
    // MOVEMENT COMMANDS
    if (cmd == 'F') {
      forward();
      currentMovement = 'F';
      digitalWrite(STATUS_LED, HIGH);
    }
    else if (cmd == 'B') {
      backward();
      currentMovement = 'B';
      digitalWrite(STATUS_LED, HIGH);
    }
    else if (cmd == 'L') {
      left();
      currentMovement = 'L';
      digitalWrite(STATUS_LED, HIGH);
    }
    else if (cmd == 'R') {
      right();
      currentMovement = 'R';
      digitalWrite(STATUS_LED, HIGH);
    }
    else if (cmd == 'S') {
      stopMotors();
      currentMovement = 'S';
      digitalWrite(STATUS_LED, LOW);
    }
    // LED CONTROL COMMANDS
    else if (cmd == 'W') {
      digitalWrite(STATUS_LED, HIGH);
      Serial.println("[LED] Status LED ON");
      BT.println("LED_ON");
    }
    else if (cmd == 'w') {
      digitalWrite(STATUS_LED, LOW);
      Serial.println("[LED] Status LED OFF");
      BT.println("LED_OFF");
    }
    // BUZZER CONTROL COMMANDS
    else if (cmd == 'V') {
      tone(BUZZER_PIN, ALARM_FREQUENCY_HIGH);
      Serial.println("[BUZZER] Buzzer ON");
      BT.println("BUZZER_ON");
    }
    else if (cmd == 'v') {
      noTone(BUZZER_PIN);
      Serial.println("[BUZZER] Buzzer OFF");
      BT.println("BUZZER_OFF");
    }
    // SYSTEM CONTROL COMMANDS
    else if (cmd == 'r') {
      systemReset();
    }
    else {
      Serial.print("[BT] Unknown command: ");
      Serial.println(cmd);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// MOTOR CONTROL FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void forward() {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  Serial.println("[MOTOR] Direction: FORWARD ▲");
}

void backward() {
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
  Serial.println("[MOTOR] Direction: BACKWARD ▼");
}

void left() {
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  Serial.println("[MOTOR] Direction: LEFT ◄");
}

void right() {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
  Serial.println("[MOTOR] Direction: RIGHT ►");
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  Serial.println("[MOTOR] Direction: STOPPED ■");
}

// ═══════════════════════════════════════════════════════════════════════════
// ACCIDENT DETECTION SYSTEM
// ═══════════════════════════════════════════════════════════════════════════

void checkForAccident() {
  // Check Vibration Sensor
  if (digitalRead(VIBRATION_SENSOR) == HIGH) {
    Serial.println("[SENSOR] Vibration detected!");
    triggerAccident("VIBRATION");
    return;
  }
  
  // Read MPU6050 sensor data
  mpu.getAcceleration(&accelX, &accelY, &accelZ);
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);
  mpu.getTemperature(&tempRaw);
  
  // Check for sudden acceleration
  long accelMagnitude = (long)accelX * accelX + (long)accelY * accelY + (long)accelZ * accelZ;
  
  if (accelMagnitude > (long)ACCEL_THRESHOLD * ACCEL_THRESHOLD) {
    Serial.print("[SENSOR] High acceleration detected: ");
    Serial.println(sqrt(accelMagnitude));
    triggerAccident("ACCELERATION");
    return;
  }
  
  // Check for sudden rotation
  long gyroMagnitude = (long)gyroX * gyroX + (long)gyroY * gyroY + (long)gyroZ * gyroZ;
  
  if (gyroMagnitude > (long)GYRO_THRESHOLD * GYRO_THRESHOLD) {
    Serial.print("[SENSOR] High rotation detected: ");
    Serial.println(sqrt(gyroMagnitude));
    triggerAccident("ROTATION");
    return;
  }
}

void triggerAccident(String reason) {
  if (accidentDetected) {
    return;
  }
  
  accidentDetected = true;
  isSystemLocked = true;
  accidentTimestamp = millis();
  blinkMode = true;
  
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║              ⚠️  ACCIDENT DETECTED! ⚠️                     ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");
  Serial.print("[ACCIDENT] Reason: ");
  Serial.println(reason);
  Serial.println("[ACCIDENT] Emergency procedures initiated:");
  
  Serial.println("[ACCIDENT]  1) Stopping all motors");
  stopMotors();
  
  Serial.println("[ACCIDENT]  2) Activating buzzer alarm");
  activateAlarmPattern();
  
  Serial.println("[ACCIDENT]  3) Flashing emergency LED");
  
  Serial.println("[ACCIDENT]  4) Sending emergency alert to app");
  BT.println("ACCIDENT_DETECTED");
  
  Serial.println();
  Serial.println("[ACCIDENT] System locked. Press RESET button on app to recover.");
  Serial.println("[ACCIDENT] Auto-alert will be sent in 20 seconds if not reset.");
  Serial.println();
}

void activateAlarmPattern() {
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, ALARM_FREQUENCY_HIGH);
    delay(ALARM_DURATION);
    noTone(BUZZER_PIN);
    delay(100);
  }
  
  tone(BUZZER_PIN, ALARM_FREQUENCY_LOW);
  delay(500);
  noTone(BUZZER_PIN);
}

// ═══════════════════════════════════════════════════════════════════════════
// EMERGENCY ALERT SYSTEM
// ═══════════════════════════════════════════════════════════════════════════

void sendAutoEmergencyAlert() {
  if (emergencyAlertSent) {
    return;
  }
  
  emergencyAlertSent = true;
  
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║            🆘 AUTOMATIC EMERGENCY ALERT 🆘               ║");
  Serial.println("║     No reset received within 20 seconds of accident       ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");
  
  for (int i = 0; i < 5; i++) {
    tone(BUZZER_PIN, ALARM_FREQUENCY_HIGH);
    delay(200);
    noTone(BUZZER_PIN);
    delay(100);
  }
  
  BT.println("EMERGENCY_AUTO_ALERT");
  
  Serial.println("[ALERT] Emergency auto-alert message sent to app.");
}

// ═══════════════════════════════════════════════════════════════════════════
// SYSTEM CONTROL FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void systemReset() {
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║                   SYSTEM RESET                           ║");
  Serial.println("║              System returning to normal mode             ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");
  
  accidentDetected = false;
  isSystemLocked = false;
  emergencyAlertSent = false;
  blinkMode = false;
  currentMovement = 'S';
  
  stopMotors();
  noTone(BUZZER_PIN);
  digitalWrite(STATUS_LED, LOW);
  
  BT.println("SYSTEM_RESET_ACK");
  
  Serial.println("[RESET] ✓ All systems reset");
  Serial.println("[RESET] ✓ Motors enabled");
  Serial.println("[RESET] ✓ Accident detection active");
  Serial.println("[RESET] Ready for normal operation");
  Serial.println();
}

// ═══════════════════════════════════════════════════════════════════════════
// LED STATUS INDICATOR FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void updateLEDStatus() {
  unsigned long currentTime = millis();
  
  if (blinkMode) {
    if (currentTime - lastLEDToggle >= 100) {
      ledState = !ledState;
      digitalWrite(STATUS_LED, ledState ? HIGH : LOW);
      lastLEDToggle = currentTime;
    }
  }
}

void performErrorBlink() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
  }
}

// END OF CODE
