#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFiManager.h>
#include <AccelStepper.h>

void setup();
void loop();
void handleSerialCommands();
void handleUDPCommands();
void updateStepper();
void updateServos();
void checkStepperTimeout();
void enableStepper();
void disableStepper();
void forceDisableStepper();
void moveStepperToAngle(float target_angle);
long stepperAngleToSteps(float base_angle_degrees);
float stepperStepsToAngle(long steps);
float normalizeAngleDelta(float delta);
void emergencyStopAll();
void homeAll();
void connectWiFi();
void initializeStepper();
void initializeServos();
void runDiagnostics();
void testStepperAngle(float base_angle_degrees);
void testServo(int servo_index);
void testAllServos();
void testBidirectionalRotation();
void checkPinStates();
void checkSerialCommands();
void configModeCallback(WiFiManager *myWiFiManager);
void saveConfigCallback();

const int UDP_PORT = 8888;
const int NUM_SERVOS = 5;
const int TOTAL_JOINTS = 6;

const int SDA_PIN = 8;
const int SCL_PIN = 9;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const int SERVO_MIN_PULSE = 190;
const int SERVO_MAX_PULSE = 440;

const int STEPPER_STEP_PIN = 5;
const int STEPPER_DIR_PIN = 6;
const int STEPPER_EN_PIN = 7;

const int STEPS_PER_REVOLUTION = 200;
const int MICROSTEPS = 16;
const int TOTAL_STEPS = STEPS_PER_REVOLUTION * MICROSTEPS;

const float GEAR_RATIO = 10.0;

const unsigned long STEPPER_IDLE_TIMEOUT = 2000;
unsigned long last_stepper_move_time = 0;
bool stepper_enabled = false;

bool stepper_is_moving = false;
float stepper_committed_target = 0.0;

float servo_velocities[5] = {0, 0, 0, 0, 0};

AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

WiFiUDP udp;
WiFiManager wifiManager;

float current_angles[TOTAL_JOINTS] = {0, 90, 90, 90, 90, 90};
float target_angles[TOTAL_JOINTS] = {0, 90, 90, 90, 90, 90};

const int MIN_SERVO_ANGLE = 0;
const int MAX_SERVO_ANGLE = 180;

bool diagnostic_mode = false;
bool debug_mode = false;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=================================");
  Serial.println("ESP32-S3 Surgical Robot Controller");
  Serial.println("  6-DOF: Stepper + 5 Servos (PCA9685)");
  Serial.printf("  Gear Ratio: %.1f:1\n", GEAR_RATIO);
  Serial.println("=================================");
  
  Serial.println("\nType 'diag' for diagnostic mode");
  Serial.println("Type 'debug' for debug mode");
  Serial.println("Or press ENTER to continue normally");
  
  unsigned long start = millis();
  String input = "";
  
  while (millis() - start < 5000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        input.trim();
        input.toLowerCase();
        
        if (input == "diag") {
          diagnostic_mode = true;
          runDiagnostics();
          return;
        } else if (input == "debug") {
          debug_mode = true;
          Serial.println("Debug mode enabled");
          break;
        }
        break;
      } else {
        input += c;
      }
    }
    delay(10);
  }
  
  checkSerialCommands();
  connectWiFi();
  
  udp.begin(UDP_PORT);
  Serial.printf("\nUDP server listening on port %d\n", UDP_PORT);
  
  initializePCA9685();
  initializeStepper();
  initializeServos();
  
  Serial.println("\n=================================");
  Serial.println("=== READY FOR COMMANDS ===");
  Serial.println("=================================");
  Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
  if (debug_mode) {
    Serial.println("Debug mode: ON");
  }
  Serial.println("=================================\n");
}

void initializePCA9685() {
  Serial.println("\nInitializing PCA9685...");
  
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  
  pwm.begin();
  delay(10);
  
  pwm.setPWMFreq(50);
  delay(10);
  
  Wire.beginTransmission(0x40);
  if (Wire.endTransmission() != 0) {
    Serial.println("✗ WARNING: PCA9685 not responding at 0x40!");
    Serial.println("\nTroubleshooting:");
    Serial.println("  1. Check I2C wiring:");
    Serial.printf("     - SDA: GPIO %d → PCA9685 SDA\n", SDA_PIN);
    Serial.printf("     - SCL: GPIO %d → PCA9685 SCL\n", SCL_PIN);
    Serial.println("  2. Check power:");
    Serial.println("     - VCC: 3.3V → PCA9685 VCC");
    Serial.println("     - GND: Common ground");
    Serial.println("  3. Run i2c_scanner.ino to verify address");
    Serial.println("  4. Check PCA9685 address jumpers");
    Serial.println("\nAttempting to continue anyway...\n");
    delay(2000);
  } else {
    Serial.printf("  I2C SDA: GPIO %d\n", SDA_PIN);
    Serial.printf("  I2C SCL: GPIO %d\n", SCL_PIN);
    Serial.println("  I2C Address: 0x40 ✓");
    Serial.println("  PWM Frequency: 50 Hz");
    Serial.printf("  Pulse Range: %d-%d (0°-180°)\n", SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    Serial.println("✓ PCA9685 initialized successfully");
  }
}

void setPWMServo(uint8_t channel, float angle) {
  angle = constrain(angle, 0.0, 180.0);
  
  float pulse_float = mapFloat(angle, 0.0, 180.0, 
                               (float)SERVO_MIN_PULSE, 
                               (float)SERVO_MAX_PULSE);
  
  int pulse = (int)(pulse_float + 0.5);
  
  pwm.setPWM(channel, 0, pulse);
}

void enableStepper() {
  if (!stepper_enabled) {
    digitalWrite(STEPPER_EN_PIN, LOW);
    stepper_enabled = true;
    if (debug_mode) Serial.println("🔌 Stepper enabled");
    delay(10);
  }
}

void disableStepper() {
  digitalWrite(STEPPER_EN_PIN, HIGH);
  if (stepper_enabled && debug_mode) {
    Serial.println("💤 Stepper disabled");
  }
  stepper_enabled = false;
}

void forceDisableStepper() {
  digitalWrite(STEPPER_EN_PIN, HIGH);
  stepper_enabled = false;
}

void runDiagnostics() {
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║   DIAGNOSTIC MODE (6-DOF PCA9685)  ║");
  Serial.println("╚════════════════════════════════════╝\n");
  Serial.printf("Gear Ratio: %.1f:1\n", GEAR_RATIO);
  Serial.printf("For base to rotate 360°, motor rotates %.0f°\n\n", 360.0 * GEAR_RATIO);
  
  initializePCA9685();
  initializeStepper();
  initializeServos();
  
  while (true) {
    Serial.println("\n=== Diagnostic Menu ===");
    Serial.println("1. Test stepper - 45° base rotation");
    Serial.println("2. Test stepper - 90° base rotation");
    Serial.println("3. Test stepper - 180° base rotation");
    Serial.println("4. Test stepper - 360° base rotation");
    Serial.println("5. Test servo 0 (Shoulder)");
    Serial.println("6. Test servo 1 (Elbow)");
    Serial.println("7. Test servo 2 (Roll)");
    Serial.println("8. Test servo 3 (Pitch)");
    Serial.println("9. Test servo 4 (Yaw)");
    Serial.println("a. Test all servos");
    Serial.println("b. Test bidirectional rotation");
    Serial.println("c. Calibrate servo pulse widths");
    Serial.println("s. Show status");
    Serial.println("0. Exit diagnostics");
    Serial.println("\nEnter choice: ");
    
    while (!Serial.available()) {
      delay(10);
    }
    
    char choice = Serial.read();
    while (Serial.available()) Serial.read();
    
    switch (choice) {
      case '1':
        testStepperAngle(45);
        break;
      case '2':
        testStepperAngle(90);
        break;
      case '3':
        testStepperAngle(180);
        break;
      case '4':
        testStepperAngle(360);
        break;
      case '5':
        testServo(0);
        break;
      case '6':
        testServo(1);
        break;
      case '7':
        testServo(2);
        break;
      case '8':
        testServo(3);
        break;
      case '9':
        testServo(4);
        break;
      case 'a':
      case 'A':
        testAllServos();
        break;
      case 'b':
      case 'B':
        testBidirectionalRotation();
        break;
      case 'c':
      case 'C':
        calibratePulseWidths();
        break;
      case 's':
      case 'S':
        checkPinStates();
        break;
      case '0':
        Serial.println("Exiting diagnostics. Restarting...");
        delay(1000);
        ESP.restart();
        break;
      default:
        Serial.println("Invalid choice");
        break;
    }
  }
}

void calibratePulseWidths() {
  Serial.println("\n=== Servo Pulse Width Calibration ===");
  Serial.println("This helps you find the correct SERVO_MIN_PULSE and SERVO_MAX_PULSE");
  Serial.println("for your specific servos.\n");
  
  Serial.println("Select servo to calibrate (0-4): ");
  while (!Serial.available()) delay(10);
  int servo_num = Serial.read() - '0';
  while (Serial.available()) Serial.read();
  
  if (servo_num < 0 || servo_num >= NUM_SERVOS) {
    Serial.println("Invalid servo number");
    return;
  }
  
  const char* joint_names[] = {"Shoulder", "Elbow", "Roll", "Pitch", "Yaw"};
  Serial.printf("\nCalibrating Servo %d (%s) on Channel %d\n\n", 
                servo_num, joint_names[servo_num], servo_num);
  
  int test_pulse = SERVO_MIN_PULSE;
  
  Serial.println("Commands:");
  Serial.println("  +/- : Adjust pulse by 10");
  Serial.println("  </> : Adjust pulse by 1");
  Serial.println("  0   : Test 0° position");
  Serial.println("  9   : Test 90° position");
  Serial.println("  1   : Test 180° position");
  Serial.println("  q   : Quit calibration\n");
  
  Serial.printf("Starting pulse: %d\n", test_pulse);
  pwm.setPWM(servo_num, 0, test_pulse);
  
  while (true) {
    if (Serial.available()) {
      char cmd = Serial.read();
      while (Serial.available()) Serial.read();
      
      switch(cmd) {
        case '+':
          test_pulse += 10;
          Serial.printf("Pulse: %d\n", test_pulse);
          pwm.setPWM(servo_num, 0, test_pulse);
          break;
        case '-':
          test_pulse -= 10;
          Serial.printf("Pulse: %d\n", test_pulse);
          pwm.setPWM(servo_num, 0, test_pulse);
          break;
        case '>':
          test_pulse += 1;
          Serial.printf("Pulse: %d\n", test_pulse);
          pwm.setPWM(servo_num, 0, test_pulse);
          break;
        case '<':
          test_pulse -= 1;
          Serial.printf("Pulse: %d\n", test_pulse);
          pwm.setPWM(servo_num, 0, test_pulse);
          break;
        case '0':
          Serial.println("Testing 0° position...");
          setPWMServo(servo_num, 0.0);
          delay(1000);
          break;
        case '9':
          Serial.println("Testing 90° position...");
          setPWMServo(servo_num, 90.0);
          delay(1000);
          break;
        case '1':
          Serial.println("Testing 180° position...");
          setPWMServo(servo_num, 180.0);
          delay(1000);
          break;
        case 'q':
        case 'Q':
          Serial.println("\nCalibration complete!");
          Serial.println("Note the pulse values that worked:");
          Serial.println("  - Pulse for 0° (no buzzing)");
          Serial.println("  - Pulse for 180° (no buzzing)");
          Serial.println("\nUpdate in code:");
          Serial.println("  const int SERVO_MIN_PULSE = ???;");
          Serial.println("  const int SERVO_MAX_PULSE = ???;");
          return;
        default:
          Serial.println("Unknown command");
          break;
      }
    }
    delay(10);
  }
}

void testBidirectionalRotation() {
  Serial.println("\n=== Testing Bidirectional Rotation ===");
  Serial.println("This tests shortest-path movement");
  
  enableStepper();
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);
  
  Serial.println("\n1. Moving to 10°...");
  moveStepperToAngle(10);
  delay(1000);
  
  Serial.println("2. Moving to 350° (should go backwards -20°)...");
  moveStepperToAngle(350);
  delay(1000);
  
  Serial.println("3. Moving to 10° (should go forwards +20°)...");
  moveStepperToAngle(10);
  delay(1000);
  
  Serial.println("4. Moving to 180° (should go forwards +170°)...");
  moveStepperToAngle(180);
  delay(1000);
  
  Serial.println("5. Returning to 0°...");
  moveStepperToAngle(0);
  
  delay(2000);
  disableStepper();
  Serial.println("Test complete!\n");
}

void moveStepperToAngle(float target_angle) {
  float current_angle = stepperStepsToAngle(stepper.currentPosition());
  
  float delta = target_angle - current_angle;
  delta = normalizeAngleDelta(delta);
  
  float new_angle = current_angle + delta;
  long target_steps = stepperAngleToSteps(new_angle);
  
  Serial.printf("  Current: %.1f°, Target: %.1f°, Delta: %.1f°, Steps: %ld\n", 
                current_angle, target_angle, delta, target_steps);
  
  stepper.moveTo(target_steps);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    delay(1);
  }
}

void testStepperAngle(float base_angle_degrees) {
  Serial.printf("\n=== Testing Stepper: %.0f° Base Rotation ===\n", base_angle_degrees);
  
  long target_steps = stepperAngleToSteps(base_angle_degrees);
  float motor_rotations = (float)target_steps / TOTAL_STEPS;
  
  Serial.printf("Base angle: %.0f°\n", base_angle_degrees);
  Serial.printf("Motor steps: %ld\n", target_steps);
  Serial.printf("Motor rotations: %.2f\n\n", motor_rotations);
  
  enableStepper();
  
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);
  stepper.moveTo(target_steps);
  
  Serial.println("Moving...");
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    delay(1);
  }
  
  Serial.println("Target reached! Waiting 2 seconds...");
  delay(2000);
  disableStepper();
  
  delay(1000);
  
  enableStepper();
  Serial.println("Returning to 0°...");
  stepper.moveTo(0);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    delay(1);
  }
  
  delay(2000);
  disableStepper();
  Serial.println("Test complete\n");
}

void testServo(int servo_index) {
  if (servo_index < 0 || servo_index >= NUM_SERVOS) {
    Serial.println("Invalid servo index");
    return;
  }
  
  const char* joint_names[] = {"Shoulder", "Elbow", "Roll", "Pitch", "Yaw"};
  
  Serial.printf("\n=== Testing Servo %d (%s) on PCA9685 Channel %d ===\n", 
                servo_index, joint_names[servo_index], servo_index);
  Serial.println("Moving 0° -> 90° -> 180° -> 90°");
  Serial.println("Watch for buzzing at endpoints!\n");
  
  Serial.println("Moving to 0°...");
  setPWMServo(servo_index, 0.0);
  delay(1500);
  
  Serial.println("Moving to 90°...");
  setPWMServo(servo_index, 90.0);
  delay(1500);
  
  Serial.println("Moving to 180°...");
  setPWMServo(servo_index, 180.0);
  delay(1500);
  
  Serial.println("Moving to 90°...");
  setPWMServo(servo_index, 90.0);
  delay(1000);
  
  Serial.println("\n✓ Test complete!");
  Serial.println("If servo buzzed or didn't reach endpoints:");
  Serial.println("  - Run option 'c' to calibrate pulse widths");
  Serial.println("  - Or adjust SERVO_MIN_PULSE / SERVO_MAX_PULSE in code");
}

void testAllServos() {
  Serial.println("\n=== Testing All 5 Servos ===");
  
  Serial.println("Moving all to 0°...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    setPWMServo(i, 0.0);
  }
  delay(2000);
  
  Serial.println("Moving all to 90°...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    setPWMServo(i, 90.0);
  }
  delay(2000);
  
  Serial.println("Moving all to 180°...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    setPWMServo(i, 180.0);
  }
  delay(2000);
  
  Serial.println("Moving all to 90°...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    setPWMServo(i, 90.0);
  }
  delay(2000);
  
  Serial.println("Test complete!");
}

void checkPinStates() {
  Serial.println("\n=== System Status ===");
  
  Serial.printf("Gear Ratio: %.1f:1\n", GEAR_RATIO);
  Serial.printf("Steps per motor revolution: %d\n", TOTAL_STEPS);
  Serial.printf("Steps per base revolution: %ld\n", (long)(TOTAL_STEPS * GEAR_RATIO));
  
  Serial.printf("\nStepper enabled: %s\n", stepper_enabled ? "YES" : "NO");
  Serial.printf("Stepper position: %ld steps (%.1f° base)\n", 
                stepper.currentPosition(), 
                stepperStepsToAngle(stepper.currentPosition()));
  Serial.printf("Stepper target: %ld steps (%.1f° base)\n", 
                stepper.targetPosition(),
                stepperStepsToAngle(stepper.targetPosition()));
  
  Serial.println("\nPCA9685 Configuration:");
  Serial.printf("  I2C Address: 0x40\n");
  Serial.printf("  PWM Frequency: 50 Hz\n");
  Serial.printf("  Pulse Range: %d-%d counts\n", SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  Serial.println("\nStepper Pins:");
  Serial.printf("  STEP: GPIO %d\n", STEPPER_STEP_PIN);
  Serial.printf("  DIR:  GPIO %d\n", STEPPER_DIR_PIN);
  Serial.printf("  EN:   GPIO %d\n", STEPPER_EN_PIN);
  
  Serial.println("\nPCA9685 Servos:");
  const char* joint_names[] = {"Shoulder", "Elbow", "Roll", "Pitch", "Yaw"};
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.printf("  Servo %d (%s): Channel %d, Current angle: %.1f°\n", 
                  i, joint_names[i], i, current_angles[i+1]);
  }
  Serial.println();
}

void checkSerialCommands() {
  Serial.println("\nType 'reset' within 5 seconds to reset WiFi");
  
  unsigned long start = millis();
  String input = "";
  
  while (millis() - start < 5000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        input.trim();
        input.toLowerCase();
        
        if (input == "reset") {
          Serial.println("\n*** RESETTING WiFi ***");
          wifiManager.resetSettings();
          delay(1000);
          return;
        }
        input = "";
      } else {
        input += c;
      }
    }
    delay(10);
  }
  
  Serial.println("Proceeding...");
}

void connectWiFi() {
  Serial.println("\nStarting WiFi...");
  
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  
  String ap_name = "SurgicalRobot-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  
  if (!wifiManager.autoConnect(ap_name.c_str(), "robot123")) {
    Serial.println("\nFailed to connect");
    delay(3000);
    ESP.restart();
  }
  
  Serial.println("\n✓ WiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║   CONFIG MODE                      ║");
  Serial.println("╚════════════════════════════════════╝");
  Serial.print("Network: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
  Serial.println("Password: robot123");
}

void saveConfigCallback() {
  Serial.println("\n✓ Config saved!");
}

void initializeStepper() {
  Serial.println("\nInitializing stepper...");
  
  pinMode(STEPPER_EN_PIN, OUTPUT);
  forceDisableStepper();
  
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);
  stepper.setCurrentPosition(0);
  
  Serial.printf("  STEP pin: GPIO %d\n", STEPPER_STEP_PIN);
  Serial.printf("  DIR pin:  GPIO %d\n", STEPPER_DIR_PIN);
  Serial.printf("  EN pin:   GPIO %d\n", STEPPER_EN_PIN);
  Serial.printf("  Max speed: 2000 steps/s\n");
  Serial.printf("  Acceleration: 1000 steps/s²\n");
  Serial.printf("  Gear ratio: %.1f:1\n", GEAR_RATIO);
  Serial.printf("  Steps/base rev: %ld\n", (long)(TOTAL_STEPS * GEAR_RATIO));
  Serial.println("✓ Stepper initialized (DISABLED)");
}

void initializeServos() {
  Serial.println("\nInitializing servos on PCA9685...");
  
  const char* joint_names[] = {"Shoulder", "Elbow", "Roll", "Pitch", "Yaw"};
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    setPWMServo(i, 90.0);
    current_angles[i + 1] = 90.0;
    target_angles[i + 1] = 90.0;
    
    Serial.printf("  Servo %d (%s): Channel %d → 90°\n", i, joint_names[i], i);
    delay(100);
  }
  
  Serial.println("✓ All servos at 90°");
}

void loop() {
  handleSerialCommands();
  handleUDPCommands();
  updateStepper();
  updateServos();
  checkStepperTimeout();
  delayMicroseconds(100);
}

void handleSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    
    if (cmd == "debug") {
      debug_mode = !debug_mode;
      Serial.printf("Debug mode: %s\n", debug_mode ? "ON" : "OFF");
    } else if (cmd == "status") {
      Serial.println("\n=== Status ===");
      Serial.printf("Stepper angle: %.1f°\n", stepperStepsToAngle(stepper.currentPosition()));
      Serial.printf("Stepper target: %.1f°\n", stepper_committed_target);
      Serial.printf("Is moving: %s\n", stepper_is_moving ? "YES" : "NO");
      Serial.printf("Distance to go: %ld steps\n", stepper.distanceToGo());
      Serial.printf("Debug mode: %s\n", debug_mode ? "ON" : "OFF");
      Serial.printf("PCA9685 pulse range: %d-%d\n", SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    } else if (cmd == "home") {
      homeAll();
    } else if (cmd == "help") {
      Serial.println("\n=== Commands ===");
      Serial.println("  debug  - Toggle debug mode");
      Serial.println("  status - Show status");
      Serial.println("  home   - Home all motors");
      Serial.println("  help   - This message");
    }
  }
}

void handleUDPCommands() {
  int packet_size = udp.parsePacket();
  
  if (packet_size > 0) {
    uint8_t buffer[64];
    int len = udp.read(buffer, sizeof(buffer));
    
    if (len == 24) {
      float* received_angles = (float*)buffer;
      
      bool valid = true;
      for (int i = 0; i < TOTAL_JOINTS; i++) {
        if (isnan(received_angles[i]) || isinf(received_angles[i])) {
          valid = false;
          break;
        }
        
        if (i == 0) {
          if (received_angles[i] < -720 || received_angles[i] > 720) {
            valid = false;
            break;
          }
        } else {
          if (received_angles[i] < -10 || received_angles[i] > 190) {
            valid = false;
            break;
          }
        }
      }
      
      if (valid) {
        float new_base_angle = received_angles[0];
        
        for (int i = 1; i < TOTAL_JOINTS; i++) {
          target_angles[i] = constrain(received_angles[i], (float)MIN_SERVO_ANGLE, (float)MAX_SERVO_ANGLE);
        }
        
        if (!stepper_is_moving) {
          float current_angle = stepperStepsToAngle(stepper.currentPosition());
          
          float delta = new_base_angle - current_angle;
          delta = normalizeAngleDelta(delta);
          
          float angle_difference = abs(delta);
          
          if (angle_difference > 2.0) {
            float actual_target_angle = current_angle + delta;
            stepper_committed_target = new_base_angle;
            long new_target_steps = stepperAngleToSteps(actual_target_angle);
            
            if (debug_mode) {
              Serial.printf("[CMD] Current: %.1f°, Requested: %.1f°, Delta: %.1f°, Moving to: %.1f° (%ld steps)\n", 
                            current_angle, new_base_angle, delta, actual_target_angle, new_target_steps);
            }
            
            enableStepper();
            stepper.moveTo(new_target_steps);
            stepper_is_moving = true;
            target_angles[0] = new_base_angle;
            last_stepper_move_time = millis();
            
            Serial.printf("Moving base: %.1f° -> %.1f° (delta: %+.1f°)\n", 
                          current_angle, new_base_angle, delta);
          }
        } else {
          if (debug_mode) {
            Serial.printf("[BUSY] Ignoring new angle %.1f° (still moving to %.1f°)\n", 
                          new_base_angle, stepper_committed_target);
          }
        }
      }
      
    } else if (len == 1) {
      if (buffer[0] == 0xFF) {
        emergencyStopAll();
      } else if (buffer[0] == 0x00) {
        homeAll();
      }
    }
  }
}

long stepperAngleToSteps(float base_angle_degrees) {
  float motor_angle = base_angle_degrees * GEAR_RATIO;
  long steps = (long)((motor_angle / 360.0) * TOTAL_STEPS);
  return steps;
}

float stepperStepsToAngle(long steps) {
  float motor_angle = (float)steps * 360.0 / TOTAL_STEPS;
  float base_angle = motor_angle / GEAR_RATIO;
  return base_angle;
}

float normalizeAngleDelta(float delta) {
  while (delta > 180.0) delta -= 360.0;
  while (delta < -180.0) delta += 360.0;
  return delta;
}

void updateStepper() {
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    last_stepper_move_time = millis();
    
    float current_angle = stepperStepsToAngle(stepper.currentPosition());
    current_angles[0] = current_angle;

    delay(1);
  }

  stepper_is_moving = false;
}

void checkStepperTimeout() {
  if (stepper_enabled && stepper.distanceToGo() == 0) {
    if (millis() - last_stepper_move_time > STEPPER_IDLE_TIMEOUT) {
      disableStepper();
    }
  }
}

void updateServos() {
  static unsigned long last_update = 0;
  
  if (last_update == 0) {
    last_update = micros();
    return;
  }
  
  unsigned long now = micros();
  float dt = (now - last_update) / 1000000.0;
  last_update = now;
  
  const float MAX_SPEED = 60.0;
  const float ACCELERATION = 120.0;
  
  for (int i = 1; i < TOTAL_JOINTS; i++) {
    int servo_idx = i - 1;
    float error = target_angles[i] - current_angles[i];
    
    float desired_velocity = error * 3.0;
    desired_velocity = constrain(desired_velocity, -MAX_SPEED, MAX_SPEED);
    
    float velocity_error = desired_velocity - servo_velocities[servo_idx];
    float max_accel_change = ACCELERATION * dt;
    
    if (abs(velocity_error) > max_accel_change) {
      if (velocity_error > 0) {
        servo_velocities[servo_idx] += max_accel_change;
      } else {
        servo_velocities[servo_idx] -= max_accel_change;
      }
    } else {
      servo_velocities[servo_idx] = desired_velocity;
    }
    
    current_angles[i] += servo_velocities[servo_idx] * dt;
    
    if (abs(error) < 0.5 && abs(servo_velocities[servo_idx]) < 1.0) {
      current_angles[i] = target_angles[i];
      servo_velocities[servo_idx] = 0;
    }
    
    float angle = constrain(current_angles[i], 
                           (float)MIN_SERVO_ANGLE, 
                           (float)MAX_SERVO_ANGLE);
    setPWMServo(servo_idx, angle);
  }
}

void emergencyStopAll() {
  Serial.println("🛑 EMERGENCY STOP");
  
  stepper.stop();
  disableStepper();
  stepper_is_moving = false;
  stepper_committed_target = 0;
  
  for (int i = 1; i < TOTAL_JOINTS; i++) {
    target_angles[i] = 90.0;
    setPWMServo(i - 1, 90.0);
    current_angles[i] = 90.0;
  }
}

void homeAll() {
  Serial.println("🏠 Homing");
  
  stepper_committed_target = 0;
  target_angles[0] = 0;
  stepper.moveTo(0);
  enableStepper();
  stepper_is_moving = true;
  last_stepper_move_time = millis();
  
  for (int i = 1; i < TOTAL_JOINTS; i++) {
    target_angles[i] = 90.0;
  }
}
