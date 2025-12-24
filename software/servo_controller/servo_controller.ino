#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <WiFiManager.h>
#include <AccelStepper.h>

const int UDP_PORT = 8888;
const int NUM_SERVOS = 5;
const int TOTAL_JOINTS = 6;

const int SERVO_PINS[NUM_SERVOS] = {15, 27, 14, 12, 13};

const int STEPPER_STEP_PIN = 19;
const int STEPPER_DIR_PIN = 33;
const int STEPPER_EN_PIN = 32;

const int STEPS_PER_REVOLUTION = 200;
const int MICROSTEPS = 16;
const int TOTAL_STEPS = STEPS_PER_REVOLUTION * MICROSTEPS;

const float GEAR_RATIO = 10.0;

const unsigned long STEPPER_IDLE_TIMEOUT = 2000;
unsigned long last_stepper_move_time = 0;
bool stepper_enabled = false;

bool stepper_is_moving = false;
float stepper_committed_target = 0.0;

Servo servos[NUM_SERVOS];
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

WiFiUDP udp;
WiFiManager wifiManager;

float current_angles[TOTAL_JOINTS] = {0, 90, 90, 90, 90, 90};
float target_angles[TOTAL_JOINTS] = {0, 90, 90, 90, 90, 90};

const int MIN_SERVO_ANGLE = 0;
const int MAX_SERVO_ANGLE = 180;

bool diagnostic_mode = false;
bool debug_mode = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=================================");
  Serial.println("ESP32 Surgical Robot Controller");
  Serial.println("  6-DOF: Stepper + 5 Servos");
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
  Serial.println("║   DIAGNOSTIC MODE (6-DOF)          ║");
  Serial.println("╚════════════════════════════════════╝\n");
  Serial.printf("Gear Ratio: %.1f:1\n", GEAR_RATIO);
  Serial.printf("For base to rotate 360°, motor rotates %.0f°\n\n", 360.0 * GEAR_RATIO);
  
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
  
  Serial.printf("\n=== Testing Servo %d (%s) ===\n", servo_index, joint_names[servo_index]);
  Serial.println("Moving 0° -> 90° -> 180° -> 90°");
  
  Serial.println("Moving to 0°...");
  servos[servo_index].write(0);
  delay(1000);
  
  Serial.println("Moving to 90°...");
  servos[servo_index].write(90);
  delay(1000);
  
  Serial.println("Moving to 180°...");
  servos[servo_index].write(180);
  delay(1000);
  
  Serial.println("Moving to 90°...");
  servos[servo_index].write(90);
  delay(1000);
  
  Serial.println("Test complete!");
}

void testAllServos() {
  Serial.println("\n=== Testing All 5 Servos ===");
  
  Serial.println("Moving all to 0°...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(0);
  }
  delay(2000);
  
  Serial.println("Moving all to 90°...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(90);
  }
  delay(2000);
  
  Serial.println("Moving all to 180°...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(180);
  }
  delay(2000);
  
  Serial.println("Moving all to 90°...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(90);
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
  
  Serial.println("\nServos:");
  const char* joint_names[] = {"Shoulder", "Elbow", "Roll", "Pitch", "Yaw"};
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.printf("  Servo %d (%s): Pin %d\n", i, joint_names[i], SERVO_PINS[i]);
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
  
  Serial.printf("  Max speed: 2000 steps/s\n");
  Serial.printf("  Acceleration: 1000 steps/s²\n");
  Serial.printf("  Gear ratio: %.1f:1\n", GEAR_RATIO);
  Serial.printf("  Steps/base rev: %ld\n", (long)(TOTAL_STEPS * GEAR_RATIO));
  Serial.println("✓ Stepper initialized (DISABLED)");
}

void initializeServos() {
  Serial.println("\nInitializing servos...");
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  const char* joint_names[] = {"Shoulder", "Elbow", "Roll", "Pitch", "Yaw"};
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], 500, 2400);
    
    servos[i].write(90);
    current_angles[i + 1] = 90;
    target_angles[i + 1] = 90;
    
    Serial.printf("  Servo %d (%s): Pin %d\n", i, joint_names[i], SERVO_PINS[i]);
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
  delay(1);
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
      Serial.printf("Current angle: %.1f°\n", stepperStepsToAngle(stepper.currentPosition()));
      Serial.printf("Target angle: %.1f°\n", stepper_committed_target);
      Serial.printf("Is moving: %s\n", stepper_is_moving ? "YES" : "NO");
      Serial.printf("Distance to go: %ld steps\n", stepper.distanceToGo());
      Serial.printf("Debug mode: %s\n", debug_mode ? "ON" : "OFF");
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
          target_angles[i] = constrain(received_angles[i], MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
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
  const float SMOOTHING_FACTOR = 0.5;
  
  for (int i = 1; i < TOTAL_JOINTS; i++) {
    current_angles[i] = current_angles[i] * (1 - SMOOTHING_FACTOR) + 
                        target_angles[i] * SMOOTHING_FACTOR;
    
    int angle = constrain((int)current_angles[i], MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    servos[i - 1].write(angle);
  }
}

void emergencyStopAll() {
  Serial.println("🛑 EMERGENCY STOP");
  
  stepper.stop();
  disableStepper();
  stepper_is_moving = false;
  stepper_committed_target = 0;
  
  for (int i = 1; i < TOTAL_JOINTS; i++) {
    target_angles[i] = 90;
    servos[i - 1].write(90);
    current_angles[i] = 90;
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
    target_angles[i] = 90;
  }
}
