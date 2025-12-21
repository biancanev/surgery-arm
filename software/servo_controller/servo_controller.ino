#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <WiFiManager.h>
#include <AccelStepper.h>

const int UDP_PORT = 8888;
const int NUM_SERVOS = 4;
const int TOTAL_JOINTS = 5;

const int SERVO_PINS[NUM_SERVOS] = {26, 27, 14, 12};

const int STEPPER_STEP_PIN = 19;
const int STEPPER_DIR_PIN = 33;
const int STEPPER_EN_PIN = 32;

const int STEPS_PER_REVOLUTION = 200;
const int MICROSTEPS = 16;
const int TOTAL_STEPS = STEPS_PER_REVOLUTION * MICROSTEPS;

Servo servos[NUM_SERVOS];
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

WiFiUDP udp;
WiFiManager wifiManager;

float current_angles[TOTAL_JOINTS] = {0, 90, 90, 90, 90};
float target_angles[TOTAL_JOINTS] = {0, 90, 90, 90, 90};

unsigned long last_command_time = 0;
const unsigned long SAFETY_TIMEOUT_MS = 1000;

const int MIN_SERVO_ANGLE = 0;
const int MAX_SERVO_ANGLE = 180;

bool emergency_stop = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=================================");
  Serial.println("ESP32 Surgical Robot Controller");
  Serial.println("  Stepper Base + 4 Servos");
  Serial.println("=================================");
  
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
  Serial.printf("UDP Port: %d\n", UDP_PORT);
  Serial.println("\nSerial Commands:");
  Serial.println("  'reset' - Reset WiFi credentials");
  Serial.println("  'status' - Show WiFi status");
  Serial.println("  'home' - Home stepper motor");
  Serial.println("=================================\n");
  
  last_command_time = millis();
}

void checkSerialCommands() {
  Serial.println("\nChecking for serial commands...");
  Serial.println("Type 'reset' within 5 seconds to reset WiFi");
  Serial.println("Type 'skip' to skip and use saved WiFi");
  Serial.println("");
  
  unsigned long start = millis();
  String input = "";
  
  while (millis() - start < 5000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        input.trim();
        input.toLowerCase();
        
        if (input == "reset") {
          Serial.println("\n*** RESETTING WiFi credentials ***");
          wifiManager.resetSettings();
          delay(1000);
          return;
        } else if (input == "skip") {
          Serial.println("\nSkipping - using saved WiFi");
          return;
        }
        input = "";
      } else {
        input += c;
      }
    }
    delay(10);
  }
  
  Serial.println("\nNo command received - proceeding with saved WiFi");
}

void connectWiFi() {
  Serial.println("\nStarting WiFi connection...");
  
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  
  String ap_name = "SurgicalRobot-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  
  Serial.println("Attempting to connect to saved WiFi...");
  
  if (!wifiManager.autoConnect(ap_name.c_str(), "robot123")) {
    Serial.println("\nFailed to connect and hit timeout");
    Serial.println("Restarting ESP32...");
    delay(3000);
    ESP.restart();
  }
  
  Serial.println("\n✓ WiFi connected successfully!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Connected to: ");
  Serial.println(WiFi.SSID());
}

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║   ENTERED CONFIG MODE              ║");
  Serial.println("╚════════════════════════════════════╝");
  Serial.println("\nESP32 has created a WiFi access point:");
  Serial.println("");
  Serial.print("  Network Name: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
  Serial.println("  Password: robot123");
  Serial.println("");
  Serial.println("Go to: http://192.168.4.1");
  Serial.println("╚════════════════════════════════════╝\n");
}

void saveConfigCallback() {
  Serial.println("\n✓ Configuration saved!");
}

void initializeStepper() {
  Serial.println("\nInitializing stepper motor...");
  Serial.println("------------------------");
  
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);
  
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);
  stepper.setCurrentPosition(0);
  
  Serial.printf("  Stepper: STEP=%d, DIR=%d, EN=%d\n", 
                STEPPER_STEP_PIN, STEPPER_DIR_PIN, STEPPER_EN_PIN);
  Serial.printf("  Steps per revolution: %d\n", TOTAL_STEPS);
  Serial.printf("  Max speed: 2000 steps/s\n");
  Serial.printf("  Acceleration: 1000 steps/s²\n");
  Serial.println("------------------------");
  Serial.println("✓ Stepper initialized at 0°");
}

void initializeServos() {
  Serial.println("\nInitializing servos...");
  Serial.println("------------------------");
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], 500, 2400);
    
    servos[i].write(90);
    current_angles[i + 1] = 90;
    target_angles[i + 1] = 90;
    
    Serial.printf("  Servo %d (Joint %d): Pin %2d - OK\n", i, i+1, SERVO_PINS[i]);
    delay(100);
  }
  
  Serial.println("------------------------");
  Serial.println("✓ All servos initialized to 90°");
}

void loop() {
  handleSerialCommands();
  
  handleUDPCommands();
  
  checkSafetyTimeout();
  
  updateStepper();
  updateServos();
  
  delay(1);
}

void handleSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    
    if (cmd == "reset") {
      Serial.println("\n*** Resetting WiFi and restarting ***");
      wifiManager.resetSettings();
      delay(1000);
      ESP.restart();
    } else if (cmd == "status") {
      Serial.println("\n=== System Status ===");
      Serial.printf("WiFi SSID: %s\n", WiFi.SSID().c_str());
      Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
      Serial.printf("Stepper position: %.1f°\n", stepperStepsToAngle(stepper.currentPosition()));
      Serial.printf("Stepper target: %.1f°\n", target_angles[0]);
      Serial.println("====================\n");
    } else if (cmd == "home") {
      Serial.println("\n*** Homing stepper to 0° ***");
      target_angles[0] = 0;
      stepper.moveTo(0);
    } else if (cmd == "restart") {
      Serial.println("\n*** Restarting ESP32 ***");
      delay(1000);
      ESP.restart();
    } else if (cmd == "help") {
      Serial.println("\n=== Available Commands ===");
      Serial.println("  reset   - Reset WiFi credentials");
      Serial.println("  status  - Show system status");
      Serial.println("  home    - Home stepper motor");
      Serial.println("  restart - Restart ESP32");
      Serial.println("  help    - Show this message");
      Serial.println("=========================\n");
    }
  }
}

void handleUDPCommands() {
  int packet_size = udp.parsePacket();
  
  if (packet_size > 0) {
    uint8_t buffer[64];
    int len = udp.read(buffer, sizeof(buffer));
    
    if (len == 20) {
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
        target_angles[0] = received_angles[0];
        
        for (int i = 1; i < TOTAL_JOINTS; i++) {
          target_angles[i] = constrain(received_angles[i], MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
        }
        
        long target_steps = stepperAngleToSteps(target_angles[0]);
        stepper.moveTo(target_steps);
        
        last_command_time = millis();
        emergency_stop = false;
        
        Serial.print("RX: ");
        Serial.printf("Base=%.1f° ", target_angles[0]);
        for (int i = 1; i < TOTAL_JOINTS; i++) {
          Serial.printf("J%d=%.1f° ", i, target_angles[i]);
        }
        Serial.println();
      } else {
        Serial.println("✗ Invalid packet data");
      }
      
    } else if (len == 1) {
      if (buffer[0] == 0xFF) {
        emergencyStopAll();
      } else if (buffer[0] == 0x00) {
        homeAll();
      }
    } else {
      Serial.printf("✗ Invalid packet size: %d bytes (expected 20)\n", len);
    }
  }
}

long stepperAngleToSteps(float angle) {
  float normalized_angle = fmod(angle, 360.0);
  if (normalized_angle < 0) normalized_angle += 360.0;
  
  return (long)((normalized_angle / 360.0) * TOTAL_STEPS);
}

float stepperStepsToAngle(long steps) {
  return (float)(steps % TOTAL_STEPS) * 360.0 / TOTAL_STEPS;
}

void updateStepper() {
  if (stepper.distanceToGo() != 0) {
    stepper.run();
    
    float current_angle = stepperStepsToAngle(stepper.currentPosition());
    current_angles[0] = current_angle;
  }
}

void updateServos() {
  const float SMOOTHING_FACTOR = 0.3;
  
  for (int i = 1; i < TOTAL_JOINTS; i++) {
    current_angles[i] = current_angles[i] * (1 - SMOOTHING_FACTOR) + 
                        target_angles[i] * SMOOTHING_FACTOR;
    
    int angle = constrain((int)current_angles[i], MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    servos[i - 1].write(angle);
  }
}

void checkSafetyTimeout() {
  if (millis() - last_command_time > SAFETY_TIMEOUT_MS) {
    if (!emergency_stop) {
      Serial.println("\n⚠ SAFETY TIMEOUT - No commands received");
      emergencyStopAll();
    }
  }
}

void emergencyStopAll() {
  Serial.println("🛑 EMERGENCY STOP ACTIVATED");
  emergency_stop = true;
  
  stepper.stop();
  digitalWrite(STEPPER_EN_PIN, HIGH);
  
  for (int i = 1; i < TOTAL_JOINTS; i++) {
    target_angles[i] = 90;
    servos[i - 1].write(90);
    current_angles[i] = 90;
  }
}

void homeAll() {
  Serial.println("🏠 Returning to home position");
  emergency_stop = false;
  
  digitalWrite(STEPPER_EN_PIN, LOW);
  
  target_angles[0] = 0;
  stepper.moveTo(0);
  
  for (int i = 1; i < TOTAL_JOINTS; i++) {
    target_angles[i] = 90;
  }
  
  last_command_time = millis();
}