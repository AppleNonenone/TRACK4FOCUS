/*
TRACK4FOCUS Arduino Code - 4-Axis Face Tracking Robot
Target-Responsive Actuating using Calibrated Kinematics for Face-Oriented Controlled Unified System

4-Axis Robot Configuration:
- Base servo (Pin 3): Horizontal rotation (left/right) - azimuth control
- Shoulder servo (Pin 5): Vertical rotation (up/down) - elevation control  
- Elbow servo (Pin 6): Reach extension (up/down) - distance control
- Wrist servo (Pin 9): Fine positioning (up/down) - final adjustment
- Laser pointer (Pin 10): Face tracking indicator

Data Format: "base,shoulder,elbow,wrist,laser\n"
Example: "90,90,90,90,1\n"

Movement Ranges:
- Base: 0-180° (0=full left, 90=center, 180=full right)
- Shoulder: 30-150° (30=down, 90=level, 150=up)
- Elbow: 45-135° (45=retracted, 90=normal, 135=extended)
- Wrist: 45-135° (45=down, 90=level, 135=up)
*/

#include <Servo.h>

// Create servo objects
Servo baseServo;
Servo shoulderServo; 
Servo elbowServo;
Servo wristServo;

// Pin definitions
const int BASE_PIN = 3;
const int SHOULDER_PIN = 5;
const int ELBOW_PIN = 6;
const int WRIST_PIN = 9;
const int LASER_PIN = 10;

// Current positions
int currentBase = 90;
int currentShoulder = 90;
int currentElbow = 90;
int currentWrist = 90;
int currentLaser = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Wait for serial connection to be established (important for some systems)
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  // Small delay to ensure stable connection
  delay(500);
  
  // Attach servos to pins
  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  wristServo.attach(WRIST_PIN);
  
  // Setup laser pin
  pinMode(LASER_PIN, OUTPUT);
  
  // Move to neutral position
  moveToNeutral();
  
  // Send ready message
  Serial.println("TRACK4FOCUS Arduino Ready");
  Serial.println("Waiting for servo commands...");
  Serial.flush(); // Ensure message is sent
}

void loop() {
  if (Serial.available() > 0) {
    // Read until newline or timeout
    String command = "";
    unsigned long startTime = millis();
    const unsigned long timeout = 100; // 100ms timeout
    
    while (millis() - startTime < timeout) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
          break;
        }
        command += c;
      }
    }
    
    command.trim();
    
    if (command.length() > 0) {
      parseAndExecuteCommand(command);
    }
  }
}

void parseAndExecuteCommand(String command) {
  // Heartbeat 명령 처리 (연결 확인)
  if (command.equalsIgnoreCase("PING")) {
    Serial.println("PONG");
    Serial.flush();
    return;
  }
  
  // Parse command format: "base,shoulder,elbow,wrist,laser"
  int commaIndex1 = command.indexOf(',');
  int commaIndex2 = command.indexOf(',', commaIndex1 + 1);
  int commaIndex3 = command.indexOf(',', commaIndex2 + 1);
  int commaIndex4 = command.indexOf(',', commaIndex3 + 1);
  
  if (commaIndex1 == -1 || commaIndex2 == -1 || commaIndex3 == -1 || commaIndex4 == -1) {
    Serial.println("Error: Invalid command format");
    return;
  }
  
  // Extract values
  int base = command.substring(0, commaIndex1).toInt();
  int shoulder = command.substring(commaIndex1 + 1, commaIndex2).toInt();
  int elbow = command.substring(commaIndex2 + 1, commaIndex3).toInt();
  int wrist = command.substring(commaIndex3 + 1, commaIndex4).toInt();
  int laser = command.substring(commaIndex4 + 1).toInt();
  
  // Validate ranges
  base = constrain(base, 0, 180);
  shoulder = constrain(shoulder, 0, 180);
  elbow = constrain(elbow, 0, 180);
  wrist = constrain(wrist, 0, 180);
  laser = constrain(laser, 0, 1);
  
  // Move servos smoothly
  moveServosSmooth(base, shoulder, elbow, wrist);
  
  // Control laser
  digitalWrite(LASER_PIN, laser ? HIGH : LOW);
  
  // Update current positions
  currentBase = base;
  currentShoulder = shoulder;
  currentElbow = elbow;
  currentWrist = wrist;
  currentLaser = laser;
  
  // Send confirmation
  Serial.print("OK: ");
  Serial.print(base); Serial.print(",");
  Serial.print(shoulder); Serial.print(",");
  Serial.print(elbow); Serial.print(",");
  Serial.print(wrist); Serial.print(",");
  Serial.println(laser);
  Serial.flush(); // Ensure response is sent immediately
}

void moveServosSmooth(int targetBase, int targetShoulder, int targetElbow, int targetWrist) {
  // Calculate steps for smooth movement
  int steps = 10;
  int delayTime = 20; // milliseconds
  
  for (int i = 1; i <= steps; i++) {
    int base = currentBase + (targetBase - currentBase) * i / steps;
    int shoulder = currentShoulder + (targetShoulder - currentShoulder) * i / steps;
    int elbow = currentElbow + (targetElbow - currentElbow) * i / steps;
    int wrist = currentWrist + (targetWrist - currentWrist) * i / steps;
    
    baseServo.write(base);
    shoulderServo.write(shoulder);
    elbowServo.write(elbow);
    wristServo.write(wrist);
    
    delay(delayTime);
  }
}

void moveToNeutral() {
  baseServo.write(90);
  shoulderServo.write(90);
  elbowServo.write(90);
  wristServo.write(90);
  digitalWrite(LASER_PIN, LOW);
  
  currentBase = 90;
  currentShoulder = 90;
  currentElbow = 90;
  currentWrist = 90;
  currentLaser = 0;
  
  delay(1000); // Allow time to reach position
}