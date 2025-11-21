/*
  Simple 2-Servo Gripper Controller for Leap Motion Gesture Recognition
  Controls 2 SG90 servo motors for gripper open/close based on serial commands from Python
  
  Hardware Setup:
  - Arduino Uno/Nano
  - 2x SG90 Micro Servos (9g) for gripper
  - External 5V power supply for servos (recommended)
  
  Servo Connections:
  - Left Gripper: Pin 9
  - Right Gripper: Pin 10
*/

#include <Servo.h>

Servo gripperLeft;
Servo gripperRight;

// Current gripper state
int gripperState = 1; // 0=closed, 1=half, 2=open
bool moving = false;

// Movement settings
const int MOVE_DELAY = 15;  // Delay between steps (ms)
const int STEP_SIZE = 3;    // Degrees per step for smooth movement

// Command buffer
String inputString = "";
boolean stringComplete = false;

void setup() {
  Serial.begin(9600);

  gripperLeft.attach(9);
  gripperRight.attach(10);

  // Start at half-open
  gripperLeft.write(90);
  gripperRight.write(90);
  
  Serial.println("Gripper Ready!");
  Serial.println("Commands:");
  Serial.println("  GESTURE:open_hand - Open gripper");
  Serial.println("  GESTURE:closed_fist - Close gripper");
  Serial.println("  GESTURE:pinch - Half position");
  Serial.println("  OPEN - Open gripper");
  Serial.println("  CLOSE - Close gripper");
  Serial.println("  HALF - Half position");
  Serial.println("  o/c/h - Single char commands (original)");
  
  inputString.reserve(200);
}

void loop() {
  // Check for serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Also handle single character commands (your original method)
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    handleSingleChar(cmd);
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // Handle single character commands immediately
    if (inChar == 'o' || inChar == 'c' || inChar == 'h') {
      handleSingleChar(inChar);
      return;
    }
    
    // Build string for multi-character commands
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void handleSingleChar(char cmd) {
  if (cmd == 'o') {           // open gripper
    openGripper();
  } 
  else if (cmd == 'c') {      // close gripper
    closeGripper();
  } 
  else if (cmd == 'h') {      // half-open
    halfGripper();
  }
}

void processCommand(String command) {
  command.trim();
  
  if (command.startsWith("GESTURE:")) {
    // Handle gesture command from Python
    String gesture = command.substring(8);
    handleGesture(gesture);
    
  } else if (command == "OPEN") {
    openGripper();
    
  } else if (command == "CLOSE") {
    closeGripper();
    
  } else if (command == "HALF") {
    halfGripper();
    
  } else if (command == "STATUS") {
    printStatus();
    
  } else {
    Serial.println("Unknown command: " + command);
    Serial.println("Use: OPEN, CLOSE, HALF, or GESTURE:gesture_name");
  }
}

void handleGesture(String gesture) {
  Serial.println("Processing gesture: " + gesture);
  
  if (gesture == "open_hand") {
    openGripper();
    Serial.println("Gesture: Open hand - gripper opened");
    
  } else if (gesture == "closed_fist") {
    closeGripper();
    Serial.println("Gesture: Closed fist - gripper closed");
    
  } else if (gesture == "pinch") {
    halfGripper();
    Serial.println("Gesture: Pinch - gripper half position");
    
  } else if (gesture == "pointing" || gesture == "peace_sign" || gesture == "thumbs_up") {
    // For other gestures, just acknowledge but don't move
    Serial.println("Gesture: " + gesture + " - acknowledged (no gripper action)");
    
  } else if (gesture.startsWith("swipe_")) {
    // Swipe gestures could control gripper speed or do fun movements
    Serial.println("Gesture: " + gesture + " - acknowledged (no gripper action)");
    
  } else {
    Serial.println("Gesture: " + gesture + " - not mapped to gripper action");
  }
}

void openGripper() {
  if (gripperState == 2 && !moving) {
    Serial.println("Gripper already open");
    return;
  }
  
  Serial.println("Opening gripper");
  smoothMove(0, 180); // Left servo to 0, Right servo to 180 (inverted)
  gripperState = 2;
}

void closeGripper() {
  if (gripperState == 0 && !moving) {
    Serial.println("Gripper already closed");
    return;
  }
  
  Serial.println("Closing gripper");
  smoothMove(180, 0); // Left servo to 180, Right servo to 0 (inverted)
  gripperState = 0;
}

void halfGripper() {
  if (gripperState == 1 && !moving) {
    Serial.println("Gripper already at half position");
    return;
  }
  
  Serial.println("Half gripper");
  smoothMove(90, 90); // Both servos to 90 degrees
  gripperState = 1;
}

void smoothMove(int targetLeft, int targetRight) {
  moving = true;
  
  int currentLeft = gripperLeft.read();
  int currentRight = gripperRight.read();
  
  // Calculate the maximum steps needed
  int stepsLeft = abs(targetLeft - currentLeft);
  int stepsRight = abs(targetRight - currentRight);
  int maxSteps = max(stepsLeft, stepsRight) / STEP_SIZE + 1;
  
  // Smooth movement
  for (int step = 0; step <= maxSteps; step++) {
    // Calculate intermediate positions
    int newLeft = currentLeft + (targetLeft - currentLeft) * step / maxSteps;
    int newRight = currentRight + (targetRight - currentRight) * step / maxSteps;
    
    // Move servos to intermediate positions
    gripperLeft.write(newLeft);
    gripperRight.write(newRight);
    
    delay(MOVE_DELAY);
  }
  
  // Ensure final positions are exact
  gripperLeft.write(targetLeft);
  gripperRight.write(targetRight);
  
  moving = false;
}

void printStatus() {
  String state;
  switch(gripperState) {
    case 0: state = "CLOSED"; break;
    case 1: state = "HALF"; break;
    case 2: state = "OPEN"; break;
  }
  
  Serial.println("Gripper Status: " + state);
  Serial.println("Left servo: " + String(gripperLeft.read()) + "°");
  Serial.println("Right servo: " + String(gripperRight.read()) + "°");
  Serial.println("Moving: " + String(moving ? "YES" : "NO"));
}