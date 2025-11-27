#include <Servo.h>

// Dual gripper servo objects
Servo leftGripperServo;   // Left gripper servo
Servo rightGripperServo;  // Right gripper servo
Servo rotationServo;      // HS-485HB rotation servo (continuous rotation)

// Servo pins
const int LEFT_GRIPPER_PIN = 9;
const int RIGHT_GRIPPER_PIN = 10;
const int ROTATION_SERVO_PIN = 11;  // HS-485HB servo pin

// Gripper positions (adjust these based on your gripper orientation)
const int LEFT_GRIPPER_OPEN = 0;      // Left gripper open position
const int LEFT_GRIPPER_CLOSED = 180;  // Left gripper closed position
const int RIGHT_GRIPPER_OPEN = 180;   // Right gripper open position (may be opposite)
const int RIGHT_GRIPPER_CLOSED = 0;   // Right gripper closed position (may be opposite)

// HS-485HB continuous rotation servo values
const int ROTATION_STOP = 90;          // Stop position
const int ROTATION_LEFT = 70;          // Rotate left (adjust speed by changing value)
const int ROTATION_RIGHT = 110;        // Rotate right (adjust speed by changing value)
const int ROTATION_DURATION = 500;     // Duration for rotation commands (ms)

// Movement parameters
const int GRIPPER_SPEED = 3;           // Degrees per step for smooth movement
const int MOVE_DELAY = 20;             // ms delay between steps
const int COMMAND_DELAY = 100;         // ms delay after completing a command

// Current positions
int currentLeftPos = 90;   // Start at middle position
int currentRightPos = 90;  // Start at middle position

void setup() {
  Serial.begin(9600);
  
  // Attach all servos
  leftGripperServo.attach(LEFT_GRIPPER_PIN);
  rightGripperServo.attach(RIGHT_GRIPPER_PIN);
  rotationServo.attach(ROTATION_SERVO_PIN);
  
  // Initialize to starting positions
  leftGripperServo.write(currentLeftPos);
  rightGripperServo.write(currentRightPos);
  rotationServo.write(ROTATION_STOP);  // Stop rotation servo
  
  delay(1000);
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("🤖 Robot Arm Control Ready!");
  Serial.println("Commands:");
  Serial.println("  'O' = Open both grippers");
  Serial.println("  'C' = Close both grippers");
  Serial.println("  'L' = Rotate left");
  Serial.println("  'R' = Rotate right");
  Serial.println("  'S' = Status");
  Serial.println("  'H' = Home position");
  
  // Signal ready with LED blinks
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
  
  Serial.println(" Robot arm system initialized and ready!");
  printStatus();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    // Clear any extra characters
    while(Serial.available()) {
      Serial.read();
    }
    
    Serial.print("Received command: '");
    Serial.print(command);
    Serial.println("'");
    
    switch(command) {
      case 'O':
      case 'o':
        openBothGrippers();
        break;
        
      case 'C':
      case 'c':
        closeBothGrippers();
        break;
        
      case 'L':
      case 'l':
        rotateLeft();
        break;
        
      case 'R':
      case 'r':
        rotateRight();
        break;
        
      case 'S':
      case 's':
        printStatus();
        break;
        
      case 'H':
      case 'h':
        goHome();
        break;
        
      default:
        Serial.print("Unknown command: '");
        Serial.print(command);
        Serial.println("' (Use O, C, L, R, S, or H)");
        break;
    }
    
    delay(COMMAND_DELAY);
  }
}

void openBothGrippers() {
  Serial.println(" Opening both grippers...");
  
  // Flash LED once for open command
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Move both grippers simultaneously to open position
  moveBothGrippers(LEFT_GRIPPER_OPEN, RIGHT_GRIPPER_OPEN);
  
  currentLeftPos = LEFT_GRIPPER_OPEN;
  currentRightPos = RIGHT_GRIPPER_OPEN;
  
  Serial.println(" Both grippers opened!");
  printGripperStatus();
}

void closeBothGrippers() {
  Serial.println(" Closing both grippers...");
  
  // Flash LED twice for close command
  for(int i = 0; i < 2; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  
  // Move both grippers simultaneously to closed position
  moveBothGrippers(LEFT_GRIPPER_CLOSED, RIGHT_GRIPPER_CLOSED);
  
  currentLeftPos = LEFT_GRIPPER_CLOSED;
  currentRightPos = RIGHT_GRIPPER_CLOSED;
  
  Serial.println(" Both grippers closed!");
  printGripperStatus();
}

void rotateLeft() {
  Serial.println(" Rotating left...");
  
  // Flash LED pattern for left rotation
  for(int i = 0; i < 2; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }
  
  // Start rotating left
  rotationServo.write(ROTATION_LEFT);
  delay(ROTATION_DURATION);
  
  // Stop rotation
  rotationServo.write(ROTATION_STOP);
  
  Serial.println(" Left rotation complete!");
}

void rotateRight() {
  Serial.println(" Rotating right...");
  
  // Flash LED pattern for right rotation  
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }
  
  // Start rotating right
  rotationServo.write(ROTATION_RIGHT);
  delay(ROTATION_DURATION);
  
  // Stop rotation
  rotationServo.write(ROTATION_STOP);
  
  Serial.println(" Right rotation complete!");
}

void goHome() {
  Serial.println(" Moving to home position...");
  
  // Flash LED multiple times
  for(int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
  
  // Stop rotation servo
  rotationServo.write(ROTATION_STOP);
  
  // Move grippers to center position
  moveBothGrippers(90, 90);
  
  currentLeftPos = 90;
  currentRightPos = 90;
  
  Serial.println(" Home position reached!");
  printStatus();
}

void moveBothGrippers(int leftTarget, int rightTarget) {
  // Calculate steps needed for each gripper
  int leftStart = currentLeftPos;
  int rightStart = currentRightPos;
  
  int leftDistance = abs(leftTarget - leftStart);
  int rightDistance = abs(rightTarget - rightStart);
  
  // Use the maximum distance to ensure both grippers finish at the same time
  int maxSteps = max(leftDistance, rightDistance) / GRIPPER_SPEED;
  if (maxSteps == 0) maxSteps = 1;
  
  // Calculate step sizes for each gripper
  float leftStepSize = (float)(leftTarget - leftStart) / maxSteps;
  float rightStepSize = (float)(rightTarget - rightStart) / maxSteps;
  
  // Move both grippers simultaneously
  for(int step = 0; step <= maxSteps; step++) {
    int leftPos = leftStart + (leftStepSize * step);
    int rightPos = rightStart + (rightStepSize * step);
    
    // Constrain positions to servo limits
    leftPos = constrain(leftPos, 0, 180);
    rightPos = constrain(rightPos, 0, 180);
    
    // Move both servos
    leftGripperServo.write(leftPos);
    rightGripperServo.write(rightPos);
    
    delay(MOVE_DELAY);
  }
  
  // Ensure final positions are exact
  leftGripperServo.write(leftTarget);
  rightGripperServo.write(rightTarget);
}

void printGripperStatus() {
  Serial.println("   Gripper Status:");
  Serial.print("     Left:  ");
  Serial.print(currentLeftPos);
  Serial.print("° (");
  printPositionStatus(currentLeftPos, LEFT_GRIPPER_OPEN, LEFT_GRIPPER_CLOSED);
  Serial.println(")");
  
  Serial.print("     Right: ");
  Serial.print(currentRightPos);
  Serial.print("° (");
  printPositionStatus(currentRightPos, RIGHT_GRIPPER_OPEN, RIGHT_GRIPPER_CLOSED);
  Serial.println(")");
}

void printPositionStatus(int currentPos, int openPos, int closedPos) {
  if (abs(currentPos - openPos) < 10) {
    Serial.print("OPEN");
  } else if (abs(currentPos - closedPos) < 10) {
    Serial.print("CLOSED");
  } else {
    Serial.print("PARTIAL");
  }
}

void printStatus() {
  Serial.println(" Robot Arm Status:");
  Serial.print("   Left Gripper:  ");
  Serial.print(currentLeftPos);
  Serial.println("°");
  
  Serial.print("   Right Gripper: ");
  Serial.print(currentRightPos);
  Serial.println("°");
  
  Serial.println("   Rotation Servo: HS-485HB (continuous rotation)");
  Serial.println("---");
}