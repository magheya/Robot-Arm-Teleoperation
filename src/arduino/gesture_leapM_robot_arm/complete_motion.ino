#include <Servo.h>
#include <Stepper.h> // Or AccelStepper (recommended for smoother movement)

// --- CONFIGURATION ---
Servo gripper;
const int stepsPerRevolution = 200; 
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11); // Adjust pins

// Variables to track state
char stepperState = 'X'; // 'X' = Stop, 'L' = Left, 'R' = Right
unsigned long lastStepTime = 0;
int stepSpeed = 10; // Speed delay in ms (Lower is faster)

void setup() {
  Serial.begin(9600);
  gripper.attach(6); // Gripper Servo Pin
  gripper.write(90); // Initial pos
}

void loop() {
  // 1. READ COMMANDS
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Gripper Commands (Immediate Action)
    if (cmd == 'O') {
      gripper.write(180); // Open
    } 
    else if (cmd == 'C') {
      gripper.write(45);  // Close
    }
    
    // Stepper Commands (Update State Only)
    else if (cmd == 'L') {
      stepperState = 'L';
    } 
    else if (cmd == 'R') {
      stepperState = 'R';
    } 
    else if (cmd == 'X') {
      stepperState = 'X';
    }
  }

  // 2. EXECUTE MOTION (Non-blocking)
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastStepTime >= stepSpeed) {
    lastStepTime = currentMillis;
    
    if (stepperState == 'L') {
      myStepper.step(-1); // Move 1 step left
    } 
    else if (stepperState == 'R') {
      myStepper.step(1);  // Move 1 step right
    }
    // If state is 'X', do nothing
  }
}