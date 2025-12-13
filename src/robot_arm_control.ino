#include <Servo.h>
#include <Stepper.h>

// --- STEPPER CONFIG ---
const int STEPS_PER_REV = 200;
Stepper myStepper(STEPS_PER_REV, 12, 13);

// --- SERVO CONFIG ---
Servo shoulder;
Servo elbow;
Servo wrist;
Servo gripL;
Servo gripR;

const int SHOULDER_PIN = 10; 
const int ELBOW_PIN = 6;
const int WRIST_PIN = 5;
const int GRIP_L_PIN = A4; 
const int GRIP_R_PIN = A5; 

// Variables for Stepper State
char moveState = 'S'; 

void setup() {
  Serial.begin(115200);
  Serial.println("--- MANUAL INDEPENDENT CONTROL ---");
  Serial.println("  Shoulder -> LOCKED 150 (Upright)");
  Serial.println("  e <angle> -> Move Elbow ONLY");
  Serial.println("  w <angle> -> Move Wrist ONLY");
  Serial.println("  l <angle> -> Left Gripper");
  Serial.println("  r <angle> -> Right Gripper");
  Serial.println("  F/B/S    -> Stepper Control");

  // POWER SETUP
  pinMode(3, OUTPUT); digitalWrite(3, HIGH);
  pinMode(11, OUTPUT); digitalWrite(11, HIGH);
  pinMode(9, OUTPUT); digitalWrite(9, LOW);
  pinMode(8, OUTPUT); digitalWrite(8, LOW);

  myStepper.setSpeed(5); // Slow speed for Torque
  
  // 1. SHOULDER (Fixed)
  shoulder.attach(SHOULDER_PIN);
  shoulder.write(150); 
  
  // 2. ELBOW (Start Bent)
  elbow.attach(ELBOW_PIN);
  elbow.write(120);
  
  // 3. WRIST (Start Tucked)
  // This will stay at 180 until YOU change it. 
  // Moving the elbow will NOT change this value.
  wrist.attach(WRIST_PIN);
  wrist.write(180);    
  
  // 4. GRIPPERS
  gripL.attach(GRIP_L_PIN); gripL.write(90);
  gripR.attach(GRIP_R_PIN); gripR.write(90);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // --- ELBOW COMMAND ---
    if (cmd == 'e') {
      int angle = Serial.parseInt(); 
      Serial.print("Elbow set to: "); Serial.println(angle);
      elbow.write(angle);
    }
    
    // --- WRIST COMMAND ---
    else if (cmd == 'w') {
      int angle = Serial.parseInt();
      Serial.print("Wrist set to: "); Serial.println(angle);
      wrist.write(angle);
    }
    
    // --- GRIPPERS ---
    else if (cmd == 'l') gripL.write(Serial.parseInt());
    else if (cmd == 'r') gripR.write(Serial.parseInt());
    
    // --- STEPPER ---
    else if (cmd == 'F' || cmd == 'B' || cmd == 'S') {
      moveState = cmd;
    }
    
    while(Serial.available() > 0 && isSpace(Serial.peek())) Serial.read();
  }

  if (moveState == 'F') myStepper.step(1); 
  else if (moveState == 'B') myStepper.step(-1); 
}