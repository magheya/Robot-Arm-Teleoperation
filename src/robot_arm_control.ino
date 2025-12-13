#include <Servo.h>
#include <Stepper.h>

// =============================================================
// CONFIGURATION
// =============================================================

// ---- 1. STEPPER SETUP (Base Rotation) ----
const int STEPS_PER_REV = 200; // Standard NEMA 17 is 200 steps/rev

// Motor Shield Pins for Stepper (Uses Channel A + Channel B)
#define DIR_A   12
#define PWM_A   3
#define BRAKE_A 9
#define DIR_B   13
#define PWM_B   11
#define BRAKE_B 8

Stepper baseStepper(STEPS_PER_REV, DIR_A, DIR_B);

// ---- 2. SERVO SETUP (Arm Joints) ----
const int NUM_SERVOS = 5;

// Pin Mapping based on your list:
// ID 0: Shoulder (Swinging Base) -> Pin 5
// ID 1: Elbow                    -> Pin 10
// ID 2: Wrist                    -> Pin 6
// ID 3: Gripper Left             -> Pin A4
// ID 4: Gripper Right            -> Pin A5
const int servoPins[NUM_SERVOS] = {10, 6, A4, A5};

Servo servos[NUM_SERVOS];
int servoPositions[NUM_SERVOS];

// ---- Auto-Run Variables (for Stepper) ----
bool stepperMoving = false;
int stepperSpeed = 0;
unsigned long lastStepperTime = 0;

void setup() {
  Serial.begin(115200);

  // --- Initialize Stepper Motor Driver ---
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(BRAKE_A, OUTPUT);
  pinMode(BRAKE_B, OUTPUT);
  digitalWrite(PWM_A, HIGH);
  digitalWrite(PWM_B, HIGH);
  digitalWrite(BRAKE_A, LOW);
  digitalWrite(BRAKE_B, LOW);
  baseStepper.setSpeed(60);

  // --- Initialize Servos ---
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    servoPositions[i] = 90;
    servos[i].write(90);
  }

  Serial.println("ROBOT ARM READY");
  Serial.println("  BASE:     Stepper (A/B terminals)");
  Serial.println("  SHOULDER: Servo ID 0 (Pin 5)");
  Serial.println("  ELBOW:    Servo ID 1 (Pin 10)");
  Serial.println("  WRIST:    Servo ID 2 (Pin 6)");
  Serial.println("  GRIPPERS: Servo ID 3/4 (Pin A4/A5)");
  Serial.println("");
  Serial.println("COMMANDS:");
  Serial.println("  STEP <steps>       -> Move Base manually");
  Serial.println("  STARTSTEP <speed>  -> Spin Base continuously");
  Serial.println("  STOPSTEP           -> Stop Base");
  Serial.println("  <id> <angle>       -> Move Servo (e.g., '0 45')");
}

void loop() {
  static String input = "";
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      processPacket(input);
      input = "";
    } else {
      input += c;
    }
  }

  // Handle Continuous Stepper Rotation
  if (stepperMoving) {
    unsigned long now = millis();
    if (now - lastStepperTime >= 10) { 
      lastStepperTime = now;
      baseStepper.step(stepperSpeed);
    }
  }
}

void processPacket(String packet) {
  packet.trim();
  if (packet.length() == 0) return;

  // ---- STEPPER COMMANDS ----
  if (packet.startsWith("STEP")) {
    int steps;
    if (sscanf(packet.c_str(), "STEP %d", &steps) == 1) {
      baseStepper.step(steps);
      Serial.println("Stepped.");
    }
    return;
  }
  if (packet.startsWith("STARTSTEP")) {
    int speed;
    if (sscanf(packet.c_str(), "STARTSTEP %d", &speed) == 1) {
      stepperMoving = true;
      stepperSpeed = speed; 
      Serial.println("Stepper Auto-Run ON");
    }
    return;
  }
  if (packet.startsWith("STOPSTEP")) {
    stepperMoving = false;
    Serial.println("Stepper Auto-Run OFF");
    return;
  }

  // ---- SERVO DIRECT ANGLE COMMANDS ----
  int id, angle;
  if (sscanf(packet.c_str(), "%d %d", &id, &angle) == 2) {
    if (id >= 0 && id < NUM_SERVOS) {
      angle = constrain(angle, 0, 180);
      servos[id].write(angle);
      servoPositions[id] = angle;
      Serial.print("Servo "); Serial.print(id);
      Serial.print(" -> "); Serial.println(angle);
    } else {
      Serial.println("Error: Servo ID must be 0-4");
    }
  }
}