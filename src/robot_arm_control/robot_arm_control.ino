#include <Servo.h>
#include <Stepper.h>

// ---- Servo setup ----
const int NUM_SERVOS = 4;
const int servoPins[NUM_SERVOS] = {5, 6, 9, 10};
Servo servos[NUM_SERVOS];
int positions[NUM_SERVOS];

// ---- Incrementing system (servos) ----
bool servoIncrementing[NUM_SERVOS] = {false};
int servoIncrementSpeed[NUM_SERVOS] = {0};
unsigned long lastUpdateTime[NUM_SERVOS] = {0};

// ---- Stepper setup ----
const int stepsPerRevolution = 200;

// Give the motor control pins names:
#define pwmA 3
#define pwmB 11
#define brakeA 9
#define brakeB 8
#define dirA 12
#define dirB 13
#define stepperSpeed 60

// Initialize the stepper library:
Stepper myStepper = Stepper(stepsPerRevolution, dirA, dirB);

// ---- Stepper auto-running system ----
bool stepperRunning = false;
int stepperRunSpeed = 0;          // steps per update
unsigned long lastStepperUpdate = 0;

void setup() {
  Serial.begin(115200);

  // Attach servos and initialize to 90 degrees
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    positions[i] = 90;
    servos[i].write(positions[i]);
  }

  // Initialize stepper
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);

  digitalWrite(pwmA, HIGH);
  digitalWrite(pwmB, HIGH);
  digitalWrite(brakeA, LOW);
  digitalWrite(brakeB, LOW);

  myStepper.setSpeed(stepperSpeed);

  Serial.println("Arduino ready - expecting packets:");
  Serial.println("<id angle>");
  Serial.println("STEP <steps>");
  Serial.println("STARTINC <id> <speed>");
  Serial.println("STOPINC <id>");
  Serial.println("STARTSTEP <speed>");
  Serial.println("STOPSTEP");
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

  unsigned long now = millis();

  // --- handle servo auto-increment ---
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servoIncrementing[i]) {
      if (now - lastUpdateTime[i] >= 50) {
        lastUpdateTime[i] = now;

        positions[i] += servoIncrementSpeed[i];
        positions[i] = constrain(positions[i], 0, 180);
        servos[i].write(positions[i]);
      }
    }
  }

  // --- handle stepper auto-run ---
  if (stepperRunning) {
    if (now - lastStepperUpdate >= 20) { // update every 20 ms
      lastStepperUpdate = now;
      myStepper.step(stepperRunSpeed);
    }
  }
}

void processPacket(String packet) {
  packet.trim();
  if (packet.length() == 0) return;

  Serial.print("Received packet: ");
  Serial.println(packet);

  // ---- STARTSTEP command ----
  if (packet.startsWith("STARTSTEP")) {
    int speed;
    int count = sscanf(packet.c_str(), "STARTSTEP %d", &speed);

    if (count != 1) {
      Serial.println("Error: Use STARTSTEP <speed>");
      return;
    }

    stepperRunning = true;
    stepperRunSpeed = speed;

    Serial.print("Stepper auto-run started with speed ");
    Serial.println(speed);
    return;
  }

  // ---- STOPSTEP command ----
  if (packet.startsWith("STOPSTEP")) {
    stepperRunning = false;
    Serial.println("Stepper auto-run stopped");
    return;
  }

  // ---- STARTINC command ----
  if (packet.startsWith("STARTINC")) {
    int id, speed;
    int count = sscanf(packet.c_str(), "STARTINC %d %d", &id, &speed);

    if (count != 2) {
      Serial.println("Error: Use STARTINC <id> <speed>");
      return;
    }

    if (id < 0 || id >= NUM_SERVOS) {
      Serial.println("Error: Servo ID out of range");
      return;
    }

    servoIncrementing[id] = true;
    servoIncrementSpeed[id] = speed;

    Serial.print("Increment started on servo ");
    Serial.print(id);
    Serial.print(" with speed ");
    Serial.println(speed);
    return;
  }

  // ---- STOPINC command ----
  if (packet.startsWith("STOPINC")) {
    int id;
    int count = sscanf(packet.c_str(), "STOPINC %d", &id);

    if (count != 1) {
      Serial.println("Error: Use STOPINC <id>");
      return;
    }

    if (id < 0 || id >= NUM_SERVOS) {
      Serial.println("Error: Servo ID out of range");
      return;
    }

    servoIncrementing[id] = false;

    Serial.print("Increment stopped on servo ");
    Serial.println(id);
    return;
  }

  // ---- STEP command ----
  if (packet.startsWith("STEP")) {
    int steps;
    int count = sscanf(packet.c_str(), "STEP %d", &steps);

    if (count != 1) {
      Serial.println("Error: STEP command must be 'STEP <steps>'");
      return;
    }

    myStepper.step(steps);
    Serial.print("Stepper moved by steps: ");
    Serial.println(steps);
    return;
  }

  // ---- Servo positioning command ----
  int id, angle;
  int count = sscanf(packet.c_str(), "%d %d", &id, &angle);

  if (count != 2) {
    Serial.println("Error: Packet must be '<id> <angle>'");
    return;
  }

  if (id < 0 || id >= NUM_SERVOS) {
    Serial.println("Error: Servo ID out of range");
    return;
  }

  angle = constrain(angle, 0, 180);
  
  servos[id].write(angle);
  positions[id] = angle;

  Serial.print("Servo ");
  Serial.print(id);
  Serial.print(" set to: ");
  Serial.println(angle);
}
