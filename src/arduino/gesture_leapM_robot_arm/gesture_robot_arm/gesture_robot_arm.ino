#include <Servo.h>
#include <Stepper.h>

// ---- Servo setup ----
const int NUM_SERVOS = 4;
const int servoPins[NUM_SERVOS] = {5, 6, 9, 10};
Servo servos[NUM_SERVOS];
int positions[NUM_SERVOS];

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

// Initialize the stepper library on the motor shield:
Stepper myStepper = Stepper(stepsPerRevolution, dirA, dirB);

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

  Serial.println("Arduino ready - expecting packets: <id angle> (id 0-5) or STEP <steps>");
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
}

void processPacket(String packet) {
  packet.trim();
  if (packet.length() == 0) return;

  Serial.print("Received packet: ");
  Serial.println(packet);

  // Stepper command
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

  // Servo command
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

  angle = constrain(angle, 0, 360);
  servos[id].write(angle);
  positions[id] = angle;

  Serial.print("Servo ");
  Serial.print(id);
  Serial.print(" set to: ");
  Serial.println(angle);
}
