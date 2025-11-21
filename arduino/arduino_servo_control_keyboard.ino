#include <Servo.h>

Servo servo;
int pos = 90;

void setup() {
  Serial.begin(115200);
  servo.attach(9);
  servo.write(pos);
  Serial.println("Arduino ready - Servo attached to pin 9");
  Serial.print("Initial position: ");
  Serial.println(pos);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    Serial.print("Received command: ");
    Serial.println(cmd);

    if (cmd == 'L') {
      pos -= 5;
      Serial.print("Moving LEFT to position: ");
    }
    if (cmd == 'R') {
      pos += 5;
      Serial.print("Moving RIGHT to position: ");
    }

    pos = constrain(pos, 0, 180);
    Serial.println(pos);
    
    servo.write(pos);
    Serial.println("Servo command sent");
    
    // Add a small delay to see the movement
    delay(100);
  }
}