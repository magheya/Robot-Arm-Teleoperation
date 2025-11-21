#include <Servo.h>

Servo servo;
int pos = 90;

void setup() {
  Serial.begin(115200);
  servo.attach(9);
  servo.write(pos);
  Serial.println("Arduino ready");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    Serial.print("Received: ");
    Serial.println(cmd);

    if (cmd == 'L') pos -= 5;
    if (cmd == 'R') pos += 5;

    pos = constrain(pos, 0, 180);
    servo.write(pos);
  }
}