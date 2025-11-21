#include <Servo.h>

Servo myServo;

void setup() {
  Serial.begin(115200);
  Serial.println("=== SERVO HARDWARE TEST ===");
  
  // Attach servo to pin 9
  myServo.attach(9);
  Serial.println("Servo attached to pin 9");
  
  // Test if servo is connected by doing a sweep
  Serial.println("Testing servo movement...");
  Serial.println("Servo should move from 0° to 180° and back");
  
  // Sweep from 0 to 180 degrees
  for(int pos = 0; pos <= 180; pos += 30) {
    myServo.write(pos);
    Serial.print("Moving to position: ");
    Serial.print(pos);
    Serial.println("°");
    delay(1000); // Wait 1 second between moves
  }
  
  // Sweep back from 180 to 0 degrees
  for(int pos = 180; pos >= 0; pos -= 30) {
    myServo.write(pos);
    Serial.print("Moving to position: ");
    Serial.print(pos);
    Serial.println("°");
    delay(1000);
  }
  
  // Return to center
  myServo.write(90);
  Serial.println("Returned to center (90°)");
  Serial.println("=== TEST COMPLETE ===");
  Serial.println("Did you see the servo move?");
  Serial.println("If NO movement:");
  Serial.println("1. Check servo connections");
  Serial.println("2. Check servo power supply");
  Serial.println("3. Try a different servo");
}

void loop() {
  // Empty - test runs once in setup
}