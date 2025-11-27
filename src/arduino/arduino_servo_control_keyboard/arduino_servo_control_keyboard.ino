#include <Servo.h>

const int NUM_SERVOS = 6;
const int servoPins[NUM_SERVOS] = {3, 5, 6, 9, 10, 11}; 
Servo servos[NUM_SERVOS];
int positions[NUM_SERVOS];  

void setup() {
  Serial.begin(115200);

  // Attach servos and initialize to 90 degrees
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    positions[i] = 90;
    servos[i].write(positions[i]);
  }

  Serial.println("Arduino ready - expecting packets: <id angle> (id 0-5)");
}

void loop() {
  static String input = "";

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      processPacket(input);
      input = ""; // clear buffer
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