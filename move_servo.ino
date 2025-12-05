// Arduino sketch: head_servo_receiver.ino
#include <Servo.h>

Servo servoYaw;
Servo servoPitch;

const int yawPin = 9;
const int pitchPin = 10;

String inputLine = "";

void setup() {
  Serial.begin(115200);
  servoYaw.attach(yawPin);
  servoPitch.attach(pitchPin);
  servoYaw.write(90);
  servoPitch.write(90);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processLine(inputLine);
      inputLine = "";
    } else {
      inputLine += c;
    }
  }
}

void processLine(String line) {
  // expected format: YdddPddd
  int yIndex = line.indexOf('Y');
  int pIndex = line.indexOf('P');
  if (yIndex >= 0 && pIndex >= 0) {
    String yStr = line.substring(yIndex + 1, pIndex);
    String pStr = line.substring(pIndex + 1);
    int y = yStr.toInt();
    int p = pStr.toInt();
    y = constrain(y, 0, 180);
    p = constrain(p, 0, 180);
    servoYaw.write(y);
    servoPitch.write(p);
  }
}
