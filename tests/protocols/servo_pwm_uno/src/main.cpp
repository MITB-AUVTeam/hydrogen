#include <Arduino.h>
#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;


void setup() {
  servo1.attach(9);
  servo2.attach(6);
  // servo3.attach(5);

  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  // servo3.writeMicroseconds(1500);

  delay(1000);
}

void loop() {
  for (int i = 0;i < 100;i++) {
    servo1.writeMicroseconds(1500 + i);
    servo2.writeMicroseconds(1500 + i);
    // servo3.writeMicroseconds(1500 + i);

    delay(100);
  }
  for (int i = 0;i < 200;i++) {
    servo1.writeMicroseconds(1600 - i);
    servo2.writeMicroseconds(1600 - i);
    // servo3.writeMicroseconds(1600 - i);
    delay(100);
  }
  for (int i = 0;i < 100;i++) {
    servo1.writeMicroseconds(1400 + i);
    servo2.writeMicroseconds(1400 + i);
    // servo3.writeMicroseconds(1400 + i);
    delay(100);
  }
}