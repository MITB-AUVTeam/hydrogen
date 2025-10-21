#include <Arduino.h>
#include <Servo.h>

Servo servo;

void setup() {
  servo.attach(9);
  servo.writeMicroseconds(1500);
}

void loop() {
  for (int i = 0;i < 150;i++) {
    servo.writeMicroseconds(1500 + i);
    delay(100);
  }
  for (int i = 0;i < 150;i++) {
    servo.writeMicroseconds(1650 - i);
    delay(100);
  }

}