#include <Arduino.h>
#include <ESP32Servo.h>

Servo myServo;  // Use Servo class

void setup() {
  myServo.attach(12);
  myServo.writeMicroseconds(1500); // Center position
  delay(1000);              // Give servo time to reach center
}

void loop() {
  // Sweep from 1500 to 1650
  for (int i = 0; i <= 150; i++) {
    myServo.writeMicroseconds(1500 + i);
    delay(20); // Smaller delay for smoother movement
  }

  // Sweep back from 1650 to 1500
  for (int i = 0; i <= 150; i++) {
    myServo.writeMicroseconds(1650 - i);
    delay(20);
  }
}
