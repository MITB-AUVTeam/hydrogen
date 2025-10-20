#include <Arduino.h>

const int escPin = 14; // ESC signal pin

// Function prototype
void sendPulse(int microseconds);

void setup() {
  pinMode(escPin, OUTPUT);
  pinMode(18, OUTPUT);

  // Initialize ESC to minimum throttle
  sendPulse(1000);
  delay(3000); // wait for ESC to arm
}

void loop() {
  digitalWrite(18, HIGH);
  for (int pulse = 1000; pulse <= 1650; pulse += 50) {
    sendPulse(pulse);
    delay(100); // update fast for OneShot125
  }

  for (int i = 0; i <= 100; i++) {
    sendPulse(1650);
    delay(100); // update fast for OneShot125
  }

  // Ramp throttle down
  for (int pulse = 1650; pulse >= 1000; pulse -= 50) {
    sendPulse(pulse);
    delay(100);
  }
  digitalWrite(18, LOW);

  delay(2000);
}

// Function to send a single pulse to the ESC
void sendPulse(int microseconds) {
  digitalWrite(escPin, HIGH);
  delayMicroseconds(microseconds);
  digitalWrite(escPin, LOW);
  delayMicroseconds(20000 - microseconds); // 20ms cycle
}
