#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial Serial2(2);

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
}

void loop() {
  Serial2.println("Hello from ESP32 Sender!");
  delay(1000);
}
