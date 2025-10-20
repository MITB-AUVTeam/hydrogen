#include <Arduino.h>
#include <HardwareSerial.h>

#define TX_PIN 17
#define RX_PIN 16
#define DE_RE 4

HardwareSerial RS485Serial(1);

void setup() {
  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW); // Enable receive mode

  RS485Serial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.begin(9600);
}

void loop() {
  if (RS485Serial.available()) {
    int received = RS485Serial.parseInt();
    Serial.print("Received: ");
    Serial.println(received);
  }
}
