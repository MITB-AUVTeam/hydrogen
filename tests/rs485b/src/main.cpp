#include <Arduino.h>
#include <HardwareSerial.h>

#define TXD2 17
#define RXD2 16
#define DE_RE 4  // GPIO controlling DE & RE

HardwareSerial RS485Serial(2);

void setup() {
  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW); // Always receive
  RS485Serial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.begin(115200);
}

void loop() {
  if (RS485Serial.available() >= sizeof(int)) {
    int receivedNumber = 0;
    RS485Serial.readBytes((char*)&receivedNumber, sizeof(receivedNumber));
    Serial.println("Received: " + String(receivedNumber));
  }
}
