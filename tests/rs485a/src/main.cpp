#include <Arduino.h>
#include <HardwareSerial.h>

#define TXD2 17
#define RXD2 16
#define DE_RE 4  // GPIO controlling DE & RE

HardwareSerial RS485Serial(2);

void setup() {
  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW); // Start in receive mode
  RS485Serial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.begin(115200);
}

void loop() {
  int numberToSend = 1234;

  // Enable transmit
  digitalWrite(DE_RE, HIGH);
  RS485Serial.write((uint8_t*)&numberToSend, sizeof(numberToSend));
  RS485Serial.flush();        // Wait until all bytes are sent
  digitalWrite(DE_RE, LOW);   // Back to receive

  Serial.println("Sent: " + String(numberToSend));
  delay(1000);
}
