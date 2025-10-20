#include <Arduino.h>
#include <HardwareSerial.h>

#define TX_PIN 17
#define RX_PIN 16
#define DE_RE 4

HardwareSerial RS485Serial(1);

int value = 0;

void setup() {
  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, HIGH); // Enable transmit mode

  RS485Serial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.begin(9600);
}

void loop() {
  RS485Serial.println(value);  // Send integer
  Serial.print("Sent: ");
  Serial.println(value);

  value++; // Increment number
  if (value > 100) value = 0;  // Loop back
  delay(500);
}
