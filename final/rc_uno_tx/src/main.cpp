#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(255, 10);

struct PotValues {
  int pot1;
  int pot2;
  int pot3;
};

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  PotValues data;

  data.pot1 = analogRead(A5);
  data.pot2 = analogRead(A4);
  data.pot3 = analogRead(A3);

  mySerial.write((uint8_t*)&data, sizeof(data));

  delay(200);
}
