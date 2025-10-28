// #include <Arduino.h>
// #include <SoftwareSerial.h>

// SoftwareSerial mySerial(11, 10);

// struct PotValues {
//   int pot1;
//   int pot2;
//   int pot3;
// };

// void setup() {
//   mySerial.begin(9600);
//   Serial.begin(9600);
// }

// void loop() {
//   PotValues data;

//   data.pot1 = analogRead(A5) * (136.0 / 1023.0) + 1500;
//   data.pot2 = analogRead(A4) * (136.0 / 1023.0) + 1500;
//   data.pot3 = analogRead(A3) * (136.0 / 1023.0) + 1500;

//   //mySerial.write((uint8_t*)&data, sizeof(data));

//   byte header[2] = {0xAA, 0x55};  // Start bytes
//   mySerial.write(header, 2);
//   mySerial.write((uint8_t*)&data, sizeof(data));
//   byte checksum = (data.pot1 + data.pot2 + data.pot3) & 0xFF;
//   mySerial.write(checksum);


//   Serial.print(data.pot1); Serial.print(" ");
//   Serial.print(data.pot2); Serial.print(" ");
//   Serial.println(data.pot3);

//   delay(100);
// }

#include <Arduino.h>

struct PotValues {
  int pot1;
  int pot2;
  int pot3;
};

void setup() {
  Serial.begin(9600); // Use hardware serial TX0/RX0
}

void loop() {
  PotValues data;

  data.pot1 = analogRead(A5) * (136.0 / 1023.0) + 1500;
  data.pot2 = analogRead(A4) * (136.0 / 1023.0) + 1500;
  data.pot3 = analogRead(A3) * (136.0 / 1023.0) + 1500;

  // Send raw struct bytes
  Serial.write((uint8_t*)&data, sizeof(data));

  delay(200);
}
