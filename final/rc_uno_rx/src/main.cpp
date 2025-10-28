// // #include <Arduino.h>
// // #include <SoftwareSerial.h>
// // #include <Servo.h>

// // Servo servo1;
// // Servo servo2;
// // Servo servo3;

// // SoftwareSerial mySerial(10, 11);

// // struct PotValues {
// //   int pot1;
// //   int pot2;
// //   int pot3;
// // };

// // void setup() {
// //   mySerial.begin(9600);
// //   Serial.begin(9600);
// //   servo1.attach(9);
// //   servo2.attach(6);
// //   servo3.attach(5);

// //   servo1.writeMicroseconds(1500);
// //   servo2.writeMicroseconds(1500);
// //   servo3.writeMicroseconds(1500);
// //   delay(5000);

// // }

// // void loop() {

// //   // if (mySerial.available() >= (int)sizeof(PotValues)) {

// //   //   PotValues data;

// //   //   mySerial.readBytes((uint8_t*)&data, sizeof(data));

// //   //   Serial.print(data.pot1); Serial.print(" ");
// //   //   Serial.print(data.pot2); Serial.print(" ");
// //   //   Serial.println(data.pot3);

// //   //   // servo1.writeMicroseconds(data.pot1);
// //   //   // servo2.writeMicroseconds(data.pot2);
// //   //   // servo3.writeMicroseconds(data.pot3);
// //   // }

// //   if (mySerial.available() >= 1) {
// //     if (mySerial.read() == 0xAA && mySerial.read() == 0x55) {
// //       if (mySerial.available() >= sizeof(PotValues) + 1) {
// //         PotValues data;
// //         mySerial.readBytes((char*)&data, sizeof(data));
// //         byte recvChecksum = mySerial.read();
// //         byte calcChecksum = (data.pot1 + data.pot2 + data.pot3) & 0xFF;
// //         if (recvChecksum == calcChecksum) {
// //           Serial.print(data.pot1); Serial.print(" ");
// //           Serial.print(data.pot2); Serial.print(" ");
// //           Serial.println(data.pot3);
// //         }
// //         else {
// //           Serial.println("Checksum error");
// //         }
// //       }
// //     }
// //   }


// // }
// #include <Arduino.h>

// struct PotValues {
//   int pot1;
//   int pot2;
//   int pot3;
// };

// void setup() {
//   Serial.begin(9600); // Use hardware serial TX0/RX0
// }

// void loop() {
//   PotValues data;

//   data.pot1 = analogRead(A5) * (136.0 / 1023.0) + 1500;
//   data.pot2 = analogRead(A4) * (136.0 / 1023.0) + 1500;
//   data.pot3 = analogRead(A3) * (136.0 / 1023.0) + 1500;

//   // Send raw struct bytes
//   Serial.write((uint8_t*)&data, sizeof(data));

//   delay(200);
// }

#include <Arduino.h>
#include <Servo.h>

Servo servo1, servo2, servo3;

struct PotValues {
  int pot1;
  int pot2;
  int pot3;
};

void setup() {
  Serial.begin(9600); // RX0/TX0
  servo1.attach(9);
  servo2.attach(6);
  servo3.attach(5);

  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);
  delay(2000);
}

void loop() {
  if (Serial.available() >= (int)sizeof(PotValues)) {
    PotValues data;
    Serial.readBytes((char*)&data, sizeof(data));

    Serial.print("Pot1: "); Serial.print(data.pot1);
    Serial.print("  Pot2: "); Serial.print(data.pot2);
    Serial.print("  Pot3: "); Serial.println(data.pot3);

    servo1.writeMicroseconds(data.pot1);
    servo2.writeMicroseconds(data.pot2);
    servo3.writeMicroseconds(data.pot3);
  }
  delay(100);
}

