#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

SoftwareSerial mySerial(10, 255);

struct PotValues {
  int pot1;
  int pot2;
  int pot3;
};

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
  servo1.attach(9);
  servo2.attach(6);
  servo3.attach(5);

  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);

}

void loop() {

  if (mySerial.available() >= (int)sizeof(PotValues)) {

    PotValues data;
    PotValues calc;

    mySerial.readBytes((char*)&data, sizeof(data));

    calc.pot1 = (((data.pot1) / 1023) * 268) + 1368;
    calc.pot2 = (((data.pot2) / 1023) * 268) + 1368;
    calc.pot3 = (((data.pot3) / 1023) * 268) + 1368;

    servo1.writeMicroseconds(calc.pot1);
    servo2.writeMicroseconds(calc.pot2);
    servo3.writeMicroseconds(calc.pot3);
  }

}
