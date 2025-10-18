#include <Arduino.h>
#include <Wire.h>
const int addr = 0x68;
int16_t ax, ay, az, gx, gy, gz;

void setup() {
  Wire.begin(21, 22);
  Serial.begin(9600);
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(100);
}

void loop() {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("I2C error");
    delay(500);
    return;
  }

  if (Wire.requestFrom(addr, 14, true) != 14) {
    Serial.println("Read error");
    delay(500);
    return;
  }

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // skip temp
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  Serial.print("ax"); Serial.print(ax);
  Serial.print("  ay"); Serial.print(ay);
  Serial.print("  az"); Serial.print(az);
  Serial.print("  gx"); Serial.print(gx);
  Serial.print("  gy"); Serial.print(gy);
  Serial.print("  gz"); Serial.println(gz);

  delay(200);
}
