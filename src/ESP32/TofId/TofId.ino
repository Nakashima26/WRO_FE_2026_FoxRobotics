/*
 * TofId — dice si el ToF conectado es VL53L0X o VL53L1X (sin librerías).
 * Un solo sensor en 0x29: VIN=3.3V, GND, SDA=21, SCL=22. Monitor a 115200.
 *
 *   VL53L0X : registro 0xC0   (8 bits)  = 0xEE
 *   VL53L1X : registro 0x010F (16 bits) = 0xEACC
 */

#include <Wire.h>

const uint8_t DIR = 0x29;

int leerL0X() {
  Wire.beginTransmission(DIR);
  Wire.write(0xC0);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom(DIR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

int leerL1X() {
  Wire.beginTransmission(DIR);
  Wire.write(0x01);
  Wire.write(0x0F);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom(DIR, (uint8_t)2) != 2) return -1;
  int hi = Wire.read();
  int lo = Wire.read();
  return (hi << 8) | lo;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(21, 22);

  Wire.beginTransmission(DIR);
  if (Wire.endTransmission() != 0) {
    Serial.println("Nada en 0x29: revisa SDA/SCL/VIN/GND");
    return;
  }

  int id0 = leerL0X();
  if (id0 == 0xEE) {
    Serial.println("Es VL53L0X  -> libreria 'VL53L0X' de Pololu");
    return;
  }
  int id1 = leerL1X();
  if (id1 == 0xEACC) {
    Serial.println("Es VL53L1X  -> libreria 'VL53L1X' de Pololu");
    return;
  }
  Serial.printf("Algo responde en 0x29 pero no lo reconozco (0xC0=0x%02X, 0x010F=0x%04X)\n", id0, id1);
}

void loop() {}
