/*
 * TofTest1 — lee UN solo VL53L0X (dirección de fábrica 0x29, sin XSHUT).
 * Librería: "VL53L0X" de Pololu. Monitor serie a 115200.
 *
 * PINES:
 *   VIN=3.3V, GND=GND, SDA=21, SCL=22
 *   XSHUT y GPIO1 sin conectar
 */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X tof;

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(21, 22);

  tof.setTimeout(100);
  if (!tof.init()) {
    Serial.println("ToF NO responde: revisa SDA/SCL/VIN/GND");
    while (true) delay(1000);
  }
  tof.startContinuous();
  Serial.println("ToF OK");
}

void loop() {
  uint16_t mm = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred()) Serial.println("TIMEOUT");
  else                       Serial.printf("%u mm\n", mm);
  delay(100);
}
