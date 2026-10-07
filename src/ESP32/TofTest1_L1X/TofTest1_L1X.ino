/*
 * TofTest1_L1X — lee UN solo VL53L1X (dirección de fábrica 0x29, sin XSHUT).
 * Librería: "VL53L1X" de Pololu. Monitor serie a 115200.
 *
 * PINES:
 *   VIN=3.3V, GND=GND, SDA=21, SCL=22
 *   XSHUT y GPIO1 sin conectar
 */

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X tof;

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println("ToF NO responde: revisa SDA/SCL/VIN/GND");
    while (true) delay(1000);
  }
  tof.setDistanceMode(VL53L1X::Long);        // Short / Medium / Long (hasta ~4 m)
  tof.setMeasurementTimingBudget(50000);     // 50 ms por medición
  tof.startContinuous(50);
  Serial.println("ToF OK");
}

void loop() {
  uint16_t mm = tof.read();
  if (tof.timeoutOccurred()) Serial.println("TIMEOUT");
  else                       Serial.printf("%u mm\n", mm);
}
