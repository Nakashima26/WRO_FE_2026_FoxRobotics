/*
 * TofTest — lee 4 VL53L1X en el mismo bus I2C.
 * Todos salen de fábrica con la dirección 0x29, así que se prenden uno por uno
 * con XSHUT y a cada uno se le asigna una dirección nueva.
 *
 * Librería: "VL53L1X" de Pololu (Library Manager).
 * Monitor serie a 115200.
 *
 * PINES (no chocan con PurePursuit.ino):
 *   I2C compartido : SDA=21, SCL=22  (el mismo bus del MPU-6050, 0x68)
 *   ToF 0 XSHUT=4   -> 0x30
 *   ToF 1 XSHUT=25  -> 0x31
 *   ToF 2 XSHUT=5   -> 0x32
 *   ToF 3 XSHUT=15  -> 0x33
 */

#include <Wire.h>
#include <VL53L1X.h>

const int N_TOF = 4;
const int XSHUT_PINS[N_TOF] = {4, 25, 5, 15};
const uint8_t DIRS[N_TOF]   = {0x30, 0x31, 0x32, 0x33};

const uint32_t BUDGET_US = 50000;  // tiempo por medición
const uint32_t PERIODO_MS = 50;    // cada cuánto mide (>= budget)

VL53L1X tof[N_TOF];
bool tofOk[N_TOF];

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // Todos apagados
  for (int i = 0; i < N_TOF; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  delay(10);

  // Prender uno por uno y cambiarle la dirección
  for (int i = 0; i < N_TOF; i++) {
    pinMode(XSHUT_PINS[i], INPUT);   // suelta XSHUT (el módulo trae pull-up)
    delay(10);

    tof[i].setTimeout(500);
    tofOk[i] = tof[i].init();
    if (tofOk[i]) {
      tof[i].setAddress(DIRS[i]);
      tof[i].setDistanceMode(VL53L1X::Long);
      tof[i].setMeasurementTimingBudget(BUDGET_US);
      tof[i].startContinuous(PERIODO_MS);
      Serial.printf("ToF %d OK en 0x%02X\n", i, DIRS[i]);
    } else {
      Serial.printf("ToF %d NO responde (XSHUT=%d)\n", i, XSHUT_PINS[i]);
    }
  }
}

void loop() {
  for (int i = 0; i < N_TOF; i++) {
    if (!tofOk[i]) {
      Serial.print("      --");
      continue;
    }
    uint16_t mm = tof[i].read();
    if (tof[i].timeoutOccurred()) Serial.print(" TIMEOUT");
    else                          Serial.printf(" %5u mm", mm);
  }
  Serial.println();
}
