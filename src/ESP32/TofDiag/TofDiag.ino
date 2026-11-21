/*
 * TofDiag — diagnóstico de los 4 VL53L1X con la MISMA configuración que
 * PurePursuitCarrera (direcciones 0x30-0x33, laterales/trasero Short 20 ms
 * ROI 8, frontal Long 33 ms, I2C 400 kHz). Monitor serie a 115200.
 *
 * Cada 0.5 s imprime por sensor: ok de init, mediciones nuevas por segundo
 * (dataReady), última distancia, status y cuántas lecturas iguales seguidas.
 * Un sensor sano da ~45/s (laterales) o ~28/s (frontal); uno congelado da 0/s.
 *
 * XSHUT: F=0 L=15 R=4 B=5 (PCB RevB, 2026-11-03).
 */
#include <Wire.h>
#include <VL53L1X.h>

struct Tof {
  const char *nombre; int xshut; uint8_t dir; bool corto;
  VL53L1X s; bool ok; uint32_t nuevas; int mm; int st; uint32_t iguales;
};

Tof t[4] = {
  {"L", 15, 0x30, true},
  {"R",  4, 0x31, true},
  {"B",  5, 0x32, true},
  {"F",  0, 0x33, false},
};

bool arrancar(Tof &x) {
  digitalWrite(x.xshut, HIGH);
  delay(10);
  x.s.setTimeout(500);
  if (!x.s.init()) return false;
  x.s.setAddress(x.dir);
  uint16_t ms = x.corto ? 20 : 33;
  x.s.setDistanceMode(x.corto ? VL53L1X::Short : VL53L1X::Long);
  if (x.corto) x.s.setROISize(8, 8);
  if (!x.s.setMeasurementTimingBudget((uint32_t)ms * 1000UL)) {
    Serial.printf("ToF %s presupuesto rechazado\n", x.nombre);
  }
  x.s.startContinuous(ms);
  return true;
}

void escanear() {
  Serial.print("I2C:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin();
  Wire.setClock(400000);
  for (auto &x : t) { pinMode(x.xshut, OUTPUT); digitalWrite(x.xshut, LOW); }
  delay(10);
  Serial.println("\n== TofDiag ==");
  escanear();
  for (auto &x : t) {
    x.ok = arrancar(x);
    x.mm = -1; x.st = -1;
    Serial.printf("ToF %s (XSHUT %d -> 0x%02X): %s\n", x.nombre, x.xshut, x.dir, x.ok ? "OK" : "FAIL");
  }
  escanear();
}

void loop() {
  static uint32_t t0 = millis();
  for (auto &x : t) {
    if (!x.ok) continue;
    if (x.s.dataReady()) {
      int mm = x.s.read(false);
      x.st = x.s.ranging_data.range_status;
      x.iguales = (mm == x.mm) ? x.iguales + 1 : 0;
      x.mm = mm;
      x.nuevas++;
    }
  }
  uint32_t dt = millis() - t0;
  if (dt >= 500) {
    for (auto &x : t) {
      if (!x.ok) { Serial.printf("%s: FAIL           | ", x.nombre); continue; }
      Serial.printf("%s: %4d mm st%d %3lu/s rep%lu | ", x.nombre, x.mm, x.st,
                    (unsigned long)(x.nuevas * 1000UL / dt), (unsigned long)x.iguales);
      x.nuevas = 0;
    }
    Serial.println();
    t0 = millis();
  }
}
