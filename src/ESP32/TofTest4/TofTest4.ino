/*
 * TofTest4 — lee 4 VL53L1X a la vez en el mismo bus I2C.
 * Librería: "VL53L1X" de Pololu. Monitor serie a 115200.
 *
 * PINES (todos: VIN=3.3V, GND, SDA=21, SCL=22):
 *   Frontal    XSHUT = GPIO 0   -> 0x30
 *   Izquierdo  XSHUT = GPIO 15  -> 0x31
 *   Derecho    XSHUT = GPIO 5   -> 0x32
 *   Trasero    XSHUT = GPIO 4   -> 0x33
 *
 * Orden de arranque: todos nacen en 0x29. Se apagan los 4 con XSHUT en bajo
 * y se prenden uno por uno, asignándole a cada uno su dirección.
 */

#include <Wire.h>
#include <VL53L1X.h>

const int XSHUT_TRAS = 4;    // -1 = sin XSHUT (siempre encendido)

struct Tof {
  const char* nombre;
  int         xshut;
  uint8_t     dir;
  VL53L1X     s;
  bool        ok;
};

Tof tofs[] = {
  {"TRAS", XSHUT_TRAS, 0x33},
  {"FREN", 0,          0x30},
  {"IZQ",  15,         0x31},
  {"DER",  5,          0x32},
};
const int N = sizeof(tofs) / sizeof(tofs[0]);

bool responde(uint8_t dir) {
  Wire.beginTransmission(dir);
  return Wire.endTransmission() == 0;
}

void configurar(Tof& t) {
  t.s.setDistanceMode(VL53L1X::Long);       // Short / Medium / Long
  t.s.setMeasurementTimingBudget(50000);    // 50 ms por medición
  t.s.startContinuous(50);
}

// Enciende (si tiene XSHUT), inicializa en 0x29 y le asigna su dirección.
bool arrancar(Tof& t) {
  if (t.xshut >= 0) {
    digitalWrite(t.xshut, HIGH);
    delay(10);                              // tiempo de arranque del sensor
  } else if (!responde(0x29) && responde(t.dir)) {
    // Sin XSHUT y ya trae su dirección (reset sin quitar corriente).
    t.s.setAddress(t.dir);                  // solo actualiza la dirección guardada en el objeto
    t.s.setTimeout(500);
    if (!t.s.init()) return false;
    configurar(t);
    return true;
  }

  t.s.setTimeout(500);
  if (!t.s.init()) return false;            // init habla en 0x29
  t.s.setAddress(t.dir);
  configurar(t);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Apagar primero los que tienen XSHUT para que solo quede el trasero en 0x29.
  for (int i = 0; i < N; i++) {
    if (tofs[i].xshut >= 0) {
      pinMode(tofs[i].xshut, OUTPUT);
      digitalWrite(tofs[i].xshut, LOW);
    }
  }
  delay(20);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  for (int i = 0; i < N; i++) {
    tofs[i].ok = arrancar(tofs[i]);
    Serial.printf("%-4s 0x%02X : %s\n", tofs[i].nombre, tofs[i].dir,
                  tofs[i].ok ? "OK" : "NO RESPONDE");
  }

  Serial.print("Direcciones en el bus:");
  for (uint8_t d = 1; d < 127; d++) if (responde(d)) Serial.printf(" 0x%02X", d);
  Serial.println();
}

uint16_t mm[4];
uint8_t  estado[4];
bool     nuevo[4];

void loop() {
  for (int i = 0; i < N; i++) {
    if (!tofs[i].ok || !tofs[i].s.dataReady()) continue;
    tofs[i].s.read(false);
    mm[i]     = tofs[i].s.ranging_data.range_mm;
    estado[i] = tofs[i].s.ranging_data.range_status;
    nuevo[i]  = true;
  }

  // Imprime una línea cuando todos los sensores activos tienen lectura nueva.
  for (int i = 0; i < N; i++) if (tofs[i].ok && !nuevo[i]) return;

  for (int i = 0; i < N; i++) {
    if (!tofs[i].ok) { Serial.printf("%s:  ---    ", tofs[i].nombre); continue; }
    if (estado[i] == VL53L1X::RangeValid) Serial.printf("%s:%5u   ", tofs[i].nombre, mm[i]);
    else Serial.printf("%s:%5u(%s)   ", tofs[i].nombre, mm[i],
                       VL53L1X::rangeStatusToString((VL53L1X::RangeStatus)estado[i]));
    nuevo[i] = false;
  }
  Serial.println();
}
