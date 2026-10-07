/*
 * TofBnoTest — lee los 4 VL53L1X (I2C) y el BNO085 (UART-RVC) a la vez.
 * Librería: "VL53L1X" de Pololu. Monitor serie a 115200.
 *
 * ToF (todos: VIN=3.3V, GND, SDA=21, SCL=22):
 *   Frontal    XSHUT = GPIO 0   -> 0x30
 *   Izquierdo  XSHUT = GPIO 15  -> 0x31
 *   Derecho    XSHUT = GPIO 5   -> 0x32
 *   Trasero    XSHUT = GPIO 4   -> 0x33
 *
 * BNO085 en UART-RVC:
 *   VIN=3.3V, GND, PS0=3.3V, PS1=GND, SDA -> GPIO 25
 *
 * Comandos: z -> heading actual en 0 | r -> contadores en 0
 *
 * Cada 100 ms imprime el yaw y la última lectura de cada ToF. Una lectura
 * inválida sale como código:
 *   SIG = signal fail   SGM = sigma fail   WRP = wrap target   OOB = out of bounds
 * Cada segundo imprime las tasas: el BNO debe andar cerca de 100 Hz y cada
 * ToF cerca de 20 Hz (50 ms por medición).
 */

#include <Wire.h>
#include <VL53L1X.h>

// ---------------- ToF ----------------
struct Tof {
  const char* nombre;
  int         xshut;
  uint8_t     dir;
  VL53L1X     s;
  bool        ok;
  uint16_t    mm;
  uint8_t     estado;
  uint32_t    lecturas;     // en el último segundo
  uint32_t    validas;      // en el último segundo
};

Tof tofs[] = {
  {"FREN", 0,  0x30},
  {"IZQ",  15, 0x31},
  {"DER",  5,  0x32},
  {"TRAS", 4,  0x33},
};
const int N = sizeof(tofs) / sizeof(tofs[0]);

bool responde(uint8_t dir) {
  Wire.beginTransmission(dir);
  return Wire.endTransmission() == 0;
}

bool arrancarTof(Tof& t) {
  digitalWrite(t.xshut, HIGH);
  delay(10);
  t.s.setTimeout(500);
  if (!t.s.init()) return false;            // nace en 0x29
  t.s.setAddress(t.dir);
  t.s.setDistanceMode(VL53L1X::Long);
  t.s.setMeasurementTimingBudget(50000);
  t.s.startContinuous(50);
  return true;
}

const char* codigo(uint8_t estado) {
  switch (estado) {
    case VL53L1X::SignalFail:              return "SIG";
    case VL53L1X::SigmaFail:               return "SGM";
    case VL53L1X::WrapTargetFail:          return "WRP";
    case VL53L1X::OutOfBoundsFail:         return "OOB";
    default:                               return "ERR";
  }
}

void leerTofs() {
  for (int i = 0; i < N; i++) {
    Tof& t = tofs[i];
    if (!t.ok || !t.s.dataReady()) continue;
    t.s.read(false);
    t.mm     = t.s.ranging_data.range_mm;
    t.estado = t.s.ranging_data.range_status;
    t.lecturas++;
    if (t.estado == VL53L1X::RangeValid) t.validas++;
  }
}

// ---------------- BNO085 (UART-RVC) ----------------
const int RVC_RX = 25;

uint8_t  buf[19];
int      nBuf = 0;
float    yaw = 0, yawCero = 0;
uint32_t bnoMuestras = 0, bnoPerdidos = 0, bnoMalos = 0;
uint32_t bnoUltimaMs = 0, bnoHuecoMax = 0;
int      bnoUltimoIdx = -1;

float normaliza(float a) {
  while (a >  180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

void procesarPaquete() {
  uint8_t suma = 0;
  for (int i = 2; i < 18; i++) suma += buf[i];
  if (suma != buf[18]) { bnoMalos++; return; }

  uint8_t idx = buf[2];
  if (bnoUltimoIdx >= 0) bnoPerdidos += (uint8_t)(idx - bnoUltimoIdx - 1);
  bnoUltimoIdx = idx;

  yaw = (int16_t)(buf[3] | (buf[4] << 8)) * 0.01f;

  uint32_t ahora = millis();
  bnoHuecoMax = max(bnoHuecoMax, ahora - bnoUltimaMs);
  bnoUltimaMs = ahora;
  bnoMuestras++;
}

void leerBno() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (nBuf == 0 && b != 0xAA) continue;
    if (nBuf == 1 && b != 0xAA) { nBuf = 0; continue; }
    buf[nBuf++] = b;
    if (nBuf == 19) { procesarPaquete(); nBuf = 0; }
  }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < N; i++) {
    pinMode(tofs[i].xshut, OUTPUT);
    digitalWrite(tofs[i].xshut, LOW);
  }
  delay(20);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  for (int i = 0; i < N; i++) {
    tofs[i].ok = arrancarTof(tofs[i]);
    Serial.printf("%-4s 0x%02X : %s\n", tofs[i].nombre, tofs[i].dir,
                  tofs[i].ok ? "OK" : "NO RESPONDE");
  }
  Serial.print("Direcciones en el bus:");
  for (uint8_t d = 1; d < 127; d++) if (responde(d)) Serial.printf(" 0x%02X", d);
  Serial.println();

  // Se abre al final de setup: arrancar los ToF tarda y el buffer se llenaría.
  // 1024 bytes = ~0.5 s de margen si el loop se tarda.
  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N1, RVC_RX, -1);
  bnoUltimaMs = millis();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') { yawCero = yaw; Serial.println("-- cero --"); }
    if (c == 'r') { bnoPerdidos = 0; bnoMalos = 0; Serial.println("-- contadores en 0 --"); }
  }

  leerBno();
  leerTofs();
  leerBno();     // otra vez: que el I2C de los ToF no deje acumular paquetes

  static uint32_t tImprime = 0, tSegundo = 0;
  uint32_t ahora = millis();

  if (ahora - tImprime >= 100) {
    tImprime = ahora;
    Serial.printf("yaw:%8.2f |", normaliza(yaw - yawCero));
    for (int i = 0; i < N; i++) {
      Tof& t = tofs[i];
      if (!t.ok)                                 Serial.printf(" %s:  ---", t.nombre);
      else if (t.estado == VL53L1X::RangeValid)  Serial.printf(" %s:%5u", t.nombre, t.mm);
      else                                       Serial.printf(" %s:  %s", t.nombre, codigo(t.estado));
    }
    Serial.println();
  }

  if (ahora - tSegundo >= 1000) {
    tSegundo = ahora;
    uint32_t hueco = max(bnoHuecoMax, ahora - bnoUltimaMs);
    Serial.printf("== BNO %3u Hz  hueco:%3u ms  perdidos:%u  malos:%u ||",
                  bnoMuestras, hueco, bnoPerdidos, bnoMalos);
    for (int i = 0; i < N; i++) {
      Tof& t = tofs[i];
      Serial.printf(" %s %2u Hz (%2u ok)", t.nombre, t.lecturas, t.validas);
      t.lecturas = 0;
      t.validas  = 0;
    }
    Serial.println();
    bnoMuestras = 0;
    bnoHuecoMax = 0;
  }
}
