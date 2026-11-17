/*
 * N30Test — prueba del motor N30 + su encoder (7 CPR) antes de meterlo a
 * PurePursuitCarrera. Monitor a 115200, fin de línea "Nueva línea".
 *
 * ANTES DE CONECTAR AL ESP32: mide con multímetro A y B contra GND girando la
 * llanta a mano. Deben llegar como máximo a 3.3 V (los GPIO 34/36 no aguantan 5 V).
 *
 * PINES (los mismos de PurePursuitCarrera):
 *   Driver TB6612: PWMA=23, AIN1=18, AIN2=19
 *   Encoder: A -> GPIO 34, B -> GPIO 36, pull-ups a 3.3 V
 *
 * Arranca con el motor PARADO y el PWM topado en PWM_MAX (150). Primero prueba
 * con las llantas AL AIRE.
 *
 * Comandos:
 *   w / s / x     adelante / reversa / parar
 *   + / -         PWM ±10
 *   <número>      fija ese PWM (0..PWM_MAX), p. ej. "100"
 *   r             pone las cuentas y la distancia en 0
 *   d<mm>         calibra: empujaste el carro <mm> desde la última 'r'
 *                 -> imprime ENC_CUENTAS_POR_MM (p. ej. "d1000")
 *   a / A         rampa adelante / reversa: sube el PWM de 1 en 1 hasta que
 *                 la llanta se mueve -> PWM de arranque
 *   k / K         barrido adelante / reversa (LLANTAS AL AIRE): ajusta
 *                 v = K·(pwm − PWM0) -> LAZO_K_* y LAZO_PWM0_*
 *   f<pwm>/b<pwm> corrida EN EL PISO adelante / reversa a ese PWM: avanza hasta
 *                 PISO_DIST_MM (o PISO_T_MAX_MS), mide v en la segunda mitad
 *                 del recorrido, frena. Guarda el punto. P. ej. "f100".
 *                 "f255,2500" = 2500 mm en vez de PISO_DIST_MM (para la vel. máx).
 *                 "v2" = velocidad en el último cuarto: si sale > v, aún aceleraba.
 *   z / Z         ajusta K y PWM0 con las corridas de piso adelante / reversa
 *   c             borra las corridas de piso guardadas
 *
 * El servo (GPIO 13) se queda fijo en SERVO_CENTRO para ir derecho.
 *
 * Protección: si el motor está mandado con PWM ≥ STALL_PWM y el encoder no
 * cuenta nada en STALL_MS, se para solo (motor trabado o encoder desconectado).
 *
 * Qué revisar:
 *   1. Con el motor parado, gira la llanta a mano: "AB" debe cambiar entre
 *      00/01/11/10 y "raw" moverse sin saltos raros.
 *   2. 'w' con PWM bajo: si "raw" SUBE, en PurePursuitCarrera ENC_SIGNO = +1;
 *      si BAJA, ENC_SIGNO = -1. Este sketch te lo dice.
 *   3. En el piso: 'r', empuja el carro 1000 mm derecho, "d1000".
 */

#include "driver/pulse_cnt.h"
#include "BluetoothSerial.h"

// Todo sale por USB y por Bluetooth ("CalibFox", el mismo nombre/emparejamiento
// de CalibFox) y los comandos entran por cualquiera: así las corridas de piso
// se mandan sin cable.
BluetoothSerial SerialBT;
class DualIO : public Stream {
 public:
  void begin(unsigned long b) { Serial.begin(b); SerialBT.begin("CalibFox"); }
  int available() override { return Serial.available() + SerialBT.available(); }
  int read() override { return Serial.available() ? Serial.read() : SerialBT.read(); }
  int peek() override { return Serial.available() ? Serial.peek() : SerialBT.peek(); }
  void flush() override { Serial.flush(); }
  size_t write(uint8_t c) override { Serial.write(c); SerialBT.write(c); return 1; }
  size_t write(const uint8_t *b, size_t n) override { Serial.write(b, n); SerialBT.write(b, n); return n; }
};
DualIO IOdual;
#define Serial IOdual

const int PWMA = 23;
const int A1   = 18;
const int A2   = 19;
const int freqMotor = 1000;
const int resMotor  = 8;
const int PWM_MAX   = 255;     // 150 hasta validar sentido/encoder (OK 2026-11-17)

const int ENC_A = 34;
const int ENC_B = 36;

// Solo para mostrar mm y mm/s. Cámbialo por lo que dé "d1000".
float CUENTAS_POR_MM = 7.523f; // N30 medido 2026-11-17 (d1000)
int   SIGNO = 1;               // solo afecta mm y mm/s mostrados; "raw" es sin signo

const int   SERVO_PIN    = 13;
const int   SERVO_CENTRO = 75;     // centro del carro v2
const float PISO_DIST_MM = 1200.0f; // largo de cada corrida de piso
const uint32_t PISO_T_MAX_MS = 5000;

const int      STALL_PWM = 90;
const uint32_t STALL_MS  = 600;

// ---------------- Encoder (PCNT x4, filtro de glitches de 1 us) ----------------
pcnt_unit_handle_t pcnt = NULL;

bool iniciarPcnt() {
  pcnt_unit_config_t ucfg = {};
  ucfg.low_limit  = -30000;
  ucfg.high_limit =  30000;
  ucfg.flags.accum_count = 1;
  if (pcnt_new_unit(&ucfg, &pcnt) != ESP_OK) return false;

  pcnt_glitch_filter_config_t fcfg = {};
  fcfg.max_glitch_ns = 1000;
  pcnt_unit_set_glitch_filter(pcnt, &fcfg);

  pcnt_chan_config_t cfgA = {};
  cfgA.edge_gpio_num  = ENC_A;
  cfgA.level_gpio_num = ENC_B;
  pcnt_channel_handle_t chA;
  pcnt_new_channel(pcnt, &cfgA, &chA);

  pcnt_chan_config_t cfgB = {};
  cfgB.edge_gpio_num  = ENC_B;
  cfgB.level_gpio_num = ENC_A;
  pcnt_channel_handle_t chB;
  pcnt_new_channel(pcnt, &cfgB, &chB);

  pcnt_channel_set_edge_action(chA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_channel_set_edge_action(chB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_unit_add_watch_point(pcnt, ucfg.high_limit);
  pcnt_unit_add_watch_point(pcnt, ucfg.low_limit);
  pcnt_unit_enable(pcnt);
  pcnt_unit_clear_count(pcnt);
  pcnt_unit_start(pcnt);
  return true;
}

int32_t leeRaw() {
  int v = 0;
  pcnt_unit_get_count(pcnt, &v);
  return v;
}

// ---------------- Motor ----------------
int pwm = 60;
int dir = 0;   // 1 adelante, -1 reversa, 0 coast

void aplicaMotor() {
  if (dir == 0) {
    ledcWrite(PWMA, 0);
    digitalWrite(A1, LOW); digitalWrite(A2, LOW);
    return;
  }
  digitalWrite(A1, dir > 0 ? HIGH : LOW);
  digitalWrite(A2, dir > 0 ? LOW : HIGH);
  ledcWrite(PWMA, constrain(pwm, 0, PWM_MAX));
}

void cambiaDir(int d) {
  if (dir != 0 && d != dir) { dir = 0; aplicaMotor(); delay(300); }  // coast antes de invertir
  dir = d;
  aplicaMotor();
}

void parar(const char* motivo) {
  cambiaDir(0);
  if (motivo) { Serial.print("** PARO: "); Serial.println(motivo); }
}

// ---------------- Velocidad ----------------
// Cuentas/s medidas en una ventana bloqueante (para rampa y barrido).
float cuentasPorSeg(uint32_t ms) {
  int32_t c0 = leeRaw();
  uint32_t t0 = millis();
  delay(ms);
  return (float)(leeRaw() - c0) * 1000.0f / (float)(millis() - t0);
}

bool hayTecla() {
  if (Serial.available()) { while (Serial.available()) Serial.read(); return true; }
  return false;
}

// Rampa: sube el PWM hasta que la llanta se mueve (> 2 vueltas del eje/s aprox).
void rampa(int d) {
  Serial.println(d > 0 ? "-- rampa adelante (cualquier tecla aborta) --" : "-- rampa reversa (cualquier tecla aborta) --");
  dir = d;
  for (int p = 0; p <= PWM_MAX; p++) {
    pwm = p; aplicaMotor();
    float cps = cuentasPorSeg(40);
    if (fabsf(cps) > 50.0f) {
      Serial.print("Arranca en PWM "); Serial.print(p);
      Serial.print("  (raw "); Serial.print(cps > 0 ? "SUBE" : "BAJA"); Serial.println(")");
      break;
    }
    if (hayTecla()) { Serial.println("abortado"); break; }
    if (p == PWM_MAX) Serial.println("No se movió ni con PWM_MAX: revisa motor/driver/encoder");
  }
  parar(nullptr);
}

// Barrido: varios PWM, mide v (mm/s) y ajusta una recta por mínimos cuadrados.
void barrido(int d) {
  Serial.println(d > 0 ? "-- barrido adelante, LLANTAS AL AIRE (tecla aborta) --"
                       : "-- barrido reversa, LLANTAS AL AIRE (tecla aborta) --");
  float sx = 0, sy = 0, sxx = 0, sxy = 0; int n = 0;
  dir = d;
  for (int p = 50; p <= PWM_MAX; p += 20) {
    pwm = p; aplicaMotor();
    delay(800);                                     // que se estabilice
    if (hayTecla()) { Serial.println("abortado"); parar(nullptr); return; }
    float cps = cuentasPorSeg(500);
    float v = fabsf(cps) / CUENTAS_POR_MM;          // mm/s (aire: sin carga)
    Serial.print("PWM "); Serial.print(p);
    Serial.print("  cuentas/s "); Serial.print(cps, 0);
    Serial.print("  v "); Serial.print(v, 0); Serial.println(" mm/s");
    if (fabsf(cps) < 50.0f) continue;               // no se movía: fuera del ajuste
    sx += p; sy += v; sxx += (float)p * p; sxy += (float)p * v; n++;
  }
  parar(nullptr);
  if (n < 2) { Serial.println("Pocos puntos para ajustar"); return; }
  float k  = (n * sxy - sx * sy) / (n * sxx - sx * sx);
  float b  = (sy - k * sx) / n;
  float p0 = -b / k;
  Serial.print(d > 0 ? "LAZO_K_FWD = " : "LAZO_K_REV = "); Serial.print(k, 2);
  Serial.print(d > 0 ? "   LAZO_PWM0_FWD = " : "   LAZO_PWM0_REV = "); Serial.println(p0, 1);
  Serial.println("(en el aire sale más rápido que en el piso; úsalo como referencia, y solo si CUENTAS_POR_MM ya está calibrado)");
}

// ---------------- Corridas de piso ----------------
void escribirServo(int ang) {
  int pulso = map(constrain(ang, 0, 180), 0, 180, 500, 2500);
  ledcWrite(SERVO_PIN, (pulso * ((1 << 16) - 1)) / 20000);
}

void frenar() {                     // freno activo del TB6612 (corto) y luego coast
  digitalWrite(A1, HIGH); digitalWrite(A2, HIGH);
  ledcWrite(PWMA, 255);
  delay(300);
  dir = 0; aplicaMotor();
}

const int PISO_N = 16;
int   pisoPwm[2][PISO_N];
float pisoV[2][PISO_N];
int   pisoCnt[2] = {0, 0};

void corridaPiso(int d, int p, float distMm) {
  p = constrain(p, 0, PWM_MAX);
  Serial.print(d > 0 ? "-- piso adelante PWM " : "-- piso reversa PWM "); Serial.print(p);
  Serial.print(" "); Serial.print(distMm, 0); Serial.println(" mm (tecla aborta) --");
  escribirServo(SERVO_CENTRO);
  int32_t c0 = leeRaw();
  float objetivo = distMm * CUENTAS_POR_MM;
  // Ventana de medida: entre 50 % y 95 % del recorrido.
  int32_t cMed0 = 0; uint32_t tMed0 = 0; bool midiendo = false;
  int32_t cMed1 = 0; uint32_t tMed1 = 0;
  int32_t cQ = 0; uint32_t tQ = 0; bool cuarto = false;
  uint32_t t0 = millis();
  dir = d; pwm = p; aplicaMotor();
  const char* fin = "distancia";
  while (true) {
    delay(5);
    uint32_t ahora = millis();
    float rec = fabsf((float)(leeRaw() - c0));
    if (!midiendo && rec >= 0.5f * objetivo) { midiendo = true; cMed0 = leeRaw(); tMed0 = ahora; }
    if (!cuarto && rec >= 0.75f * objetivo) { cuarto = true; cQ = leeRaw(); tQ = ahora; }
    if (rec >= 0.95f * objetivo) break;
    if (ahora - t0 > PISO_T_MAX_MS) { fin = "tiempo"; break; }
    if (ahora - t0 > 600 && rec < 50) { fin = "SIN CUENTAS"; break; }
    if (hayTecla()) { fin = "abortado"; break; }
  }
  cMed1 = leeRaw(); tMed1 = millis();
  frenar();
  delay(400);
  float recTotal = fabsf((float)(leeRaw() - c0)) / CUENTAS_POR_MM;
  if (!midiendo || tMed1 - tMed0 < 100) {
    Serial.print("Sin ventana de medida ("); Serial.print(fin); Serial.println(")");
    return;
  }
  float v = fabsf((float)(cMed1 - cMed0)) / CUENTAS_POR_MM * 1000.0f / (float)(tMed1 - tMed0);
  Serial.print("PWM "); Serial.print(p);
  Serial.print("  v "); Serial.print(v, 0); Serial.print(" mm/s");
  if (cuarto && tMed1 - tQ >= 50) {
    float v2 = fabsf((float)(cMed1 - cQ)) / CUENTAS_POR_MM * 1000.0f / (float)(tMed1 - tQ);
    Serial.print("  v2 "); Serial.print(v2, 0);
  }
  Serial.print("  ventana "); Serial.print(tMed1 - tMed0); Serial.print(" ms");
  Serial.print("  total "); Serial.print(recTotal, 0); Serial.print(" mm (con frenado)");
  Serial.print("  fin "); Serial.println(fin);
  int i = d > 0 ? 0 : 1;
  if (pisoCnt[i] < PISO_N) { pisoPwm[i][pisoCnt[i]] = p; pisoV[i][pisoCnt[i]] = v; pisoCnt[i]++; }
}

void ajustePiso(int d) {
  int i = d > 0 ? 0 : 1, n = pisoCnt[i];
  if (n < 2) { Serial.println("Faltan corridas (mínimo 2)"); return; }
  float sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int j = 0; j < n; j++) {
    float x = pisoPwm[i][j], y = pisoV[i][j];
    sx += x; sy += y; sxx += x * x; sxy += x * y;
  }
  float k = (n * sxy - sx * sy) / (n * sxx - sx * sx);
  float p0 = -((sy - k * sx) / n) / k;
  Serial.print("PISO "); Serial.print(n); Serial.print(" corridas: ");
  Serial.print(d > 0 ? "K_FWD = " : "K_REV = "); Serial.print(k, 2);
  Serial.print(d > 0 ? "   PWM0_FWD = " : "   PWM0_REV = "); Serial.println(p0, 1);
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  ledcAttach(PWMA, freqMotor, resMotor);
  cambiaDir(0);
  ledcAttach(SERVO_PIN, 50, 16);
  escribirServo(SERVO_CENTRO);

  if (!iniciarPcnt()) {
    Serial.println("PCNT no arrancó: sin encoder no se prueba nada");
    while (true) delay(1000);
  }

  Serial.println("N30Test: w/s/x, +/-, número=PWM, r, d<mm>, a/A rampa, k/K barrido, f<pwm>/b<pwm> piso, z/Z ajuste");
  Serial.print("PWM_MAX = "); Serial.print(PWM_MAX);
  Serial.print("   CUENTAS_POR_MM = "); Serial.println(CUENTAS_POR_MM, 3);
}

String   entrada;
int32_t  rawAnt = 0;
uint32_t tAnt = 0;
int32_t  rawStall = 0;
uint32_t tStall = 0;
int      signoReportado = 0;

void procesaLinea(String s) {
  s.trim();
  if (s.length() == 0) return;
  if (s[0] == 'd') {
    float mm = s.substring(1).toFloat();
    int32_t raw = leeRaw();
    if (mm <= 0 || raw == 0) { Serial.println("uso: r, empuja, d1000"); return; }
    CUENTAS_POR_MM = fabsf((float)raw) / mm;
    Serial.print(raw); Serial.print(" cuentas en "); Serial.print(mm, 0); Serial.println(" mm");
    Serial.print(">>> ENC_CUENTAS_POR_MM = "); Serial.print(CUENTAS_POR_MM, 3); Serial.println("f;");
    Serial.print(">>> empujado adelante -> raw "); Serial.print(raw > 0 ? "SUBIÓ" : "BAJÓ");
    Serial.print(" -> ENC_SIGNO = "); Serial.println(raw > 0 ? "+1" : "-1");
    return;
  }
  if ((s[0] == 'f' || s[0] == 'b') && s.length() > 1) {
    int coma = s.indexOf(',');
    float dist = coma > 0 ? s.substring(coma + 1).toFloat() : PISO_DIST_MM;
    if (dist < 300 || dist > 5000) dist = PISO_DIST_MM;
    corridaPiso(s[0] == 'f' ? 1 : -1, s.substring(1, coma > 0 ? coma : s.length()).toInt(), dist);
    rawAnt = leeRaw();
    return;
  }
  if (isDigit(s[0])) {
    pwm = constrain(s.toInt(), 0, PWM_MAX);
    aplicaMotor();
    Serial.print("PWM = "); Serial.println(pwm);
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (entrada.length() == 0) {
      if (c == 'w') { cambiaDir(1);  signoReportado = 0; tStall = millis(); rawStall = leeRaw(); continue; }
      if (c == 's') { cambiaDir(-1); signoReportado = 0; tStall = millis(); rawStall = leeRaw(); continue; }
      if (c == 'x') { parar(nullptr); continue; }
      if (c == '+') { pwm = min(pwm + 10, PWM_MAX); aplicaMotor(); continue; }
      if (c == '-') { pwm = max(pwm - 10, 0);       aplicaMotor(); continue; }
      if (c == 'r') { pcnt_unit_clear_count(pcnt); rawAnt = 0; rawStall = 0; Serial.println("-- cuentas en 0 --"); continue; }
      if (c == 'a') { rampa(1);    continue; }
      if (c == 'A') { rampa(-1);   continue; }
      if (c == 'k') { barrido(1);  continue; }
      if (c == 'K') { barrido(-1); continue; }
      if (c == 'z') { ajustePiso(1);  continue; }
      if (c == 'Z') { ajustePiso(-1); continue; }
      if (c == 'c') { pisoCnt[0] = pisoCnt[1] = 0; Serial.println("-- corridas de piso borradas --"); continue; }
    }
    if (c == '\n' || c == '\r') { procesaLinea(entrada); entrada = ""; }
    else entrada += c;
  }

  uint32_t ahora = millis();
  int32_t raw = leeRaw();

  // Protección: mandado con PWM alto y el encoder no se mueve.
  if (dir != 0 && pwm >= STALL_PWM) {
    if (raw != rawStall) { rawStall = raw; tStall = ahora; }
    else if (ahora - tStall > STALL_MS) parar("sin cuentas con PWM alto (motor trabado o encoder desconectado)");
  } else { rawStall = raw; tStall = ahora; }

  if (ahora - tAnt >= 200) {
    float dt = (ahora - tAnt) / 1000.0f;
    float cps = (raw - rawAnt) / dt;
    rawAnt = raw; tAnt = ahora;

    // Con el motor mandado, di una vez para qué lado cuenta.
    if (dir != 0 && !signoReportado && fabsf(cps) > 100.0f) {
      bool sube = (cps > 0) == (dir > 0);
      Serial.print(">>> adelante el raw "); Serial.print(sube ? "SUBE" : "BAJA");
      Serial.print(" -> ENC_SIGNO = "); Serial.println(sube ? "+1" : "-1");
      signoReportado = 1;
    }

    Serial.print("dir "); Serial.print(dir);
    Serial.print("  pwm "); Serial.print(dir ? pwm : 0);
    Serial.print("  raw "); Serial.print(raw);
    Serial.print("  c/s "); Serial.print(cps, 0);
    Serial.print("  mm "); Serial.print(SIGNO * raw / CUENTAS_POR_MM, 0);
    Serial.print("  mm/s "); Serial.print(SIGNO * cps / CUENTAS_POR_MM, 0);
    Serial.print("  AB "); Serial.print(digitalRead(ENC_A)); Serial.println(digitalRead(ENC_B));
  }
}
