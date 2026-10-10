/*
 * CalibFox.ino — Banco de calibracion del Fox (WRO Future Engineers 2026, v2)
 *
 * Sketch INDEPENDIENTE de PurePursuit.ino: no necesita la Pi, ni READY, ni V2.
 * Se controla por texto en el USB (Serial, 115200) y emite CSV. Lo maneja
 * src/RASPI/cam/pure_pursuit/twin/tools/calib/calib_capture.py (o cualquier
 * terminal serie).
 *
 * PINES (copiados de PurePursuit.ino l.31-55, no cambian):
 *   Motor TB6612FNG : PWMA=23 (LEDC 1 kHz, 8 bits), A1=18, A2=19
 *                     adelante = A1 HIGH / A2 LOW ; reversa = A1 LOW / A2 HIGH
 *                     coast = A1 LOW / A2 LOW ; freno = pines de direccion + PWM 0
 *   Servo SG90      : SERVO_PIN=13 (LEDC 50 Hz, 16 bits, 500..2500 us <-> 0..180 deg)
 *   Encoder cuadratura: ENC_A=34, ENC_B=39 (solo-entrada, SIN pull-up interno:
 *                     el modulo del encoder debe traer pull-ups)
 *   HC-SR04 izq/der/frontal: TRIG/ECHO = 27/32, 26/35, 14/33
 *   MPU-6050        : I2C por defecto (SDA=21, SCL=22), libreria MPU6050_tockn
 *
 * COMPILAR (misma configuracion que PurePursuit, ver src/README.md):
 *   arduino-cli core install esp32:esp32          (arduino-esp32 >= 3.0: usa ledcAttach)
 *   arduino-cli lib  install "MPU6050_tockn"      (solo si CALIB_MPU=1, por defecto si)
 *   arduino-cli compile --fqbn esp32:esp32:esp32 src/ESP32/CalibFox/CalibFox.ino
 *   Sin MPU: -DCALIB_MPU=0 (quita la libreria y el I2C; LOOPTIME omite el mpu).
 *
 * COMANDOS (una linea, mayusculas o minusculas; respuesta final "OK ..." o "ERR ...";
 * las lineas que empiezan con '#' son comentarios, el resto de las filas es CSV):
 *   HELP
 *   PING                         -> "OK CalibFox v1"
 *   INFO                         pines, CPMM, periodo, tope de PWM
 *   PWM <v> <ms> [servo]         PWM firmado (-255..255; negativo = reversa) durante
 *                                <ms>, luego COAST. Registra CSV. servo opcional (deg).
 *   RAMP <de> <a> <paso> <hold_ms> [servo]
 *                                escalera de PWM (mismo signo; de/a/paso). Cada escalon
 *                                dura hold_ms. Para v(PWM), zona muerta y pwm0.
 *   COAST <v> <run_ms> <log_ms> [brake]
 *                                corre a v run_ms, luego coast (o freno con "brake") y
 *                                registra log_ms mas: da tau_coast.
 *   ENCZERO                      pone el encoder en 0
 *   ENC                          "OK ENC counts=<n> err=<n>" (err = transiciones dobles,
 *                                es decir cuentas perdidas)
 *   ENCLOG <ms>                  motor APAGADO: registra CSV mientras empujas el carro
 *                                a mano (p. ej. 1 m con regla -> counts_per_mm)
 *   CPMM <valor>                 cuentas/mm usado para la columna v_mm_s (def. 20.73)
 *   SERVO <deg>                  escribe el servo (15..165; igual mapeo que el .ino)
 *   SERVOOFF                     quita los pulsos del servo (se puede mover a mano)
 *   PERIOD <ms>                  periodo de muestreo del CSV (def. 10, 2..1000)
 *   MAXPWM <v>                   tope de |PWM| (def. 200, max 255)
 *   I2CHZ <hz>                   reloj I2C (def. 100000; PurePursuit con ToF usa 400000)
 *   LOOPTIME [n]                 mide la secuencia de sensado del loop() de
 *                                PurePursuit (mpu.update x2 + 3 pings HC-SR04, l.3196
 *                                y l.3287-3302): min/media/p99/max en us por etapa.
 *   TOF <L|R|B|F> <SHORT|LONG> <budget_ms|DEF> <dur_ms> [period_ms]
 *                                tasa real, range_status y mm de un VL53L1X (XSHUT 25/4/5/15).
 *                                DEF = presupuesto por defecto de la libreria (50 ms);
 *                                20 SHORT vs DEF LONG reproduce el hallazgo de fox/tof-rate.
 *                                CSV i,t_ms,mm,status,dt_ms,sig_mcps + "# TOFSUM ...".
 *   STOP                         coast inmediato (tambien aborta una corrida: cualquier
 *                                byte recibido durante una corrida la aborta).
 *
 * CSV: t_ms,pwm,enc_counts,v_mm_s,servo_deg,enc_err
 *   t_ms       ms desde el inicio de la corrida (3 decimales, de micros())
 *   pwm        PWM firmado realmente aplicado (con el recorte a MAXPWM)
 *   enc_counts cuentas x4 con signo desde el inicio de la corrida (adelante = +)
 *   v_mm_s     velocidad por diferencias de cuentas entre muestras / CPMM (nominal;
 *              el ajuste en Python recalcula desde enc_counts)
 *   servo_deg  ultimo angulo comandado
 *   enc_err    acumulado de transiciones dobles del encoder
 *
 * SEGURIDAD
 *  - El motor nunca queda encendido tras una corrida: siempre termina en coast.
 *  - No se invierte el puente H con el motor girando (TB6612 frito el 2026-09-01,
 *    ver PurePursuit.ino l.350): entre sentidos opuestos espera MANIOBRA_FRENO_MS.
 *  - Tope absoluto de duracion por corrida: 20 s.
 *  - Sin la Pi: el carro NO se mueve solo; solo corre lo que escribas.
 *  - SIN VERIFICAR EN HARDWARE: este sketch no se ha compilado ni corrido en el ESP32
 *    (no habia toolchain ni carro). Revisado estaticamente y compilado en host contra
 *    un mock de Arduino (ver tests/test_calib_tools.py si hay g++).
 */

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef CALIB_MPU
#define CALIB_MPU 1
#endif

#if CALIB_MPU
#include <Wire.h>
#include <MPU6050_tockn.h>
MPU6050 mpu(Wire);
static bool mpuReady = false;
#endif

#ifndef CALIB_TOF
#define CALIB_TOF 1          // comando TOF (VL53L1X de Pololu); -DCALIB_TOF=0 si no esta la libreria
#endif
#if CALIB_TOF
#include <Wire.h>
#include <VL53L1X.h>
static VL53L1X tofDev;
#define TOF_XSHUT_L 25
#define TOF_XSHUT_R 4
#define TOF_XSHUT_B 5
#define TOF_XSHUT_F 15
#endif

// ── Pines (identicos a PurePursuit.ino) ───────────────────────────────────────
#define TRIG_L      27
#define ECHO_L      32
#define TRIG_R      26
#define ECHO_R      35
#define TRIG_F      14
#define ECHO_F      33
#define PWMA        23
#define A1          18
#define A2          19
#define SERVO_PIN   13
#define ENC_PIN_A   34
#define ENC_PIN_B   39

// ── PWM (identico a PurePursuit.ino l.95-98) ──────────────────────────────────
static const int freqServo = 50;
static const int resServo  = 16;
static const int freqMotor = 1000;
static const int resMotor  = 8;

// ── Parametros de la herramienta ──────────────────────────────────────────────
static const unsigned long FRENO_MS      = 300;     // = MANIOBRA_FRENO_MS del .ino
static const unsigned long RUN_HARD_MAX_MS = 20000;
static const int   SERVO_MIN_SAFE = 15;
static const int   SERVO_MAX_SAFE = 165;
static const unsigned long PING_TIMEOUT_US = 7000;   // = pulseIn del .ino l.2579
static const int   LT_MAX = 1000;                    // muestras max de LOOPTIME

static float         cpmm        = 20.73f;           // ENC_CUENTAS_POR_MM del .ino l.434
static unsigned long periodMs    = 10;
static int           maxPwm      = 200;
static int           servoDeg    = 90;
static bool          servoActive = false;
static int           motorDir    = 0;                // +1 adelante, -1 reversa, 0 coast
static unsigned long lastCoastMs = 0;
static int           lastRunDir  = 0;

// ── Encoder (misma tabla/ISR que PurePursuit.ino l.279-295, + contador de saltos) ─
static volatile int32_t encCount = 0;
static volatile uint32_t encErr  = 0;
static volatile uint8_t encPrevAB = 0;
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
static const int8_t encTrans[16] = {
  0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0
};

void IRAM_ATTR encIsr() {
  uint8_t a  = digitalRead(ENC_PIN_A) ? 1 : 0;
  uint8_t b  = digitalRead(ENC_PIN_B) ? 1 : 0;
  uint8_t ab = (a << 1) | b;
  int8_t d   = encTrans[(encPrevAB << 2) | ab];
  if (d == 0 && ab != encPrevAB) encErr++;   // salto de 2 bits = flanco perdido
  encPrevAB = ab;
  if (d) encCount += d;
}

static int32_t readEnc() {
  portENTER_CRITICAL(&encMux);
  int32_t c = encCount;
  portEXIT_CRITICAL(&encMux);
  return c;
}

static uint32_t readEncErr() {
  portENTER_CRITICAL(&encMux);
  uint32_t e = encErr;
  portEXIT_CRITICAL(&encMux);
  return e;
}

static void zeroEnc() {
  portENTER_CRITICAL(&encMux);
  encCount = 0;
  encErr   = 0;
  portEXIT_CRITICAL(&encMux);
}

// ── Actuadores ────────────────────────────────────────────────────────────────
static void pwmWrite(int duty) { ledcWrite(PWMA, duty); }

static void servoWrite(int ang) {
  ang = constrain(ang, 0, 180);
  int pulso = map(ang, 0, 180, 500, 2500);
  int duty  = (pulso * ((1 << resServo) - 1)) / 20000;   // = escribirServo() del .ino
  ledcWrite(SERVO_PIN, duty);
  servoDeg = ang;
  servoActive = true;
}

static void motorCoast() {
  pwmWrite(0);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  motorDir = 0;
  lastCoastMs = millis();
}

// Aplica sentido + PWM (>=0). Solo cambia pines si el sentido cambia.
static void motorDrive(int dir, int duty) {
  if (dir != motorDir) {
    if (dir > 0)      { digitalWrite(A1, HIGH); digitalWrite(A2, LOW);  }
    else if (dir < 0) { digitalWrite(A1, LOW);  digitalWrite(A2, HIGH); }
    else              { digitalWrite(A1, LOW);  digitalWrite(A2, LOW);  }
    motorDir = dir;
  }
  pwmWrite(duty);
}

// Antes de arrancar: si el sentido es opuesto al de la corrida anterior, espera a
// que pasen FRENO_MS desde el ultimo coast (no invertir el puente con el motor girando).
static void waitBeforeStart(int dir) {
  if (lastRunDir != 0 && dir != 0 && dir != lastRunDir) {
    while (millis() - lastCoastMs < FRENO_MS) delay(1);
  }
}

// ── Perfiles de PWM ───────────────────────────────────────────────────────────
enum ProfMode { PM_CONST, PM_STAIRS, PM_COAST };

struct Profile {
  ProfMode mode;
  int p0, p1, pstep;          // PWM firmado
  unsigned long holdMs;       // STAIRS: duracion por escalon
  unsigned long runMs;        // CONST/COAST: tiempo con motor
  unsigned long totalMs;      // duracion total del registro
  bool brakeAfter;            // COAST: true = freno (PWM 0 con pines de direccion)
};

// PWM firmado en el instante t (ms). Fuera del tramo con motor devuelve 0.
static int pwmAt(const Profile &p, unsigned long t) {
  switch (p.mode) {
    case PM_CONST: return t < p.runMs ? p.p0 : 0;
    case PM_COAST: return t < p.runMs ? p.p0 : 0;
    case PM_STAIRS: {
      unsigned long idx = t / p.holdMs;
      long v = (long)p.p0 + (long)idx * p.pstep;
      // no pasarse de p1
      if (p.pstep > 0 && v > p.p1) v = p.p1;
      if (p.pstep < 0 && v < p.p1) v = p.p1;
      return (int)v;
    }
  }
  return 0;
}

static int clampPwm(int v) {
  if (v >  maxPwm) return  maxPwm;
  if (v < -maxPwm) return -maxPwm;
  return v;
}

static bool serialAbort() {
  if (Serial.available() > 0) {
    while (Serial.available() > 0) Serial.read();
    return true;
  }
  return false;
}

// Corre el perfil registrando CSV. 0 = ok, 1 = abortado, 2 = duracion invalida
// (no imprime nada: la linea OK/ERR final la da finishRun, una sola por comando).
static int runProfile(const Profile &p) {
  if (p.totalMs == 0 || p.totalMs > RUN_HARD_MAX_MS) return 2;
  int firstPwm = clampPwm(pwmAt(p, 0));
  int dir0 = (firstPwm > 0) - (firstPwm < 0);
  waitBeforeStart(dir0);

  zeroEnc();
  Serial.println("# BEGIN");
  Serial.println("t_ms,pwm,enc_counts,v_mm_s,servo_deg,enc_err");

  const unsigned long periodUs = periodMs * 1000UL;
  unsigned long t0 = micros();
  unsigned long next = t0;
  int32_t prevCnt = 0;
  unsigned long prevT = t0;
  int curDir = 0;
  bool coasted = false;
  bool aborted = false;
  int appliedPwm = 0;

  for (;;) {
    unsigned long now = micros();
    unsigned long tUs = now - t0;
    unsigned long tMs = tUs / 1000UL;
    if (tMs >= p.totalMs) break;
    if (serialAbort()) { aborted = true; break; }

    int want = clampPwm(pwmAt(p, tMs));
    int dir = (want > 0) - (want < 0);
    if (dir == 0) {
      // ENCLOG (PM_CONST con p0 = 0) debe dejar el eje libre para empujar el carro a mano.
      if ((p.mode == PM_COAST && !p.brakeAfter) || (p.mode == PM_CONST && p.p0 == 0)) {
        if (!coasted) { motorCoast(); coasted = true; }
        appliedPwm = 0;
      } else {
        // PWM 0 con los pines en el ultimo sentido = freno (como setMotor(0) del .ino).
        motorDrive(curDir != 0 ? curDir : (motorDir != 0 ? motorDir : 1), 0);
        appliedPwm = 0;
      }
    } else {
      if (curDir != 0 && dir != curDir) { motorCoast(); aborted = true; break; }  // nunca deberia pasar
      curDir = dir;
      motorDrive(dir, abs(want));
      appliedPwm = want;
    }

    if ((long)(now - next) >= 0) {
      next += periodUs;
      int32_t cnt = readEnc();
      float dtS = (now - prevT) / 1.0e6f;
      float v = (dtS > 0.0f) ? ((cnt - prevCnt) / cpmm) / dtS : 0.0f;
      prevCnt = cnt;
      prevT = now;
      char buf[96];
      snprintf(buf, sizeof(buf), "%lu.%03lu,%d,%ld,%.1f,%d,%lu",
               (unsigned long)(tUs / 1000UL), (unsigned long)(tUs % 1000UL),
               appliedPwm, (long)cnt, (double)v, servoDeg, (unsigned long)readEncErr());
      Serial.println(buf);
    }
  }
  motorCoast();
  lastRunDir = curDir != 0 ? curDir : lastRunDir;
  Serial.println(aborted ? "# ABORT" : "# END");
  return aborted ? 1 : 0;
}

static void finishRun(int code, const char *name) {
  if (code == 0)      { Serial.print("OK ");  Serial.println(name); }
  else if (code == 1) { Serial.print("ERR "); Serial.print(name); Serial.println(" abortado"); }
  else                { Serial.print("ERR "); Serial.print(name); Serial.println(" duracion fuera de 1..20000 ms"); }
}

// ── LOOPTIME ──────────────────────────────────────────────────────────────────
static uint32_t ltBuf[5][LT_MAX];   // 0=mpu 1=pingL 2=pingR 3=pingF 4=total
static const char *ltName[5] = {"mpu_update", "ping_L", "ping_R", "ping_F", "total"};

static long ping(int trig, int echo, bool &timeout) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, PING_TIMEOUT_US);   // identico a leerDistancia() l.2579
  timeout = (dur == 0);
  return dur;
}

static int cmpU32(const void *a, const void *b) {
  uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
  return (x > y) - (x < y);
}

static void doLoopTime(int n) {
  if (n < 5) n = 5;
  if (n > LT_MAX) n = LT_MAX;
#if CALIB_MPU
  if (!mpuReady) { Wire.begin(); mpu.begin(); mpuReady = true; }
#endif
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);

  int toL = 0, toR = 0, toF = 0;
  for (int i = 0; i < n; i++) {
    uint32_t t0 = micros();
    uint32_t t1 = t0;
#if CALIB_MPU
    mpu.update();            // 1er mpu.update() de loop() (l.3196)
    mpu.update();            // 2do (l.3287), tal cual el .ino
#endif
    t1 = micros();
    ltBuf[0][i] = t1 - t0;
    bool to;
    ping(TRIG_L, ECHO_L, to); if (to) toL++;
    uint32_t t2 = micros(); ltBuf[1][i] = t2 - t1;
    ping(TRIG_R, ECHO_R, to); if (to) toR++;
    uint32_t t3 = micros(); ltBuf[2][i] = t3 - t2;
    ping(TRIG_F, ECHO_F, to); if (to) toF++;
    uint32_t t4 = micros(); ltBuf[3][i] = t4 - t3;
    ltBuf[4][i] = t4 - t0;
    if (serialAbort()) { n = i; break; }
  }
  Serial.println("# LOOPTIME: secuencia de sensado de PurePursuit.loop() (sin ToF ni Serial2 ni logica)");
  Serial.println("stage,n,min_us,mean_us,p99_us,max_us,timeouts");
  for (int s = 0; s < 5; s++) {
    if (n <= 0) break;
    uint64_t sum = 0;
    for (int i = 0; i < n; i++) sum += ltBuf[s][i];
    uint32_t mn = ltBuf[s][0], mx = ltBuf[s][0];
    for (int i = 1; i < n; i++) { if (ltBuf[s][i] < mn) mn = ltBuf[s][i]; if (ltBuf[s][i] > mx) mx = ltBuf[s][i]; }
    qsort(ltBuf[s], n, sizeof(uint32_t), cmpU32);
    int ip = (int)ceilf(0.99f * n) - 1;
    if (ip < 0) ip = 0;
    int to = (s == 1) ? toL : (s == 2) ? toR : (s == 3) ? toF : 0;
    char buf[128];
    snprintf(buf, sizeof(buf), "%s,%d,%lu,%lu,%lu,%lu,%d", ltName[s], n,
             (unsigned long)mn, (unsigned long)(sum / n), (unsigned long)ltBuf[s][ip],
             (unsigned long)mx, to);
    Serial.println(buf);
  }
  // Jitter del propio loop() vacio de este sketch (referencia del costo de micros()).
  uint32_t a = micros(), b = micros();
  Serial.print("# micros() delta tipico (us): "); Serial.println((unsigned long)(b - a));
}

// ── TOF: tasa real, range_status y alcance valido de un VL53L1X ───────────────
// El init de la libreria de Pololu deja 50 ms de presupuesto (Long) y startContinuous(ms)
// solo cambia el periodo entre mediciones: nunca puede ser < presupuesto. Por eso
// aqui el presupuesto se fija ANTES de startContinuous, igual que en la rama fox/tof-rate.
// TOF <L|R|B|F> <SHORT|LONG> <budget_ms|DEF> <dur_ms> [period_ms]
//   DEF = no llamar setMeasurementTimingBudget (comportamiento de la libreria: 50 ms).
// CSV: i,t_ms,mm,status,dt_ms,sig_mcps ; status 0 = valido. Al final "# TOFSUM ...".
#if CALIB_TOF
static void doTof(char *sensor, char *mode, char *budget, long durMs, long periodMs) {
  int idx = -1; int xs = 0;
  switch (sensor[0]) {
    case 'L': idx = 0; xs = TOF_XSHUT_L; break;
    case 'R': idx = 1; xs = TOF_XSHUT_R; break;
    case 'B': idx = 2; xs = TOF_XSHUT_B; break;
    case 'F': idx = 3; xs = TOF_XSHUT_F; break;
    default: Serial.println("ERR TOF sensor L|R|B|F"); return;
  }
  bool shortMode = !strcmp(mode, "SHORT");
  if (!shortMode && strcmp(mode, "LONG")) { Serial.println("ERR TOF modo SHORT|LONG"); return; }
  bool defBudget = !strcmp(budget, "DEF");
  long bud = defBudget ? 50 : atol(budget);
  if (!defBudget && (bud < 15 || bud > 500)) { Serial.println("ERR TOF presupuesto 15..500 ms o DEF"); return; }
  if (periodMs <= 0) periodMs = bud;

  Wire.begin();
  const int pins[4] = {TOF_XSHUT_L, TOF_XSHUT_R, TOF_XSHUT_B, TOF_XSHUT_F};
  for (int i = 0; i < 4; i++) { pinMode(pins[i], OUTPUT); digitalWrite(pins[i], LOW); }
  delay(10);
  digitalWrite(xs, HIGH);                    // solo este sensor despierto: direccion por defecto
  delay(10);
  tofDev.setTimeout(500);
  if (!tofDev.init()) { Serial.println("ERR TOF init fallo (cableado/XSHUT/direccion)"); return; }
  tofDev.setDistanceMode(shortMode ? VL53L1X::Short : VL53L1X::Long);
  if (!defBudget && !tofDev.setMeasurementTimingBudget((uint32_t)bud * 1000UL)) {
    Serial.println("# aviso: presupuesto rechazado por la libreria");
  }
  tofDev.startContinuous((uint32_t)periodMs);

  Serial.println("# BEGIN");
  Serial.println("i,t_ms,mm,status,dt_ms,sig_mcps");
  unsigned long t0 = millis(), tLast = t0;
  int n = 0, valid = 0;
  int hist[16] = {0};
  bool aborted = false;
  while ((long)(millis() - t0) < durMs) {
    if (serialAbort()) { aborted = true; break; }
    if (tofDev.dataReady()) {
      uint16_t mm = tofDev.read(false);
      uint8_t st = tofDev.rangingData.range_status;
      unsigned long now = millis();
      char buf[80];
      snprintf(buf, sizeof(buf), "%d,%lu,%u,%u,%lu,%.2f", n, now - t0, (unsigned)mm, (unsigned)st,
               now - tLast, (double)tofDev.rangingData.peak_signal_count_rate_MCPS);
      Serial.println(buf);
      tLast = now;
      n++;
      if (st == 0) valid++;
      hist[st & 15]++;
    }
  }
  tofDev.stopContinuous();
  digitalWrite(xs, LOW);
  float secs = (millis() - t0) / 1000.0f;
  Serial.print("# TOFSUM sensor="); Serial.print(sensor);
  Serial.print(" idx="); Serial.print(idx);
  Serial.print(" mode="); Serial.print(mode);
  Serial.print(" budget_ms="); if (defBudget) Serial.print("DEF"); else Serial.print((long)bud);
  Serial.print(" period_ms="); Serial.print(periodMs);
  Serial.print(" n="); Serial.print(n);
  Serial.print(" rate_hz="); Serial.print(secs > 0 ? n / secs : 0.0f, 2);
  Serial.print(" valid="); Serial.print(valid);
  Serial.print(" status_hist=");
  for (int s = 0; s < 16; s++) if (hist[s]) { Serial.print(s); Serial.print(':'); Serial.print(hist[s]); Serial.print(' '); }
  Serial.println();
  Serial.println(aborted ? "# ABORT" : "# END");
}
#endif

// ── Comandos ──────────────────────────────────────────────────────────────────
static void printInfo() {
  Serial.println("# CalibFox v1");
  Serial.println("# pines: PWMA=23 A1=18 A2=19 SERVO=13 ENC_A=34 ENC_B=39 "
                 "TRIG/ECHO L=27/32 R=26/35 F=14/33");
  Serial.print("# cpmm="); Serial.print(cpmm, 3);
  Serial.print(" period_ms="); Serial.print(periodMs);
  Serial.print(" maxpwm="); Serial.print(maxPwm);
  Serial.print(" servo="); Serial.print(servoDeg);
  Serial.print(" enc="); Serial.print((long)readEnc());
  Serial.print(" enc_err="); Serial.println((unsigned long)readEncErr());
}

static char *nextTok(char *&ctx) { return strtok_r(nullptr, " \t,", &ctx); }

static bool getInt(char *&ctx, long &out) {
  char *t = nextTok(ctx);
  if (!t) return false;
  char *e = nullptr;
  out = strtol(t, &e, 10);
  return e != t && *e == '\0';
}

static void handleLine(char *line) {
  char *ctx = nullptr;
  char *cmd = strtok_r(line, " \t,", &ctx);
  if (!cmd) return;
  for (char *c = cmd; *c; c++) if (*c >= 'a' && *c <= 'z') *c -= 32;

  if (!strcmp(cmd, "PING")) { Serial.println("OK CalibFox v1"); return; }
  if (!strcmp(cmd, "HELP") || !strcmp(cmd, "?")) {
    Serial.println("# PWM v ms [servo] | RAMP de a paso hold_ms [servo] | COAST v run_ms log_ms [brake]");
    Serial.println("# ENCZERO | ENC | ENCLOG ms | CPMM x | SERVO deg | SERVOOFF | PERIOD ms | MAXPWM v");
    Serial.println("# I2CHZ hz | LOOPTIME [n] | TOF s modo budget dur [per] | STOP | INFO | PING");
    Serial.println("OK HELP");
    return;
  }
  if (!strcmp(cmd, "INFO")) { printInfo(); Serial.println("OK INFO"); return; }
  if (!strcmp(cmd, "STOP")) { motorCoast(); Serial.println("OK STOP"); return; }

  if (!strcmp(cmd, "ENCZERO")) { zeroEnc(); Serial.println("OK ENCZERO"); return; }
  if (!strcmp(cmd, "ENC")) {
    Serial.print("OK ENC counts="); Serial.print((long)readEnc());
    Serial.print(" err="); Serial.println((unsigned long)readEncErr());
    return;
  }
  if (!strcmp(cmd, "CPMM")) {
    char *t = nextTok(ctx);
    float v = t ? (float)atof(t) : 0.0f;
    if (v < 1.0f || v > 200.0f) { Serial.println("ERR CPMM 1..200"); return; }
    cpmm = v; Serial.println("OK CPMM"); return;
  }
  if (!strcmp(cmd, "PERIOD")) {
    long v; if (!getInt(ctx, v) || v < 2 || v > 1000) { Serial.println("ERR PERIOD 2..1000"); return; }
    periodMs = (unsigned long)v; Serial.println("OK PERIOD"); return;
  }
  if (!strcmp(cmd, "MAXPWM")) {
    long v; if (!getInt(ctx, v) || v < 1 || v > 255) { Serial.println("ERR MAXPWM 1..255"); return; }
    maxPwm = (int)v; Serial.println("OK MAXPWM"); return;
  }
  if (!strcmp(cmd, "I2CHZ")) {
    long v; if (!getInt(ctx, v) || v < 10000 || v > 1000000) { Serial.println("ERR I2CHZ 10000..1000000"); return; }
#if CALIB_MPU
    Wire.begin(); Wire.setClock((uint32_t)v);
#endif
    Serial.println("OK I2CHZ"); return;
  }
  if (!strcmp(cmd, "SERVO")) {
    long v; if (!getInt(ctx, v)) { Serial.println("ERR SERVO deg"); return; }
    if (v < SERVO_MIN_SAFE || v > SERVO_MAX_SAFE) { Serial.println("ERR SERVO fuera de 15..165"); return; }
    servoWrite((int)v); Serial.println("OK SERVO"); return;
  }
  if (!strcmp(cmd, "SERVOOFF")) { ledcWrite(SERVO_PIN, 0); servoActive = false; Serial.println("OK SERVOOFF"); return; }

  if (!strcmp(cmd, "LOOPTIME")) {
    long n = 300; getInt(ctx, n);
    doLoopTime((int)n);
    Serial.println("OK LOOPTIME");
    return;
  }

  if (!strcmp(cmd, "TOF")) {
#if CALIB_TOF
    char *sn = nextTok(ctx); char *md = nextTok(ctx); char *bd = nextTok(ctx);
    long dur = 0, per = 0;
    if (!sn || !md || !bd || !getInt(ctx, dur) || dur < 200 || dur > (long)RUN_HARD_MAX_MS) {
      Serial.println("ERR TOF <L|R|B|F> <SHORT|LONG> <budget_ms|DEF> <dur_ms 200..20000> [period_ms]"); return;
    }
    getInt(ctx, per);
    for (char *c = sn; *c; c++) if (*c >= 'a' && *c <= 'z') *c -= 32;
    for (char *c = md; *c; c++) if (*c >= 'a' && *c <= 'z') *c -= 32;
    for (char *c = bd; *c; c++) if (*c >= 'a' && *c <= 'z') *c -= 32;
    motorCoast();
    doTof(sn, md, bd, dur, per);
    Serial.println("OK TOF");
#else
    Serial.println("ERR TOF no compilado (CALIB_TOF=0)");
#endif
    return;
  }

  if (!strcmp(cmd, "ENCLOG")) {
    long ms; if (!getInt(ctx, ms) || ms < 100 || ms > (long)RUN_HARD_MAX_MS) { Serial.println("ERR ENCLOG 100..20000 ms"); return; }
    motorCoast();
    Profile p = {PM_CONST, 0, 0, 0, 0, 0, (unsigned long)ms, false};
    finishRun(runProfile(p), "ENCLOG");
    return;
  }

  if (!strcmp(cmd, "PWM")) {
    long v, ms, sv = -1;
    if (!getInt(ctx, v) || !getInt(ctx, ms)) { Serial.println("ERR PWM v ms [servo]"); return; }
    if (v < -255 || v > 255) { Serial.println("ERR PWM -255..255"); return; }
    if (getInt(ctx, sv)) {
      if (sv < SERVO_MIN_SAFE || sv > SERVO_MAX_SAFE) { Serial.println("ERR servo 15..165"); return; }
      servoWrite((int)sv);
      delay(400);   // que el servo llegue antes de arrancar
    }
    Profile p = {PM_CONST, (int)v, (int)v, 0, 0, (unsigned long)ms, (unsigned long)ms, false};
    finishRun(runProfile(p), "PWM");
    return;
  }

  if (!strcmp(cmd, "RAMP")) {
    long a, b, st, hold, sv = -1;
    if (!getInt(ctx, a) || !getInt(ctx, b) || !getInt(ctx, st) || !getInt(ctx, hold)) {
      Serial.println("ERR RAMP de a paso hold_ms [servo]"); return;
    }
    if (st == 0 || hold < 20) { Serial.println("ERR RAMP paso!=0 y hold>=20"); return; }
    if ((b - a) * st < 0) st = -st;                    // el paso apunta hacia b
    if ((a < 0 && b > 0) || (a > 0 && b < 0)) { Serial.println("ERR RAMP no cruzar el cero (reversa aparte)"); return; }
    if (a < -255 || a > 255 || b < -255 || b > 255) { Serial.println("ERR RAMP -255..255"); return; }
    long nSteps = (labs(b - a) / labs(st)) + 1;
    unsigned long total = (unsigned long)(nSteps * hold);
    if (total > RUN_HARD_MAX_MS) { Serial.println("ERR RAMP > 20 s: menos escalones o hold menor"); return; }
    if (getInt(ctx, sv)) {
      if (sv < SERVO_MIN_SAFE || sv > SERVO_MAX_SAFE) { Serial.println("ERR servo 15..165"); return; }
      servoWrite((int)sv); delay(400);
    }
    Profile p = {PM_STAIRS, (int)a, (int)b, (int)st, (unsigned long)hold, total, total, false};
    finishRun(runProfile(p), "RAMP");
    return;
  }

  if (!strcmp(cmd, "COAST")) {
    long v, runMs, logMs;
    if (!getInt(ctx, v) || !getInt(ctx, runMs) || !getInt(ctx, logMs)) {
      Serial.println("ERR COAST v run_ms log_ms [brake]"); return;
    }
    if (v < -255 || v > 255 || runMs < 100 || logMs < 100) { Serial.println("ERR COAST args"); return; }
    char *t = nextTok(ctx);
    bool brake = false;
    if (t) { for (char *c = t; *c; c++) if (*c >= 'a' && *c <= 'z') *c -= 32; brake = !strcmp(t, "BRAKE"); }
    Profile p = {PM_COAST, (int)v, (int)v, 0, 0, (unsigned long)runMs, (unsigned long)(runMs + logMs), brake};
    finishRun(runProfile(p), "COAST");
    return;
  }

  Serial.print("ERR comando desconocido: "); Serial.println(cmd);
}

// ── setup / loop ──────────────────────────────────────────────────────────────
static char lineBuf[96];
static size_t lineLen = 0;

void setup() {
  Serial.setTxBufferSize(4096);   // CSV a 10 ms sin bloquear el muestreo
  Serial.begin(115200);

  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);          // coast desde el arranque

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PWMA,      freqMotor, resMotor);
  ledcAttach(SERVO_PIN, freqServo, resServo);
#else
  ledcSetup(0, freqMotor, resMotor); ledcAttachPin(PWMA, 0);
  ledcSetup(1, freqServo, resServo); ledcAttachPin(SERVO_PIN, 1);
#endif
  pwmWrite(0);
  motorCoast();
  servoWrite(90);

  pinMode(ENC_PIN_A, INPUT);
  pinMode(ENC_PIN_B, INPUT);
  encPrevAB = ((digitalRead(ENC_PIN_A) ? 1 : 0) << 1) | (digitalRead(ENC_PIN_B) ? 1 : 0);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), encIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), encIsr, CHANGE);

  delay(300);
  printInfo();
  Serial.println("OK READY");
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      lineLen = 0;
      handleLine(lineBuf);
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                       // linea demasiado larga: descartar
      Serial.println("ERR linea larga");
    }
  }
}
