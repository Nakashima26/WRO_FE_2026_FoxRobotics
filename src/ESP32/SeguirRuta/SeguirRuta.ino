/*
 * SeguirRuta — el carro sigue una ruta de puntos (x, y en mm) que le manda la

 * PC, con pure pursuit sobre su propia odometría (encoder + BNO). Sirve para
 * probar el seguimiento SIN la Raspberry ni la cámara. En la PC:
 *   scripts/ruta_bt.py   genera la ruta (recta, carril, s, vuelta o archivo),
 *                        la simula, la sube, arranca el carro, graba el log y
 *                        grafica la ruta pedida contra la que hizo el carro
 *
 * POSICIÓN (x, y, th): la de VueltaCalibracion.
 *   - mm, origen en la esquina EXTERIOR de la pista detrás del carro (o donde
 *     quieras si pruebas en el piso: solo importa que la ruta y la salida usen
 *     el mismo origen).
 *   - th en grados, crece al girar a la IZQUIERDA (0 = +x, 90 = +y).
 *   - El punto que se sigue es el centro del eje TRASERO.
 *
 * SEGUIMIENTO: el mismo pure pursuit de rutaSteer() en PurePursuit.ino (modo
 *   RUTA): punto más cercano en una ventana hacia adelante, objetivo a
 *   lookahead mm por la ruta, rueda = atan(2·batalla·lateral / d²). Lo único
 *   que cambia es el mapeo rueda -> servo, que aquí es el medido en el tapete
 *   el 2026-10-22 (radios con servo 1..16 y 140..152).
 *
 * PROTOCOLO (USB o Bluetooth, líneas con Enter). Los tres primeros son los de
 * PurePursuit.ino, así la ruta que suba la Pi después es la misma:
 *   RTB,n,paso[,cerrada]   empieza una ruta de n puntos separados `paso` mm;
 *                          cerrada 1 = vuelta (sin el campo = 1, como PurePursuit), 0 = abierta
 *   RTP,i,x,y,x,y,...      puntos desde el índice i (enteros, mm), en orden
 *   RTE,n                  fin: la ruta queda cargada si llegaron los n puntos
 *   RTS,x,y,th,pwm,pwmCurva,vueltas,lookahead,centro,kd
 *                          pose de salida y parámetros (se pueden omitir desde
 *                          la derecha; sin RTS sale del 1er punto viendo al 2º)
 *   g -> arrancar   x -> parar   (sueltos, sin Enter)
 *
 * LOG (por USB y Bluetooth, BT_NOMBRE):
 *   # ...                               configuración, avisos, resumen
 *   P,t_ms,x,y,th,i,lat,servo,pwm,giro  cada POSE_MS: pose, punto más cercano,
 *                                       error lateral (mm, + = carro a la izquierda
 *                                       de la ruta), servo, PWM y giro medido (°/s)
 *   E,t_ms,texto                        eventos (arranque, fin)
 *
 * SEGURIDAD: para solo si el error lateral pasa de LAT_MAX_MM, si el encoder no
 * avanza, por timeout, o si el ToF frontal (si está) ve algo a < FRENO_FRONTAL_MM.
 *
 * PINES: los de VueltaCalibracion (TB6612 23/18/19, servo 13, encoder A 34 /
 * B 36, BNO RVC -> 25, ToF frontal XSHUT 0; los otros ToF se dejan apagados).
 * Librería: "VL53L1X" de Pololu.
 */

#include <Wire.h>
#include <VL53L1X.h>
#include "BluetoothSerial.h"
#include "driver/pulse_cnt.h"

// ── Seguimiento ──────────────────────────────────────────────────────────────
const float BATALLA_MM = 113.0f;   // eje trasero a eje delantero (RUTA_BATALLA_MM)
float lookaheadMm      = 200.0f;   // RUTA_LOOKAHEAD_MM; RTS lo cambia
// Amortiguamiento con el giro del BNO: rueda += kdGiro · (giro que pide el pure
// pursuit − giro medido), en grados de rueda por °/s. A 255 el pure pursuit solo
// oscila al salir de las esquinas (el servo tarda ~0.12 s): esto le quita volante
// cuando el carro ya está girando más de lo pedido. 0 = apagado. RTS lo cambia.
// 2026-10-23 a PWM 255/230: 0 -> lateral rms 8 / max 20 mm; 0.08 -> oscilación
// rápida (~3 Hz) al salir de la 1a esquina; 0.04 -> 2-3 / 6 mm en 3 vueltas.
float kdGiro = 0.04f;
const unsigned long VEL_MS = 20;   // ventana para medir velocidad y giro
// Ventana del punto más cercano: de 5 puntos atrás a esto adelante.
const float VENTANA_MM = 600.0f;

// Rueda -> servo, medido en el tapete a PWM 150 (2026-10-22, radio R en el eje
// trasero, rueda equivalente = atan(batalla / R)):
//   der: servo 16 / 8 / 4 / 1   -> 34 / 39 / 41 / 43°  => ~1.78 servo por grado
//   izq: servo 140 / 144 / 148  -> 36 / 39 / 40°       => ~1.75
// Más allá de RUEDA_MAX_* el servo ya no cierra más el radio.
const float SERVO_POR_RUEDA_IZQ = 1.75f;
const float SERVO_POR_RUEDA_DER = 1.78f;
const float RUEDA_MAX_IZQ = 40.0f;
const float RUEDA_MAX_DER = 42.5f;

// ── Velocidad ────────────────────────────────────────────────────────────────
int   pwmCrucero = 150;   // RTS lo cambia. Medido: PWM 180 ≈ 540 mm/s en recta
int   pwmCurva   = 120;   // con la rueda a >= RUEDA_CURVA_MAX (menos arrastre = encoder más exacto)
const float RUEDA_CURVA_MIN = 10.0f;   // de aquí a RUEDA_CURVA_MAX baja de pwmCrucero a pwmCurva
const float RUEDA_CURVA_MAX = 30.0f;
const int   PWM_ARRANQUE = 90;
const unsigned long RAMPA_MS = 500;

// ── Seguridad ────────────────────────────────────────────────────────────────
const float LAT_MAX_MM        = 400.0f;
const unsigned long TIMEOUT_MS = 120000;
const bool  USAR_TOF_FRONTAL  = true;   // si no arranca, avisa y sigue sin freno
const int   FRENO_FRONTAL_MM  = 120;
const unsigned long FRENO_QUIETO_MS = 200;   // al terminar: el encoder sin moverse esto = ya se paró

// ── Servo (convención de RondaAbierta: servo = centro + X, X > 0 = izquierda) ──
// Centro: el servo con el que va derecho. 2026-10-23: en las rectas de SeguirRuta el
// seguidor lo dejaba en promedio en 74.4 / 75.2 / 74.7 (con 77 iba +5 mm a la izquierda).
// RTS lo cambia sin recompilar (ruta_bt.py --centro).
int       centroServo = 75;
const int SERVO_MIN   = 1;     // topes mecánicos del carro: 1..156
const int SERVO_MAX   = 156;

// ── Log ──────────────────────────────────────────────────────────────────────
const char* BT_NOMBRE = "SeguirRuta";
const unsigned long POSE_MS = 20;

// ── Pines ────────────────────────────────────────────────────────────────────
const int PWMA = 23, A1 = 18, A2 = 19;
const int freqMotor = 1000, resMotor = 8;
const int SERVO_PIN = 13, freqServo = 50, resServo = 16;
const int ENC_A = 34, ENC_B = 36;
const int RVC_RX = 25;
const int XSHUT_FRONTAL = 0;
const int XSHUT_APAGADOS[] = {15, 5, 4};   // L, R y trasero: apagados (todos nacen en 0x29)

// ── Encoder (de VueltaCalibracion) ───────────────────────────────────────────
const float MM_POR_CUENTA = 1.0f / 5.45f;   // medido 2026-10-19 (4031 cuentas en 740 mm)
const int   SIGNO_ENC     = -1;             // adelante cuenta negativo

const int   BNO_SIGNO  = -1;     // yaw del BNO crece a la derecha; th crece a la izquierda
const float BNO_ESCALA = 1.0f;   // el de RondaAbierta (1080 / ángulo medido al girarlo 3 vueltas a mano)

// ═══════════════════════════════════════════════════════════════════════════════
// Estado
// ═══════════════════════════════════════════════════════════════════════════════

BluetoothSerial SerialBT;
class LogDual : public Print {
public:
  size_t write(uint8_t c) override { Serial.write(c); SerialBT.write(c); return 1; }
  size_t write(const uint8_t* buf, size_t n) override { Serial.write(buf, n); SerialBT.write(buf, n); return n; }
};
LogDual Log;

pcnt_unit_handle_t pcnt = NULL;
VL53L1X tofF;
bool    tofOk = false;
uint16_t tofMm = 0;
uint8_t  tofSt = 255;

// BNO
uint8_t  bnoBuf[19];
int      bnoN = 0;
float    yawPrev = 0;
bool     yawInicial = false;
float    anguloGyro = 0;    // °, acumulado, crece a la izquierda
uint32_t bnoPaquetes = 0;

// Ruta (los puntos llegan directo aquí: solo se sube con el carro parado)
const int RUTA_MAX = 1500;   // 30 m a 20 mm
float rutaX[RUTA_MAX], rutaY[RUTA_MAX];
int   rutaN = 0;             // puntos de la ruta cargada (0 = sin ruta)
int   rutaCarga = 0;         // puntos anunciados en RTB
int   rutaRecib = 0;         // puntos recibidos EN ORDEN de la carga en curso
float rutaPaso = 20.0f;
bool  rutaCerrada = true, cargaCerrada = true;
int   rutaI = -1;            // punto más cercano (-1 = buscar en toda)
long  progreso = 0;          // puntos avanzados desde la salida
int   vueltas = 1;           // solo ruta cerrada

// Salida
float salX = 0, salY = 0, salTh = 0;

// Pose
float x = 0, y = 0, th0 = 0;
float distPrev = 0, thPrev = 0, dist0 = 0;
float velMmS = 0, giroDegS = 0;            // medidos cada VEL_MS
float velDPrev = 0, velAPrev = 0;
unsigned long velTPrev = 0;                // 0 = sin medición previa

enum Fase { ESPERA, CORRIENDO };
Fase fase = ESPERA;
unsigned long tArranque = 0, tPose = 0;
int   servoActual = -1, pwmActual = 0;
float latMaxAbs = 0, latSuma2 = 0;
long  latN = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// Actuadores
// ═══════════════════════════════════════════════════════════════════════════════

void escribirServo(int angulo) {
  angulo = constrain(angulo, SERVO_MIN, SERVO_MAX);
  if (angulo == servoActual) return;
  servoActual = angulo;
  int pulso = map(angulo, 0, 180, 500, 2500);
  ledcWrite(SERVO_PIN, (pulso * ((1 << resServo) - 1)) / 20000);
}

void setMotor(int pwm) {
  pwmActual = constrain(pwm, 0, 255);
  ledcWrite(PWMA, pwmActual);
}

void frenar() {
  digitalWrite(A1, HIGH); digitalWrite(A2, HIGH);   // freno corto del TB6612
  ledcWrite(PWMA, 255);
  delay(300);
  ledcWrite(PWMA, 0);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);
  pwmActual = 0;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Sensores
// ═══════════════════════════════════════════════════════════════════════════════

float normaliza(float a) {
  while (a >  180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

void leerBno() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (bnoN == 0 && b != 0xAA) continue;
    if (bnoN == 1 && b != 0xAA) { bnoN = 0; continue; }
    bnoBuf[bnoN++] = b;
    if (bnoN < 19) continue;
    bnoN = 0;
    uint8_t suma = 0;
    for (int i = 2; i < 18; i++) suma += bnoBuf[i];
    if (suma != bnoBuf[18]) continue;
    float yaw = (int16_t)(bnoBuf[3] | (bnoBuf[4] << 8)) * 0.01f;
    if (yawInicial) anguloGyro += BNO_SIGNO * BNO_ESCALA * normaliza(yaw - yawPrev);
    yawPrev = yaw;
    yawInicial = true;
    bnoPaquetes++;
  }
}

bool iniciarPcnt() {
  pcnt_unit_config_t ucfg = {};
  ucfg.low_limit  = -30000;
  ucfg.high_limit =  30000;
  ucfg.flags.accum_count = 1;   // con los watch points de abajo la cuenta pasa de ±30000
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

float distanciaMm() {
  int v = 0;
  pcnt_unit_get_count(pcnt, &v);
  return SIGNO_ENC * v * MM_POR_CUENTA;
}

bool arrancarTofFrontal() {
  digitalWrite(XSHUT_FRONTAL, HIGH);
  delay(10);
  tofF.setTimeout(500);
  if (!tofF.init()) return false;
  tofF.setDistanceMode(VL53L1X::Long);
  tofF.setROISize(8, 8);                  // como RondaAbierta: con 16 ve el piso
  tofF.setMeasurementTimingBudget(50000);
  tofF.startContinuous(50);
  return true;
}

void leerTofFrontal() {
  if (!tofOk || !tofF.dataReady()) return;
  tofF.read(false);
  tofMm = tofF.ranging_data.range_mm;
  tofSt = tofF.ranging_data.range_status;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Pose
// ═══════════════════════════════════════════════════════════════════════════════

float thActual() { return th0 + anguloGyro; }

// Avance del encoder proyectado con el rumbo del BNO (rumbo medio del paso).
void actualizarPose() {
  float d  = distanciaMm();
  float ds = d - distPrev;
  distPrev = d;
  float th = thActual();
  float thMed = (th + thPrev) * 0.5f;
  thPrev = th;
  if (fase != CORRIENDO || ds == 0) return;   // parado, moverlo a mano no cuenta
  x += ds * cosf(thMed * DEG_TO_RAD);
  y += ds * sinf(thMed * DEG_TO_RAD);
}

// Velocidad (encoder) y giro (BNO) en una ventana de VEL_MS: derivar cada loop
// sería puro ruido (una cuenta = 0.18 mm, el BNO manda cada 10 ms).
void actualizarVelocidades() {
  unsigned long now = millis();
  if (velTPrev != 0 && now - velTPrev < VEL_MS) return;
  float d = distanciaMm();
  if (velTPrev != 0) {
    float dt = (now - velTPrev) / 1000.0f;
    velMmS   = (d - velDPrev) / dt;
    giroDegS = (anguloGyro - velAPrev) / dt;
  }
  velTPrev = now;
  velDPrev = d;
  velAPrev = anguloGyro;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Ruta y pure pursuit
// ═══════════════════════════════════════════════════════════════════════════════

int idx(int i) {
  return rutaCerrada ? ((i % rutaN) + rutaN) % rutaN : constrain(i, 0, rutaN - 1);
}

// Punto i de la ruta. Abierta: más allá del último sigue derecho en la dirección
// del último tramo (así el objetivo no se pega al carro al llegar al final).
void puntoRuta(int i, float& px, float& py) {
  if (!rutaCerrada && i > rutaN - 1) {
    float dx = rutaX[rutaN - 1] - rutaX[rutaN - 2], dy = rutaY[rutaN - 1] - rutaY[rutaN - 2];
    float d = max(sqrtf(dx * dx + dy * dy), 1.0f);
    float extra = (i - (rutaN - 1)) * rutaPaso;
    px = rutaX[rutaN - 1] + dx / d * extra;
    py = rutaY[rutaN - 1] + dy / d * extra;
    return;
  }
  int k = idx(i);
  px = rutaX[k];
  py = rutaY[k];
}

// Actualiza rutaI (y progreso). false = no hay punto de la ruta que vaya para
// donde mira el carro (solo al arrancar).
bool buscarCercano() {
  if (rutaI < 0) {
    // Primera vez: el punto más cercano cuyo tramo va para donde mira el carro.
    float h = thActual() * DEG_TO_RAD, c = cosf(h), s = sinf(h);
    float best = 1e18f;
    for (int i = 0; i < rutaN; i++) {
      int a = i, b = i + 1;
      if (!rutaCerrada && b >= rutaN) { a = rutaN - 2; b = rutaN - 1; }
      float tx = rutaX[idx(b)] - rutaX[a], ty = rutaY[idx(b)] - rutaY[a];
      if (tx * c + ty * s < 0.3f * rutaPaso) continue;      // más de ~70° de diferencia
      float dx = rutaX[i] - x, dy = rutaY[i] - y;
      float d2 = dx * dx + dy * dy;
      if (d2 < best) { best = d2; rutaI = i; }
    }
    return rutaI >= 0;
  }
  // Después: ventana hacia adelante, sin saltos hacia atrás.
  int bestO = 0;
  float best = 1e18f;
  int ventana = (int)(VENTANA_MM / rutaPaso) + 1;
  for (int o = -5; o <= ventana; o++) {
    int i = rutaI + o;
    if (!rutaCerrada && (i < 0 || i >= rutaN)) continue;
    int k = idx(i);
    float dx = rutaX[k] - x, dy = rutaY[k] - y;
    float d2 = dx * dx + dy * dy;
    if (d2 < best) { best = d2; bestO = o; }
  }
  progreso += bestO;
  rutaI = idx(rutaI + bestO);
  return true;
}

// Ángulo de rueda del pure pursuit, + = izquierda.
float ruedaPP() {
  float px, py;
  puntoRuta(rutaI + (int)(lookaheadMm / rutaPaso + 0.5f), px, py);
  float h = thActual() * DEG_TO_RAD, c = cosf(h), s = sinf(h);
  float dx = px - x, dy = py - y;
  float adelante  =  dx * c + dy * s;
  float izquierda = -dx * s + dy * c;
  float ld2 = max(adelante * adelante + izquierda * izquierda, 1.0f);
  return atanf(2.0f * BATALLA_MM * izquierda / ld2) * RAD_TO_DEG;
}

int servoParaRueda(float rueda) {
  rueda = constrain(rueda, -RUEDA_MAX_DER, RUEDA_MAX_IZQ);
  float s = rueda * (rueda >= 0 ? SERVO_POR_RUEDA_IZQ : SERVO_POR_RUEDA_DER);
  return centroServo + (int)lroundf(s);
}

// Distancia del eje trasero al tramo actual de la ruta, + = carro a la izquierda.
float errorLateral() {
  int a = rutaI, b = rutaI + 1;
  if (!rutaCerrada && b >= rutaN) { a = rutaN - 2; b = rutaN - 1; }
  float ax = rutaX[a], ay = rutaY[a];
  float tx = rutaX[idx(b)] - ax, ty = rutaY[idx(b)] - ay;
  float d = max(sqrtf(tx * tx + ty * ty), 1.0f);
  return (tx * (y - ay) - ty * (x - ax)) / d;
}

bool rutaTerminada() {
  return rutaCerrada ? progreso >= (long)vueltas * rutaN : rutaI >= rutaN - 1;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Comandos
// ═══════════════════════════════════════════════════════════════════════════════

void procesarLinea(char* s) {
  while (*s == ' ') s++;
  bool esRuta = strncmp(s, "RT", 2) == 0;
  if (esRuta && fase == CORRIENDO) { Log.println("# ignorado: el carro está corriendo"); return; }

  if (strncmp(s, "RTB,", 4) == 0) {
    int n = 0, cerr = 1;
    float paso = 20.0f;
    sscanf(s + 4, "%d,%f,%d", &n, &paso, &cerr);
    if (n > RUTA_MAX) Log.printf("# ruta de %d puntos: el máximo es %d\n", n, RUTA_MAX);
    rutaCarga = constrain(n, 0, RUTA_MAX);
    rutaPaso = paso >= 1.0f ? paso : 20.0f;
    cargaCerrada = cerr != 0;
    rutaRecib = 0;
    rutaN = 0;   // la ruta anterior se pisa
  } else if (strncmp(s, "RTP,", 4) == 0) {
    char* p = s + 4;
    int i = strtol(p, &p, 10);
    while (*p == ',' && i < rutaCarga) {
      float px = strtof(p + 1, &p);
      if (*p != ',') break;
      float py = strtof(p + 1, &p);
      if (i == rutaRecib) {   // solo en orden: una línea perdida = ruta rechazada
        rutaX[i] = px;
        rutaY[i] = py;
        rutaRecib++;
      }
      i++;
    }
  } else if (strncmp(s, "RTE,", 4) == 0) {
    int n = atoi(s + 4);
    if (n == rutaCarga && rutaRecib == n && n >= 10) {
      rutaN = n;
      rutaCerrada = cargaCerrada;
      // Salida por omisión: el primer punto, viendo hacia el segundo.
      salX = rutaX[0];
      salY = rutaY[0];
      salTh = atan2f(rutaY[1] - rutaY[0], rutaX[1] - rutaX[0]) * RAD_TO_DEG;
      float largo = 0;
      for (int i = 1; i < n; i++) largo += hypotf(rutaX[i] - rutaX[i - 1], rutaY[i] - rutaY[i - 1]);
      Log.printf("# ruta OK n=%d paso=%.1f cerrada=%d largo=%.0f mm\n", n, rutaPaso, rutaCerrada ? 1 : 0, largo);
    } else {
      Log.printf("# ruta RECHAZADA: llegaron %d de %d puntos (RTE dice %d)\n", rutaRecib, rutaCarga, n);
    }
  } else if (strncmp(s, "RTS,", 4) == 0) {
    float sx = salX, sy = salY, sth = salTh, la = lookaheadMm;
    int p1 = pwmCrucero, p2 = pwmCurva, v = vueltas, c = centroServo;
    float kd = kdGiro;
    sscanf(s + 4, "%f,%f,%f,%d,%d,%d,%f,%d,%f", &sx, &sy, &sth, &p1, &p2, &v, &la, &c, &kd);
    salX = sx; salY = sy; salTh = sth;
    pwmCrucero = constrain(p1, 0, 255);
    pwmCurva   = constrain(p2, 0, 255);
    vueltas    = max(v, 1);
    lookaheadMm = constrain(la, 50.0f, 1000.0f);
    centroServo = constrain(c, 60, 95);
    kdGiro = constrain(kd, 0.0f, 1.0f);
    escribirServo(centroServo);
    Log.printf("# salida x=%.0f y=%.0f th=%.1f pwm=%d/%d vueltas=%d lookahead=%.0f centro=%d kd=%.3f\n",
               salX, salY, salTh, pwmCrucero, pwmCurva, vueltas, lookaheadMm, centroServo, kdGiro);
  }
}

// 'g' y 'x' sueltos al principio de una línea; lo demás se junta hasta el Enter.
void leerComandos(Stream& port, char* lin, int& n, int tam) {
  while (port.available()) {
    char c = port.read();
    if (n == 0 && c == 'g') { if (fase == ESPERA) arrancar(); continue; }
    if (n == 0 && c == 'x') { if (fase == CORRIENDO) terminar("parado a mano"); continue; }
    if (c == '\n' || c == '\r') {
      lin[n] = '\0';
      if (n > 0) procesarLinea(lin);
      n = 0;
    } else if (n < tam - 1) {
      lin[n++] = c;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Recorrido
// ═══════════════════════════════════════════════════════════════════════════════

void arrancar() {
  if (rutaN < 10) { Log.println("# no hay ruta: manda RTB / RTP / RTE primero"); return; }
  x = salX; y = salY; th0 = salTh;
  anguloGyro = 0;
  thPrev = th0;
  distPrev = dist0 = distanciaMm();
  rutaI = -1;
  progreso = 0;
  latMaxAbs = latSuma2 = 0;
  latN = 0;
  if (!buscarCercano()) {
    Log.println("# la salida no va en el sentido de ningún tramo de la ruta: revisa RTS (th)");
    return;
  }
  float lat = errorLateral();
  Log.printf("# SeguirRuta n=%d paso=%.1f cerrada=%d vueltas=%d lookahead=%.0f pwm=%d/%d centro=%d kd=%.3f\n",
             rutaN, rutaPaso, rutaCerrada ? 1 : 0, vueltas, lookaheadMm, pwmCrucero, pwmCurva, centroServo, kdGiro);
  Log.printf("# start x=%.0f y=%.0f th=%.1f i=%d lat=%.0f\n", x, y, th0, rutaI, lat);
  if (fabsf(lat) > 150) Log.println("# OJO: la salida queda a más de 15 cm de la ruta");
  tArranque = millis();
  tPose = 0;
  velTPrev = 0;   // la ventana de giro no cruza el reset de anguloGyro
  velMmS = giroDegS = 0;
  digitalWrite(A1, HIGH); digitalWrite(A2, LOW);
  fase = CORRIENDO;
  Log.printf("E,0,ARRANCA\n");
}

void terminar(const char* motivo) {
  unsigned long t = millis() - tArranque;
  float xFreno = x, yFreno = y;
  frenar();
  // Lo que rueda mientras frena también cuenta: así "# fin" es donde de verdad se
  // paró y se puede comparar directo con la cinta (2026-10-23: eran ~2-3 cm).
  float dUlt = distanciaMm();
  unsigned long tUlt = millis(), t0 = millis();
  while (millis() - tUlt < FRENO_QUIETO_MS && millis() - t0 < 1500) {
    leerBno();
    actualizarPose();
    if (distanciaMm() != dUlt) { dUlt = distanciaMm(); tUlt = millis(); }
  }
  escribirServo(centroServo);
  Log.printf("E,%lu,FIN %s\n", t, motivo);
  Log.printf("# fin x=%.0f y=%.0f th=%.1f i=%d recorrido=%.0f mm en %.1f s | frenando %.0f mm | lateral max %.0f mm rms %.0f mm\n",
             x, y, thActual(), rutaI, distanciaMm() - dist0, t / 1000.0f, hypotf(x - xFreno, y - yFreno),
             latMaxAbs, latN ? sqrtf(latSuma2 / latN) : 0.0f);
  fase = ESPERA;
}

void recorrido() {
  unsigned long t = millis() - tArranque;
  if (t > TIMEOUT_MS) { terminar("timeout"); return; }
  if (tofOk && tofSt == 0 && tofMm > 0 && tofMm < FRENO_FRONTAL_MM) { terminar("pared enfrente"); return; }
  // Encoder al revés o sin contar: la posición no avanza y nunca llegaría.
  if (t > 1500 && distanciaMm() - dist0 < 50) {
    terminar("el encoder no avanza (¿SIGNO_ENC al revés o cable suelto?)"); return;
  }

  buscarCercano();
  if (rutaTerminada()) { terminar("ruta completa"); return; }

  float lat = errorLateral();
  latMaxAbs = max(latMaxAbs, fabsf(lat));
  latSuma2 += lat * lat;
  latN++;
  if (fabsf(lat) > LAT_MAX_MM) { terminar("se salió de la ruta"); return; }

  float ruedaRuta = ruedaPP();
  float rueda = ruedaRuta;
  // Amortiguamiento (kdGiro): giro que pide el pure pursuit contra el que mide el BNO
  // (+ = izquierda en los dos). Si ya gira más de lo pedido, le quita volante.
  if (kdGiro > 0 && velMmS > 100) {
    float giroPedido = velMmS * tanf(ruedaRuta * DEG_TO_RAD) / BATALLA_MM * RAD_TO_DEG;
    rueda += kdGiro * (giroPedido - giroDegS);
  }
  escribirServo(servoParaRueda(rueda));

  // Más lento con la rueda cerrada (la de la ruta: la corrección no debe mover el PWM); rampa al arrancar.
  float f = constrain((fabsf(ruedaRuta) - RUEDA_CURVA_MIN) / (RUEDA_CURVA_MAX - RUEDA_CURVA_MIN), 0.0f, 1.0f);
  int pwm = (int)lroundf(pwmCrucero + (pwmCurva - pwmCrucero) * f);
  if (t < RAMPA_MS) pwm = min(pwm, (int)map(t, 0, RAMPA_MS, PWM_ARRANQUE, pwmCrucero));
  setMotor(pwm);

  if (t - tPose >= POSE_MS) {
    tPose = t;
    Log.printf("P,%lu,%.0f,%.0f,%.2f,%d,%.0f,%d,%d,%.0f\n", t, x, y, thActual(), rutaI, lat, servoActual, pwmActual,
               giroDegS);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Setup / loop
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  SerialBT.begin(BT_NOMBRE);
  delay(200);

  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);
  ledcAttach(PWMA, freqMotor, resMotor);
  ledcWrite(PWMA, 0);
  ledcAttach(SERVO_PIN, freqServo, resServo);
  escribirServo(centroServo);

  bool encOk = iniciarPcnt();

  for (int p : XSHUT_APAGADOS) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  pinMode(XSHUT_FRONTAL, OUTPUT);
  digitalWrite(XSHUT_FRONTAL, LOW);
  delay(20);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (USAR_TOF_FRONTAL) tofOk = arrancarTofFrontal();

  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N1, RVC_RX, -1);
  uint32_t t0 = millis();
  while (bnoPaquetes < 20 && millis() - t0 < 3000) leerBno();
  bool bnoOk = bnoPaquetes >= 20;

  // Se repite hasta que llegue a estar listo, para que se vea aunque el BT se conecte después.
  while (true) {
    Log.printf("# BNO:%s | encoder:%s | ToF frontal:%s\n", bnoOk ? "OK" : "NO", encOk ? "OK" : "NO",
               !USAR_TOF_FRONTAL ? "no se usa" : tofOk ? "OK" : "NO (sin freno frontal)");
    if (!bnoOk || !encOk) { Log.println("# NO ARRANCA: falta el BNO o el encoder"); delay(1000); continue; }
    break;
  }
}

void loop() {
  leerBno();
  actualizarPose();
  actualizarVelocidades();
  leerTofFrontal();

  static char linUsb[256], linBt[256];
  static int nUsb = 0, nBt = 0;
  leerComandos(Serial, linUsb, nUsb, sizeof(linUsb));
  leerComandos(SerialBT, linBt, nBt, sizeof(linBt));

  if (fase == CORRIENDO) {
    recorrido();
  } else {
    static unsigned long tAviso = 0;
    if (millis() - tAviso >= 1000) {
      tAviso = millis();
      Log.printf("# listo: ruta n=%d, manda 'g' para arrancar | encoder %.0f mm | ToF %u st %u\n",
                 rutaN, distanciaMm(), tofMm, tofSt);
    }
  }
}
