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
 *   RTS,x,y,th,pwm,pwmCurva,vueltas,lookahead,centro,kd,pared
 *                          pose de salida y parámetros (se pueden omitir desde
 *                          la derecha; sin RTS sale del 1er punto viendo al 2º)
 *   RTM,x0,y0,x1,y1,ix0,iy0,ix1,iy1
 *                          paredes de la pista (mm): cuadro exterior e interior
 *   g -> arrancar   x -> parar   (sueltos, sin Enter)
 *
 * LOG (por USB y Bluetooth, BT_NOMBRE):
 *   # ...                               configuración, avisos, resumen
 *   P,t_ms,x,y,th,i,lat,servo,pwm,giro  cada POSE_MS: pose, punto más cercano,
 *                                       error lateral (mm, + = carro a la izquierda
 *                                       de la ruta), servo, PWM y giro medido (°/s)
 *   T,t_ms,sensor,mm,st,senal,amb       cada lectura nueva de un ToF, CRUDA (F/L/R,
 *                                       como VueltaCalibracion)
 *   C,t_ms,sensor,rumbo_deg,n,largo_mm  corrección de rumbo con una pared (modo pared 2)
 *   E,t_ms,texto                        eventos (arranque, fin)
 *
 * PAREDES (modo `pared` de RTS, bits; 0 = solo se registran los ToF):
 *   1  posición: en las rectas, cada lectura lateral se compara con la distancia
 *      que debería dar según el mapa y la pose; la diferencia mueve x, y hacia
 *      donde dice la pared (PARED_K_POS de la diferencia por lectura).
 *   2  rumbo: con varias lecturas de la misma pared a lo largo de la recta, la
 *      pendiente dice si el carro va de verdad paralelo; corrige th.
 *   4  avance: lo mismo que 1 con el frontal contra la pared de enfrente.
 *   8  usar también el cuadro interior. Sin este bit solo cuentan las paredes
 *      EXTERIORES (el borde de la lona): el 2026-10-23 el ToF izquierdo contra el
 *      cuadro interior no coincidía con el derecho contra el exterior (~5 cm en
 *      un lado) y la corrección de rumbo con él se fue a −8° en 3 vueltas.
 *   Solo se usan lecturas en recta (rumbo cerca de un múltiplo de 90, rueda casi
 *   derecha), con el cono entero en una sola pared y poco ángulo de incidencia.
 *
 * SEGURIDAD: para solo si el error lateral pasa de LAT_MAX_MM, si el encoder no
 * avanza, por timeout, o si el ToF frontal (si está) ve algo a < FRENO_FRONTAL_MM.
 *
 * PINES: los de VueltaCalibracion (TB6612 23/18/19, servo 13, encoder A 34 /
 * B 36, BNO RVC -> 25, ToF SDA 21 SCL 22 con XSHUT frontal 0 / izq 15 / der 5,
 * trasero XSHUT 4 apagado).
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
const int   FRENO_FRONTAL_MM  = 120;      // ToF frontal con status 0 a menos de esto
const unsigned long FRENO_QUIETO_MS = 200;   // al terminar: el encoder sin moverse esto = ya se paró

// ── Servo (convención de RondaAbierta: servo = centro + X, X > 0 = izquierda) ──
// Centro: el servo con el que va derecho. 2026-10-23: en las rectas de SeguirRuta el
// seguidor lo dejaba en promedio en 74.4 / 75.2 / 74.7 (con 77 iba +5 mm a la izquierda).
// RTS lo cambia sin recompilar (ruta_bt.py --centro).
int       centroServo = 75;
const int SERVO_MIN   = 1;     // topes mecánicos del carro: 1..156
const int SERVO_MAX   = 156;

// ── Paredes (ver PAREDES arriba) ─────────────────────────────────────────────
int   paredModo = 0;                      // RTS lo cambia (ruta_bt.py --pared)
const float PARED_K_POS        = 0.15f;   // fracción de la diferencia que se corrige por lectura (~20 Hz)
const float PARED_PASO_MAX_MM  = 10.0f;   // corrección máxima de posición por lectura
const float PARED_GATE_MM      = 120.0f;  // más diferencia que esto: no era esa pared (señal, esquina, cono)
const float PARED_INC_MAX_DEG  = 20.0f;   // ángulo máximo entre el rayo y la perpendicular a la pared
const float PARED_RECTA_DEG    = 8.0f;    // en recta: rumbo a menos de esto de un múltiplo de 90
const float PARED_RUEDA_MAX    = 12.0f;   // ... y la rueda del pure pursuit a menos de esto
const float PARED_K_RUMBO      = 0.5f;    // fracción del error de rumbo que se corrige
const float PARED_RUMBO_MAX    = 3.0f;    // ° máximo por corrección
const float PARED_RUMBO_LARGO  = 300.0f;  // mm de pared que se juntan para estimar el rumbo
const int   PARED_RUMBO_N      = 8;       // ... y lecturas mínimas
const float CONO_MEDIO_DEG     = 7.5f;    // ROI 8 ≈ 15° de cono
const float FRONTAL_CORR_MM    = 30.0f;   // el frontal lee de más (RondaAbierta CORR_FRONTAL_MM, status 0)
// Mapa: cuadro exterior e interior (x0, y0, x1, y1). RTM lo cambia (la lona del
// 2026-10-23 mide 2975, no 3000).
float mapaExt[4] = {0, 0, 3000, 3000};
float mapaInt[4] = {1000, 1000, 2000, 2000};

// ── Log ──────────────────────────────────────────────────────────────────────
const char* BT_NOMBRE = "SeguirRuta";
const unsigned long POSE_MS = 20;

// ── Pines ────────────────────────────────────────────────────────────────────
const int PWMA = 23, A1 = 18, A2 = 19;
const int freqMotor = 1000, resMotor = 8;
const int SERVO_PIN = 13, freqServo = 50, resServo = 16;
const int ENC_A = 34, ENC_B = 36;
const int RVC_RX = 25;
const int XSHUT_TRAS = 4;      // trasero: apagado (todos nacen en 0x29)

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

// ToF: posición de la CARA que mide desde el centro del eje trasero (CAD v2,
// config.SENSOR_MOUNTS; el derecho es espejo del izquierdo, sin medir). Un error
// aquí sale como error en la corrección con paredes: MÍDELO.
// Frontal en Long (ve la pared negra a 1.5 m; freno y avance). Laterales en Short +
// ROI 8 + 50 ms: la mejor de la calibración estática del 2026-10-21 (lateral izq:
// −4/−5/−3/0/−8 mm a 10/20/30/45/60 cm; a 80 ya −40).
struct Tof {
  const char* nombre;
  int      xshut;
  uint8_t  dir;
  bool     largo;                     // modo Long (frontal) o Short
  float    adelante, izquierda, angulo;   // mm, mm, ° (+ = izquierda)
  int      maxMm;                     // más lejos que esto no se usa para corregir
  VL53L1X  s;
  bool     ok;
  uint16_t mm;
  uint8_t  st;
  unsigned long msDato;
  long     usadas;                    // lecturas que sí corrigieron la posición
  // Rumbo contra la pared (corregirConPared): sumas de la recta s -> D
  int      rSeg;
  int      rN;
  float    rS, rS2, rD, rSD, rNT, rS0, rSUlt;
};
Tof tofs[] = {
  {"F", 0,  0x30, true,  141.4f,   0.0f,   0.0f, 1200},
  {"L", 15, 0x31, false, 113.0f,  35.2f,  90.0f,  650},   // adelante = eje delantero (usuario 2026-10-23)
  {"R", 5,  0x32, false, 113.0f, -35.2f, -90.0f,  650},
};
const int N_TOF = sizeof(tofs) / sizeof(tofs[0]);
Tof& tofF = tofs[0];
float ruedaUltima = 0;                // rueda del pure pursuit (para saber si va en recta)
float rumboCorregido = 0;             // ° sumados a th0 por las paredes en esta corrida

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

bool arrancarTof(Tof& t) {
  digitalWrite(t.xshut, HIGH);
  delay(10);
  t.s.setTimeout(500);
  if (!t.s.init()) return false;
  t.s.setAddress(t.dir);
  t.s.setDistanceMode(t.largo ? VL53L1X::Long : VL53L1X::Short);
  t.s.setROISize(8, 8);                  // como RondaAbierta: con 16 ve el piso
  t.s.setMeasurementTimingBudget(50000);
  t.s.startContinuous(50);
  t.msDato = millis();
  return true;
}

void corregirConPared(Tof& t);

void leerTofs() {
  for (int i = 0; i < N_TOF; i++) {
    Tof& t = tofs[i];
    if (!t.ok || !t.s.dataReady()) continue;
    t.s.read(false);
    const VL53L1X::RangingData& r = t.s.ranging_data;
    t.mm = r.range_mm;
    t.st = r.range_status;
    t.msDato = millis();
    if (fase != CORRIENDO) continue;
    Log.printf("T,%lu,%s,%u,%u,%.3f,%.3f\n", millis() - tArranque, t.nombre, (unsigned)t.mm,
               (unsigned)t.st, r.peak_signal_count_rate_MCPS, r.ambient_count_rate_MCPS);
    corregirConPared(t);
  }
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
// Paredes
// ═══════════════════════════════════════════════════════════════════════════════

// Segmento k del mapa: 0..3 el cuadro exterior, 4..7 el interior.
void segmentoMapa(int k, float& ax, float& ay, float& bx, float& by) {
  const float* r = k < 4 ? mapaExt : mapaInt;
  switch (k % 4) {
    case 0:  ax = r[0]; ay = r[1]; bx = r[2]; by = r[1]; break;
    case 1:  ax = r[2]; ay = r[1]; bx = r[2]; by = r[3]; break;
    case 2:  ax = r[2]; ay = r[3]; bx = r[0]; by = r[3]; break;
    default: ax = r[0]; ay = r[3]; bx = r[0]; by = r[1]; break;
  }
}

// Rayo desde (ox, oy) con dirección unitaria (dx, dy) contra el mapa: distancia a
// la primera pared (-1 = ninguna), cuál (seg) y dónde cae en ella (u de 0 a 1).
float rayoMapa(float ox, float oy, float dx, float dy, int& seg, float& u) {
  float best = -1;
  for (int k = 0; k < 8; k++) {
    float ax, ay, bx, by;
    segmentoMapa(k, ax, ay, bx, by);
    float ex = bx - ax, ey = by - ay;
    float den = dx * ey - dy * ex;
    if (fabsf(den) < 1e-6f) continue;              // rayo paralelo a la pared
    float wx = ax - ox, wy = ay - oy;
    float tr = (wx * ey - wy * ex) / den;          // a lo largo del rayo
    float us = (wx * dy - wy * dx) / den;          // a lo largo de la pared
    if (tr <= 0 || us < 0 || us > 1) continue;
    if (best < 0 || tr < best) { best = tr; seg = k; u = us; }
  }
  return best;
}

// Una lectura nueva de un ToF contra el mapa (ver PAREDES arriba).
void corregirConPared(Tof& t) {
  bool lateral = !t.largo;
  int bitPos = lateral ? 1 : 4;
  bool quierePos = paredModo & bitPos, quiereRumbo = lateral && (paredModo & 2);
  if (!quierePos && !quiereRumbo) return;
  if (millis() - tArranque < RAMPA_MS) return;
  bool stOk = lateral ? (t.st == 0 || t.st == 2 || t.st == 7) : t.st == 0;
  if (!stOk || t.mm < 40 || t.mm > t.maxMm) return;
  float th = thActual();
  if (fabsf(normaliza(th - 90.0f * lroundf(th / 90.0f))) > PARED_RECTA_DEG
      || fabsf(ruedaUltima) > PARED_RUEDA_MAX) { t.rN = 0; return; }

  // Dónde está el sensor y hacia dónde ve, según la pose que cree el carro.
  float h = th * DEG_TO_RAD, ch = cosf(h), sh = sinf(h);
  float ox = x + t.adelante * ch - t.izquierda * sh;
  float oy = y + t.adelante * sh + t.izquierda * ch;
  float hr = (th + t.angulo) * DEG_TO_RAD, dx = cosf(hr), dy = sinf(hr);
  int seg = -1;
  float u = 0;
  float esperado = rayoMapa(ox, oy, dx, dy, seg, u);
  if (esperado < 0) return;

  // Perpendicular a la pared, apuntando hacia el sensor.
  float ax, ay, bx, by;
  segmentoMapa(seg, ax, ay, bx, by);
  float largo = hypotf(bx - ax, by - ay);
  float nx = -(by - ay) / largo, ny = (bx - ax) / largo;
  if ((ox - ax) * nx + (oy - ay) * ny < 0) { nx = -nx; ny = -ny; }
  float cosInc = -(dx * nx + dy * ny);
  if (cosInc < cosf(PARED_INC_MAX_DEG * DEG_TO_RAD)) return;
  // Todo el cono en la misma pared: lejos de sus orillas (esquinas, huecos).
  float margen = esperado * tanf(CONO_MEDIO_DEG * DEG_TO_RAD) / cosInc + 30.0f;
  if (u * largo < margen || (1 - u) * largo < margen) return;

  float medido = t.mm - (lateral ? 0.0f : FRONTAL_CORR_MM);
  float e = (medido - esperado) * cosInc;   // + = el carro está más lejos de la pared de lo que cree
  if (fabsf(e) > PARED_GATE_MM) return;
  if (seg >= 4 && !(paredModo & 8)) return;   // pared interior: solo con el bit 8

  if (quierePos) {
    float paso = constrain(PARED_K_POS * e, -PARED_PASO_MAX_MM, PARED_PASO_MAX_MM);
    x += paso * nx;
    y += paso * ny;
    t.usadas++;
  }

  if (quiereRumbo) {
    // Distancia perpendicular MEDIDA contra lo recorrido: su pendiente es lo que de
    // verdad se acerca o aleja de la pared. Lo que debería cambiar según el rumbo
    // creído es n·t. La diferencia, entre n·(perpendicular a t), es el error de rumbo.
    // Con lo medido (no con e) para que las correcciones de posición no la ensucien.
    float s = distanciaMm() - dist0;
    if (seg != t.rSeg || t.rN == 0 || s - t.rSUlt > 200.0f) {
      t.rSeg = seg;
      t.rN = 0;
      t.rS = t.rS2 = t.rD = t.rSD = t.rNT = 0;
      t.rS0 = s;
    }
    float sl = s - t.rS0, D = medido * cosInc;
    t.rN++;
    t.rS += sl; t.rS2 += sl * sl; t.rD += D; t.rSD += sl * D;
    t.rNT += nx * ch + ny * sh;
    t.rSUlt = s;
    if (t.rN >= PARED_RUMBO_N && sl >= PARED_RUMBO_LARGO) {
      float nPerp = -nx * sh + ny * ch;
      float den = t.rN * t.rS2 - t.rS * t.rS;
      if (fabsf(nPerp) > 0.9f && den > 1.0f) {
        float pendiente = (t.rN * t.rSD - t.rS * t.rD) / den;
        float alfa = constrain((pendiente - t.rNT / t.rN) / nPerp * RAD_TO_DEG, -PARED_RUMBO_MAX, PARED_RUMBO_MAX);
        th0 += PARED_K_RUMBO * alfa;
        rumboCorregido += PARED_K_RUMBO * alfa;
        Log.printf("C,%lu,%s,%.2f,%d,%.0f\n", millis() - tArranque, t.nombre, PARED_K_RUMBO * alfa, t.rN, sl);
      }
      t.rN = 0;   // ventana nueva: el rumbo ya cambió
    }
  }
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
    int pm = paredModo;
    sscanf(s + 4, "%f,%f,%f,%d,%d,%d,%f,%d,%f,%d", &sx, &sy, &sth, &p1, &p2, &v, &la, &c, &kd, &pm);
    salX = sx; salY = sy; salTh = sth;
    pwmCrucero = constrain(p1, 0, 255);
    pwmCurva   = constrain(p2, 0, 255);
    vueltas    = max(v, 1);
    lookaheadMm = constrain(la, 50.0f, 1000.0f);
    centroServo = constrain(c, 60, 95);
    kdGiro = constrain(kd, 0.0f, 1.0f);
    paredModo = constrain(pm, 0, 15);
    escribirServo(centroServo);
    Log.printf("# salida x=%.0f y=%.0f th=%.1f pwm=%d/%d vueltas=%d lookahead=%.0f centro=%d kd=%.3f pared=%d\n",
               salX, salY, salTh, pwmCrucero, pwmCurva, vueltas, lookaheadMm, centroServo, kdGiro, paredModo);
  } else if (strncmp(s, "RTM,", 4) == 0) {
    float m[8];
    if (sscanf(s + 4, "%f,%f,%f,%f,%f,%f,%f,%f", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5], &m[6], &m[7]) == 8) {
      for (int i = 0; i < 4; i++) { mapaExt[i] = m[i]; mapaInt[i] = m[4 + i]; }
      Log.printf("# mapa exterior %.0f,%.0f-%.0f,%.0f interior %.0f,%.0f-%.0f,%.0f\n",
                 m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7]);
    } else {
      Log.println("# RTM: faltan números (x0,y0,x1,y1,ix0,iy0,ix1,iy1)");
    }
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
  rumboCorregido = 0;
  ruedaUltima = 0;
  for (int i = 0; i < N_TOF; i++) { tofs[i].usadas = 0; tofs[i].rN = 0; tofs[i].rSeg = -1; }
  if (!buscarCercano()) {
    Log.println("# la salida no va en el sentido de ningún tramo de la ruta: revisa RTS (th)");
    return;
  }
  float lat = errorLateral();
  Log.printf("# SeguirRuta n=%d paso=%.1f cerrada=%d vueltas=%d lookahead=%.0f pwm=%d/%d centro=%d kd=%.3f pared=%d\n",
             rutaN, rutaPaso, rutaCerrada ? 1 : 0, vueltas, lookaheadMm, pwmCrucero, pwmCurva, centroServo, kdGiro,
             paredModo);
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
  if (paredModo)
    Log.printf("# pared modo %d: lecturas usadas F %ld L %ld R %ld | rumbo corregido %.2f grados\n", paredModo,
               tofs[0].usadas, tofs[1].usadas, tofs[2].usadas, rumboCorregido);
  fase = ESPERA;
}

void recorrido() {
  unsigned long t = millis() - tArranque;
  if (t > TIMEOUT_MS) { terminar("timeout"); return; }
  if (tofF.ok && tofF.st == 0 && tofF.mm > 0 && tofF.mm < FRENO_FRONTAL_MM) { terminar("pared enfrente"); return; }
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
  ruedaUltima = ruedaRuta;
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

  // ToF: todos apagados con XSHUT y se prenden uno por uno con su dirección.
  pinMode(XSHUT_TRAS, OUTPUT);
  digitalWrite(XSHUT_TRAS, LOW);
  for (int i = 0; i < N_TOF; i++) { pinMode(tofs[i].xshut, OUTPUT); digitalWrite(tofs[i].xshut, LOW); }
  delay(20);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  for (int i = 0; i < N_TOF; i++) tofs[i].ok = arrancarTof(tofs[i]);

  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N1, RVC_RX, -1);
  uint32_t t0 = millis();
  while (bnoPaquetes < 20 && millis() - t0 < 3000) leerBno();
  bool bnoOk = bnoPaquetes >= 20;

  // Se repite hasta que llegue a estar listo, para que se vea aunque el BT se conecte después.
  while (true) {
    Log.printf("# BNO:%s | encoder:%s | ToF F:%s L:%s R:%s\n", bnoOk ? "OK" : "NO", encOk ? "OK" : "NO",
               tofs[0].ok ? "OK" : "NO (sin freno frontal)", tofs[1].ok ? "OK" : "NO", tofs[2].ok ? "OK" : "NO");
    if (!bnoOk || !encOk) { Log.println("# NO ARRANCA: falta el BNO o el encoder"); delay(1000); continue; }
    break;
  }
}

void loop() {
  leerBno();
  actualizarPose();
  actualizarVelocidades();
  leerTofs();

  static char linUsb[256], linBt[256];
  static int nUsb = 0, nBt = 0;
  leerComandos(Serial, linUsb, nUsb, sizeof(linUsb));
  leerComandos(SerialBT, linBt, nBt, sizeof(linBt));

  static unsigned long tAviso = 0;
  if (fase == CORRIENDO) {
    recorrido();
    // Un ToF que deja de mandar datos se avisa (el frontal se congeló el 2026-10-23).
    if (millis() - tAviso >= 1000) {
      tAviso = millis();
      for (int i = 0; i < N_TOF; i++)
        if (tofs[i].ok && millis() - tofs[i].msDato > 500)
          Log.printf("E,%lu,!! ToF %s sin datos hace %lu ms\n", millis() - tArranque, tofs[i].nombre,
                     millis() - tofs[i].msDato);
    }
  } else if (millis() - tAviso >= 1000) {
    tAviso = millis();
    Log.printf("# listo: ruta n=%d, manda 'g' para arrancar | encoder %.0f mm | ToF F %u/%u L %u/%u R %u/%u (mm/st)\n",
               rutaN, distanciaMm(), tofs[0].mm, tofs[0].st, tofs[1].mm, tofs[1].st, tofs[2].mm, tofs[2].st);
    // Parado: un ToF congelado se rearranca (XSHUT abajo y arriba, ~30 ms).
    for (int i = 0; i < N_TOF; i++) {
      Tof& t = tofs[i];
      if (t.ok && millis() - t.msDato > 2000) {
        digitalWrite(t.xshut, LOW);
        delay(5);
        t.ok = arrancarTof(t);
        Log.printf("# ToF %s estaba congelado: %s\n", t.nombre, t.ok ? "rearrancado" : "NO rearrancó");
      }
    }
  }
}
