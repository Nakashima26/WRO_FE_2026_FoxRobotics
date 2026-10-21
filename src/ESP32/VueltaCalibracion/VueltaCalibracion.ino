/*
 * VueltaCalibracion — da la vuelta a la pista manejando SOLO con encoder + BNO
 * (los ToF no se usan para manejar) y loguea todo lo que leen los 3 ToF junto
 * con la posición del carro. En la PC:
 *   scripts/grabar_bt.py      graba el log por Bluetooth a un archivo
 *   scripts/analiza_vuelta.py pone la ruta sobre el mapa y compara cada lectura
 *                             con la distancia que DEBIÓ dar según el mapa
 * Así se ve qué status / señal / luz ambiente tiene una pared real y cuáles
 * son fantasmas, en cada parte de la pista.
 *
 * POSICIÓN (x, y, th):
 *   - mm, origen en la esquina EXTERIOR detrás del carro al arrancar. Sentido
 *     IZQ: arranca viendo hacia +x; DER: viendo hacia +y. Así la pista queda
 *     igual en el mapa para los dos sentidos.
 *   - th en grados, crece al girar a la IZQUIERDA (0 = +x, 90 = +y).
 *   - El punto que se sigue es el centro del eje TRASERO (las llantas que mueve
 *     el encoder). En analiza_vuelta.py las posiciones de los ToF se miden desde ahí.
 *   - x += avance * cos(th), y += avance * sin(th): el encoder da el avance y
 *     el BNO el rumbo. La ruta no tiene que ser perfecta: lo que importa es
 *     saber dónde estaba el carro en cada lectura.
 *
 * CÓMO SE PONE EL CARRO:
 *   Centro de una esquina: el eje trasero a START_X_MM de la pared de atrás y
 *   a START_Y_MM de la pared exterior del lado, viendo derecho a lo largo de la
 *   recta. SENTIDO_IZQ = true: la pared exterior queda a su DERECHA y gira a la
 *   izquierda. false: la exterior a su IZQUIERDA y gira a la derecha.
 *   Alinéalo bien: el error de la posición inicial se arrastra toda la vuelta.
 *
 * RECORRIDO: las rectas van por el centro de los pasillos (cuadro de LADO_MM).
 *   El carro no gira en su lugar: empieza el giro RADIO_GIRO_MM antes de la
 *   esquina. Si RADIO_GIRO_MM no es el real, el control lateral (con la misma
 *   posición estimada) lo regresa al centro del pasillo. Al final de cada giro
 *   se imprime el radio medido (línea E): pon ese valor en RADIO_GIRO_MM.
 *
 * LOG (por USB y Bluetooth, BT_NOMBRE):
 *   # ...                          configuración y avisos
 *   P,t_ms,x,y,th,tramo            posición cada POSE_MS
 *   T,t_ms,sensor,mm,st,senal,amb  cada lectura nueva de un ToF, CRUDA
 *                                  (sensor F/L/R, mm = range_mm, st = status,
 *                                  señal y ambiente en MCPS)
 *   E,t_ms,texto                   eventos (giro, radio medido, fin)
 *
 * COMANDOS (USB o BT): g -> arrancar   x -> parar
 *
 * PINES: los de RondaAbierta (TB6612 23/18/19, servo 13, ToF SDA 21 SCL 22
 * con XSHUT 0/15/5, trasero XSHUT 4 apagado, BNO RVC -> 25) y el encoder de
 * RectanguloTest (A -> 34, B -> 36, pull-ups a 3.3 V).
 * Librería: "VL53L1X" de Pololu.
 */

#include <Wire.h>
#include <VL53L1X.h>
#include "BluetoothSerial.h"
#include "driver/pulse_cnt.h"

// ── Recorrido ─────────────────────────────────────────────────────────────────
const bool  SENTIDO_IZQ   = false;     // true = antihorario (gira a la izquierda)
const int   VUELTAS       = 1;
const float LADO_MM       = 2000.0f;  // centro a centro de pasillo (pista 3 m, pasillos de 1 m)
const float START_X_MM    = 500.0f;   // eje trasero a esto de la pared de atrás
const float START_Y_MM    = 500.0f;   // y a esto de la pared exterior del lado
const float RADIO_GIRO_MM = 280.0f;   // radio del giro (con GIRO_FRACCION_TOPE); el log lo mide (línea E): pon ese.
                                      // 2026-10-19 a tope (1.0) midió 183-213 con el encoder 20% corto
                                      // -> ~225 real; 280 es un estimado para 0.8
const float DIST_FINAL_MM = 600.0f;   // tras el último giro, cuánto avanza antes de frenar
const unsigned long TIMEOUT_MS = 120000;

// ── Velocidad ─────────────────────────────────────────────────────────────────
// Lento = posición más exacta. Para ver si los ToF cambian con la velocidad,
// corre una vuelta así y otra con PWM_CRUCERO más alto.
const int PWM_CRUCERO = 180;
const int PWM_GIRO    = 130;   // lento en el giro: con el servo cerrado las llantas se arrastran y el encoder se equivoca
const int PWM_ARRANQUE = 90;
const unsigned long RAMPA_MS = 500;

// ── Control (convenciones de RondaAbierta: servo = centro + X, X > 0 = izquierda)
int       centroServo    = 77;    // medido 2026-10-21 (juego de ±2 alrededor)
const int SERVO_MIN      = 0;
const int SERVO_MAX      = 150;   // en 154 la dirección se traba (calib 2026-10-21)
const int SERVO_TOPE_DER = 0;
const int SERVO_TOPE_IZQ = 150;   // el giro izquierdo más cerrado (radio 154 mm); 156 trababa la dirección
float KpRumbo = 1.0f;                  // grados de servo por grado de error (el de RondaAbierta)
const int PWM_REF_PID = 255;
// Control lateral con la posición estimada: corre el rumbo objetivo
// KP_LATERAL grados por mm fuera del centro del pasillo, hasta MAX_LATERAL_DEG.
const float KP_LATERAL      = 0.05f;   // 10 cm fuera -> 5° de rumbo
const float MAX_LATERAL_DEG = 15.0f;
const float GIRO_ANTICIPO_DEG = 8.0f;  // suelta el tope a esto del rumbo nuevo
// Qué tanto del tope usa en los giros (1.0 = a tope). Menos = radio más grande
// pero menos arrastre de llantas, que es lo que le mete error al encoder.
const float GIRO_FRACCION_TOPE = 0.8f;

// ── ToF: IGUAL que RondaAbierta (lo que se calibra es lo que se corre) ───────
const uint32_t TOF_BUDGET_US  = 50000;
const uint8_t  TOF_ROI        = 8;
const int      TOF_PERIODO_MS = 50;
// Freno de emergencia: el frontal con status 0 a menos de esto.
const int      FRENO_FRONTAL_MM = 120;

// ── Log ───────────────────────────────────────────────────────────────────────
const char* BT_NOMBRE = "VueltaCalib";
const unsigned long POSE_MS = 20;

// ── Pines ─────────────────────────────────────────────────────────────────────
const int PWMA = 23, A1 = 18, A2 = 19;
const int freqMotor = 1000, resMotor = 8;
const int SERVO_PIN = 13, freqServo = 50, resServo = 16;
const int ENC_A = 34, ENC_B = 36;
const int RVC_RX = 25;
const int XSHUT_TRAS = 4;

// ── Encoder (PCNT, de RectanguloTest) ────────────────────────────────────────
const float DIAMETRO_MM        = 42.7f;
// 12 CPR (3 pulsos x 4 de cuadratura) x caja 50.092:1 x engranes 1.25:1 = ~751,
// como MotorTest. (Con el 1.5 de RectanguloTest = 900, la vuelta del 2026-10-19
// salió ~20% más chica de lo que de verdad recorrió.)
const float CUENTAS_POR_VUELTA = 3.0f * 4 * 50.092f * 1.25f;
// Medido 2026-10-19 con el carro avanzando solo (4031 cuentas en 740 mm):
// 5.45 cuentas/mm, no los 5.60 de la fórmula (la llanta rueda como Ø 43.9 mm).
const float MM_POR_CUENTA      = 1.0f / 5.45f;
const int   SIGNO_ENC          = -1;   // adelante cuenta negativo

const int BNO_SIGNO = -1;   // yaw del BNO crece a la derecha; th crece a la izquierda

// ═══════════════════════════════════════════════════════════════════════════════
// Tipos y estado
// ═══════════════════════════════════════════════════════════════════════════════

BluetoothSerial SerialBT;
class LogDual : public Print {
public:
  size_t write(uint8_t c) override { Serial.write(c); SerialBT.write(c); return 1; }
  size_t write(const uint8_t* buf, size_t n) override { Serial.write(buf, n); SerialBT.write(buf, n); return n; }
};
LogDual Log;

struct Tof {
  const char* nombre;   // F / L / R en el log
  int         xshut;
  uint8_t     dir;
  VL53L1X     s;
  bool        ok;
  uint16_t    ultimoMm;
  uint8_t     ultimoSt;
  unsigned long msDato;   // millis() del último dato (válido o no)
  // Resumen para la prueba estática (en espera): lecturas desde el último resumen.
  uint16_t    bufMm[40];
  uint8_t     bufSt[40];
  float       sumaSig;
  int         nBuf;
};
Tof tofs[] = {
  {"F", 0,  0x30},
  {"L", 15, 0x31},
  {"R", 5,  0x32},
};
const int N_TOF = sizeof(tofs) / sizeof(tofs[0]);
Tof& tofF = tofs[0];

pcnt_unit_handle_t pcnt = NULL;

// BNO
uint8_t  bnoBuf[19];
int      bnoN = 0;
float    yawPrev = 0;
bool     yawInicial = false;
float    anguloGyro = 0;    // °, acumulado, crece a la izquierda
uint32_t bnoPaquetes = 0;

// Pose
float x = 0, y = 0, th0 = 0;
float distPrev = 0, thPrev = 0, dist0 = 0;

// Recorrido
enum Fase { ESPERA, RECTA, GIRO, FIN };
Fase  fase = ESPERA;
int   tramo = 0;            // recta actual: 0, 1, 2... (se le suma 1 en cada giro)
float esqX = 0, esqY = 0;   // esquina donde EMPIEZA la recta actual
float distGiro0 = 0, thGiro0 = 0;
unsigned long tArranque = 0, tPose = 0;
int   servoActual = -1, pwmActual = 0;

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
    if (yawInicial) anguloGyro += BNO_SIGNO * normaliza(yaw - yawPrev);
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
  t.s.setDistanceMode(VL53L1X::Long);
  t.s.setROISize(TOF_ROI, TOF_ROI);
  t.s.setMeasurementTimingBudget(TOF_BUDGET_US);
  t.s.startContinuous(TOF_PERIODO_MS);
  return true;
}

// Cada lectura nueva va al log tal cual, sin filtrar ni descartar nada:
// decidir qué sirve es trabajo de analiza_vuelta.py.
void leerTofs() {
  for (int i = 0; i < N_TOF; i++) {
    Tof& t = tofs[i];
    if (!t.ok || !t.s.dataReady()) continue;
    t.s.read(false);
    const VL53L1X::RangingData& r = t.s.ranging_data;
    t.ultimoMm = r.range_mm;
    t.ultimoSt = r.range_status;
    t.msDato   = millis();
    if (t.nBuf < 40) {
      t.bufMm[t.nBuf] = r.range_mm;
      t.bufSt[t.nBuf] = r.range_status;
      t.nBuf++;
      t.sumaSig += r.peak_signal_count_rate_MCPS;
    }
    if (fase == RECTA || fase == GIRO)
      Log.printf("T,%lu,%s,%u,%u,%.3f,%.3f\n", millis() - tArranque, t.nombre,
                 (unsigned)r.range_mm, (unsigned)r.range_status,
                 r.peak_signal_count_rate_MCPS, r.ambient_count_rate_MCPS);
  }
}

// Prueba estática: con el carro parado frente a una pared a distancia conocida
// (medida desde la CARA del sensor), cada segundo sale por sensor la mediana,
// mín-máx, cuántas lecturas de cada status y la señal promedio.
//   # ToF F: med  612 mm (598-631) n=20 st0:0 st2:20 st7:0 otro:0 señal 0.85
// Calibración estática: cambia modo/ROI/presupuesto de los 3 ToF sin recargar
// el sketch (scripts/tof_estatica.py prueba varias configuraciones por distancia).
void configurarTofs(char modo, int roi, int budgetMs) {
  VL53L1X::DistanceMode m = modo == 'S' ? VL53L1X::Short : modo == 'M' ? VL53L1X::Medium : VL53L1X::Long;
  roi = constrain(roi, 4, 16);
  budgetMs = constrain(budgetMs, modo == 'S' ? 20 : 33, 1000);
  for (int i = 0; i < N_TOF; i++) {
    Tof& t = tofs[i];
    if (!t.ok) continue;
    t.s.stopContinuous();
    delay(5);
    t.s.setDistanceMode(m);
    t.s.setROISize(roi, roi);
    t.s.setMeasurementTimingBudget((uint32_t)budgetMs * 1000);
    t.s.startContinuous(budgetMs);
    t.nBuf = 0;
    t.sumaSig = 0;
  }
  Log.printf("# cfg modo=%c roi=%d budget=%d OK\n", modo, roi, budgetMs);
}

void comandoLinea(char* s) {
  while (*s == ' ') s++;
  if (s[0] == 'c') {
    char modo = 'L'; int roi = TOF_ROI, budget = TOF_BUDGET_US / 1000;
    if (sscanf(s + 1, " %c %d %d", &modo, &roi, &budget) >= 1) {
      if (modo >= 'a') modo -= 32;
      configurarTofs(modo, roi, budget);
    }
  } else if (s[0] == 'r') {
    for (int i = 0; i < N_TOF; i++) { tofs[i].nBuf = 0; tofs[i].sumaSig = 0; }
    Log.println("# resumen reiniciado");
  }
}

void imprimirResumenTof() {
  for (int i = 0; i < N_TOF; i++) {
    Tof& t = tofs[i];
    int n = t.nBuf;
    if (n == 0) { Log.printf("# ToF %s: sin datos\n", t.nombre); continue; }
    uint16_t v[40];
    int c0 = 0, c2 = 0, c7 = 0;
    for (int k = 0; k < n; k++) {
      v[k] = t.bufMm[k];
      if (t.bufSt[k] == 0) c0++; else if (t.bufSt[k] == 2) c2++; else if (t.bufSt[k] == 7) c7++;
    }
    // Orden por inserción (n <= 40) para la mediana.
    for (int a = 1; a < n; a++) {
      uint16_t x = v[a]; int b = a - 1;
      while (b >= 0 && v[b] > x) { v[b + 1] = v[b]; b--; }
      v[b + 1] = x;
    }
    Log.printf("# ToF %s: med %4u mm (%u-%u) n=%d st0:%d st2:%d st7:%d otro:%d señal %.2f\n",
               t.nombre, v[n / 2], v[0], v[n - 1], n, c0, c2, c7, n - c0 - c2 - c7, t.sumaSig / n);
    t.nBuf = 0;
    t.sumaSig = 0;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Posición y recorrido
// ═══════════════════════════════════════════════════════════════════════════════

float thActual() { return th0 + anguloGyro; }

// Avance del encoder proyectado con el rumbo del BNO (rumbo medio del paso).
// Solo corriendo: parado, moverlo con la mano para acomodarlo no cuenta.
void actualizarPose() {
  if (fase != RECTA && fase != GIRO) return;
  float d  = distanciaMm();
  float ds = d - distPrev;
  distPrev = d;
  float th = thActual();
  float thMed = (th + thPrev) * 0.5f;
  thPrev = th;
  if (ds == 0) return;
  x += ds * cosf(thMed * DEG_TO_RAD);
  y += ds * sinf(thMed * DEG_TO_RAD);
}

float rumboTramo(int n) { return th0 + (SENTIDO_IZQ ? 90.0f : -90.0f) * n; }

// Avance a lo largo de la recta actual y desvío lateral (+ = a la izquierda).
void enRecta(float& avance, float& lateral) {
  float r = rumboTramo(tramo) * DEG_TO_RAD;
  float dx = x - esqX, dy = y - esqY;
  avance  =  dx * cosf(r) + dy * sinf(r);
  lateral = -dx * sinf(r) + dy * cosf(r);
}

void pidRumbo(float objetivo) {
  float err = constrain(objetivo - thActual(), -20.0f, 20.0f);
  float escala = constrain((float)PWM_REF_PID / max(pwmActual, 1), 0.5f, 1.2f);
  float out = constrain(escala * KpRumbo * err, -25.0f, 25.0f);
  escribirServo(centroServo + (int)out);
}

void arrancar() {
  // IZQ: viendo hacia +x, pared de atrás x = 0, exterior a la derecha (y = 0).
  // DER: viendo hacia +y, pared de atrás y = 0, exterior a la izquierda (x = 0).
  // En los dos casos la esquina exterior de atrás es el origen.
  th0 = SENTIDO_IZQ ? 0.0f : 90.0f;
  x = SENTIDO_IZQ ? START_X_MM : START_Y_MM;
  y = SENTIDO_IZQ ? START_Y_MM : START_X_MM;
  esqX = x; esqY = y;
  anguloGyro = 0;
  thPrev = th0;
  distPrev = dist0 = distanciaMm();
  tramo = 0;
  tArranque = millis();
  tPose = 0;
  Log.printf("# VueltaCalibracion sentido=%s vueltas=%d lado=%.0f radio=%.0f pwm=%d/%d roi=%d budget_us=%lu\n",
             SENTIDO_IZQ ? "IZQ" : "DER", VUELTAS, LADO_MM, RADIO_GIRO_MM, PWM_CRUCERO, PWM_GIRO,
             TOF_ROI, (unsigned long)TOF_BUDGET_US);
  Log.printf("# start x=%.0f y=%.0f th=%.0f\n", x, y, th0);
  for (int i = 0; i < N_TOF; i++) tofs[i].msDato = millis();
  digitalWrite(A1, HIGH); digitalWrite(A2, LOW);
  fase = RECTA;
  Log.printf("E,0,ARRANCA\n");
}

void terminar(const char* motivo) {
  frenar();
  escribirServo(centroServo);
  Log.printf("E,%lu,FIN %s\n", millis() - tArranque, motivo);
  Log.printf("# fin x=%.0f y=%.0f th=%.1f\n", x, y, thActual());
  fase = FIN;
}

void recorrido() {
  unsigned long t = millis() - tArranque;
  if (t > TIMEOUT_MS) { terminar("timeout"); return; }
  if (tofF.ultimoSt == 0 && tofF.ultimoMm > 0 && tofF.ultimoMm < FRENO_FRONTAL_MM) {
    terminar("pared enfrente"); return;
  }
  // Encoder al revés o sin contar: la posición no avanzaría y nunca giraría.
  if (t > 1500 && distanciaMm() - dist0 < 50) {
    terminar("el encoder no avanza (¿SIGNO_ENC al revés o cable suelto?)"); return;
  }

  int girosHechos = tramo;
  bool ultimaRecta = girosHechos >= 4 * VUELTAS;

  if (fase == RECTA) {
    float avance, lateral;
    enRecta(avance, lateral);
    float desvio = constrain(-KP_LATERAL * lateral, -MAX_LATERAL_DEG, MAX_LATERAL_DEG);
    pidRumbo(rumboTramo(tramo) + desvio);

    int pwm = PWM_CRUCERO;
    if (tramo == 0 && t < RAMPA_MS) pwm = map(t, 0, RAMPA_MS, PWM_ARRANQUE, PWM_CRUCERO);
    setMotor(pwm);

    if (ultimaRecta && avance >= DIST_FINAL_MM) { terminar("vuelta completa"); return; }
    if (!ultimaRecta && avance >= LADO_MM - RADIO_GIRO_MM) {
      fase = GIRO;
      distGiro0 = distanciaMm();
      thGiro0 = thActual();
      Log.printf("E,%lu,GIRO %d en x=%.0f y=%.0f lateral=%.0f\n", t, tramo + 1, x, y, lateral);
    }
  } else if (fase == GIRO) {
    int tope = SENTIDO_IZQ ? SERVO_TOPE_IZQ : SERVO_TOPE_DER;
    escribirServo(centroServo + (int)lroundf((tope - centroServo) * GIRO_FRACCION_TOPE));
    setMotor(PWM_GIRO);
    float falta = fabsf(rumboTramo(tramo + 1) - thActual());
    if (falta <= GIRO_ANTICIPO_DEG) {
      // Radio medido: arco recorrido / ángulo girado.
      float arco = distanciaMm() - distGiro0;
      float ang  = fabsf(thActual() - thGiro0) * DEG_TO_RAD;
      Log.printf("E,%lu,FIN GIRO %d radio_medido=%.0f mm (arco %.0f mm, %.1f grados)\n",
                 t, tramo + 1, ang > 0.1f ? arco / ang : 0.0f, arco, ang * RAD_TO_DEG);
      // La recta siguiente empieza en la esquina del cuadro (centro de pasillos).
      float r = rumboTramo(tramo) * DEG_TO_RAD;
      esqX += LADO_MM * cosf(r);
      esqY += LADO_MM * sinf(r);
      tramo++;
      fase = RECTA;
    }
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

  pinMode(XSHUT_TRAS, OUTPUT);
  digitalWrite(XSHUT_TRAS, LOW);
  for (int i = 0; i < N_TOF; i++) {
    pinMode(tofs[i].xshut, OUTPUT);
    digitalWrite(tofs[i].xshut, LOW);
  }
  delay(20);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  bool todosOk = true;
  for (int i = 0; i < N_TOF; i++) {
    tofs[i].ok = arrancarTof(tofs[i]);
    todosOk &= tofs[i].ok;
  }

  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N1, RVC_RX, -1);
  uint32_t t0 = millis();
  while (bnoPaquetes < 20 && millis() - t0 < 3000) leerBno();
  bool bnoOk = bnoPaquetes >= 20;

  // Se repite hasta que algo falle o llegue la 'g', para que se vea aunque el
  // BT se conecte después.
  while (true) {
    Log.printf("# ToF F:%s L:%s R:%s | BNO:%s | encoder:%s\n",
               tofs[0].ok ? "OK" : "NO", tofs[1].ok ? "OK" : "NO", tofs[2].ok ? "OK" : "NO",
               bnoOk ? "OK" : "NO", encOk ? "OK" : "NO");
    if (!todosOk || !bnoOk || !encOk) { Log.println("# NO ARRANCA: falta un sensor"); delay(1000); continue; }
    Log.println("# listo: manda 'g' para arrancar");
    break;
  }
}

void loop() {
  leerBno();
  actualizarPose();
  leerTofs();
  leerBno();

  // Comandos: 'g' y 'x' sueltos (sin Enter), o líneas terminadas en Enter:
  //   c <S|M|L> <roi> <budget_ms>   reconfigura los 3 ToF (solo en espera)
  //   r                             reinicia el resumen de "# ToF" (datos frescos)
  static char lin[32];
  static int nLin = 0;
  while (Serial.available() || SerialBT.available()) {
    char c = Serial.available() ? Serial.read() : SerialBT.read();
    if (nLin == 0 && c == 'g' && (fase == ESPERA || fase == FIN)) {
      if (fase == FIN) { fase = ESPERA; }
      arrancar();
    } else if (nLin == 0 && c == 'x' && (fase == RECTA || fase == GIRO)) {
      terminar("parado a mano");
    } else if (c == '\n' || c == '\r') {
      lin[nLin] = '\0';
      if (nLin > 0 && fase == ESPERA) comandoLinea(lin);
      nLin = 0;
    } else if (nLin < (int)sizeof(lin) - 1 && c != 'g' && c != 'x') {
      lin[nLin++] = c;
    }
  }

  if (fase == RECTA || fase == GIRO) {
    recorrido();
    // Un ToF que deja de mandar datos (cable flojo, se colgó) se avisa en el log:
    // el 2026-10-19 el derecho no mandó nada en toda una vuelta y no se notó.
    static unsigned long tAvisoTof = 0;
    if (millis() - tAvisoTof >= 1000) {
      tAvisoTof = millis();
      for (int i = 0; i < N_TOF; i++)
        if (millis() - tofs[i].msDato > 500)
          Log.printf("E,%lu,!! ToF %s sin datos hace %lu ms\n", millis() - tArranque,
                     tofs[i].nombre, millis() - tofs[i].msDato);
    }
    unsigned long t = millis() - tArranque;
    if (t - tPose >= POSE_MS) {
      tPose = t;
      Log.printf("P,%lu,%.0f,%.0f,%.2f,%d\n", t, x, y, thActual(), tramo);
    }
  } else if (fase == ESPERA) {
    static unsigned long tAviso = 0;
    // El encoder también se imprime: para revisar la escala, empuja el carro
    // a mano una distancia medida (p. ej. 2 m con cinta) y compara.
    if (millis() - tAviso >= 1000) {
      tAviso = millis();
      Log.printf("# listo: manda 'g' para arrancar | encoder %.0f mm\n", distanciaMm());
      imprimirResumenTof();
    }
  }
}
