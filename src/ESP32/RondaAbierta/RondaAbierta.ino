/*
 * RondaAbierta — la ronda abierta de PurePursuit.ino (rondaObstaculos = false)
 * en el carro nuevo, sin Raspberry ni botón:
 *   - Al prender espera ARRANQUE_MS (5 s) y arranca solo.
 *   - Paredes con 3 VL53L1X (frontal, izq, der) en lugar de los HC-SR04.
 *   - Rumbo con el BNO085 (UART-RVC) en lugar del MPU-6050.
 * La lógica es la misma de PurePursuit.ino:
 *   SIGUIENDO : antes del giro 1 solo gyro PID (rumbo 0); después, en cascada,
 *               la pared INTERIOR (a WALL_HOLD_CM) corre el rumbo objetivo hasta
 *               ±WALL_MAX_DESVIO_DEG y el gyro PID lo sigue.
 *               Frena a VEL_APROX_CERRADA con la pared de enfrente < FRONT_SLOWDOWN_CM.
 *   esquina   : un lateral > umbralPared con pared de enfrente < FRONT_ESQUINA_MAX,
 *               esquinaDebounce lecturas. Giro 1: además el forzado por frontal
 *               (FRONT_FORCE_GIRO_CM) hacia el lado con más hueco. La dirección
 *               de la pista se decide en el giro 1 y queda fija.
 *   GIRANDO   : servo a tope hasta quedar a GIRO_ANTICIPO_DEG del rumbo de la
 *               recta siguiente; el gyro PID termina el giro.
 *   Rumbo     : absoluto desde el arranque (el BNO085 casi no deriva). La recta n
 *               va a n x 90° exactos, así el error de un giro no se arrastra.
 *   TERMINANDO: tras el giro 12, TERMINANDO_MS más de recta y frena.
 * Los sensores se procesan a ritmo fijo (CONTROL_MS ~ un dato nuevo de ToF),
 * así los debounces cuentan lecturas, como con los ultrasónicos.
 * Librería: "VL53L1X" de Pololu. Monitor a 115200 (opcional).
 *
 * PINES:
 *   Driver TB6612: PWMA=23, AIN1=18, AIN2=19
 *   Servo:         GPIO 13 (0 = todo a la derecha, 156 = todo a la izquierda)
 *   ToF (VIN=3.3V, GND, SDA=21, SCL=22):
 *     Frontal XSHUT=0 -> 0x30 | Izquierdo XSHUT=15 -> 0x31 | Derecho XSHUT=5 -> 0x32
 *     Trasero XSHUT=4 se deja apagado.
 *   BNO085 (UART-RVC): VIN=3.3V, GND, PS0=3.3V, PS1=GND, SDA -> GPIO 25
 *
 * Convenciones (las de PurePursuit.ino): servo = centroServo + X, X > 0 =
 * izquierda; anguloGyro crece al girar a la IZQUIERDA. El yaw del BNO crece
 * al girar a la DERECHA (medido con RectanguloTest), por eso se le cambia el
 * signo (BNO_SIGNO).
 */

#include <Wire.h>
#include <VL53L1X.h>

// ── Carrera ───────────────────────────────────────────────────────────────────
const int           TURNS_PER_RACE = 12;      // 12 = 3 vueltas
const unsigned long ARRANQUE_MS    = 5000;    // espera tras prender antes de arrancar
const unsigned long TERMINANDO_MS  = 1000;    // recta tras el último giro hasta frenar

// ── Servo ─────────────────────────────────────────────────────────────────────
int       centroServo    = 75;    // medido con ServoCentro
const int SERVO_MIN      = 0;
const int SERVO_MAX      = 156;
const int SERVO_TOPE_DER = 0;     // giro a tope
const int SERVO_TOPE_IZQ = 156;

// ── Velocidades (PWM 0-255) — números del carro viejo, calibrar en pista ─────
const int MOTOR_MAX         = 255;   // rectas
const int VEL_INICIAL       = 180;   // antes del giro 1 (y todo el giro 1)
const int VEL_APROX_CERRADA = 120;   // pared de enfrente < FRONT_SLOWDOWN_CM
const int VEL_APROX2        = 90;    // giro 1: pared de enfrente < FRONT_SLOWDOWN2_CM
const int VEL_GIRO_INICIO   = 180;   // GIRANDO, |ángulo| < 45
const int VEL_GIRO_MEDIO    = 170;   // GIRANDO, |ángulo| < 70
const int VEL_GIRO_FINAL    = 190;   // GIRANDO, resto

// ── PID paredes ───────────────────────────────────────────────────────────────
// En CASCADA: la pared no mueve el servo directo, corre el rumbo objetivo como
// mucho WALL_MAX_DESVIO_DEG; el gyro PID es el único que mueve el servo.
// (Sumar las dos salidas dejaba que la pared le ganara al rumbo: con la pared
// interior lejos, el carro giraba en espiral hasta chocar.)
float KpWall = 0;                    // grados de rumbo por cm de error
float KdWall = 0;                    // grados de rumbo por (cm/s)
const float WALL_HOLD_CM        = 28.0;  // distancia a la pared INTERIOR tras el giro 1
const float WALL_MAX_DESVIO_DEG = 15.0;  // lo más que la pared corre el rumbo
const long  WALL_CONFIABLE_CM   = 55;    // pared interior más lejos que esto = no seguirla, solo rumbo
const long  WALL_HISTERESIS_CM  = 10;    // entra a seguirla bajo 45 cm, la suelta arriba de 65 cm
// El desvío de la pared cambia como mucho a este ritmo: al entrar/salir del modo
// "hold" (la pared aparece o desaparece) ya no da un brinco de golpe al rumbo
// objetivo, que el gyro PID convertía en un volantazo.
const float DESVIO_RAPIDEZ_DEG_S = 20.0;

// ── PID rumbo ─────────────────────────────────────────────────────────────────
float KpGyro = 1.0;
float KiGyro = 0.0;
float KdGyro = 0.0;
// PWM al que se afinan KpGyro/KdGyro en ESTE carro (el 255 del motor nuevo va
// como el 180 del viejo). A menos PWM las ganancias suben en proporción, hasta 1.2x.
const int PWM_REF_PID = 255;

// ── Esquinas ──────────────────────────────────────────────────────────────────
const int umbralPared         = 100;  // cm: lateral más lejos que esto = se abrió la esquina
const int esquinaDebounce     = 2;    // lecturas
const int FRONT_ESQUINA_MAX   = 100;  // la esquina solo cuenta con pared de enfrente < esto
                                      // (era el tope del HC-SR04; el ToF ve más lejos,
                                      // así que se puede subir si entra tarde en esquinas anchas)
const int FRONT_SLOWDOWN_CM   = 90;
const int FRONT_SLOWDOWN2_CM  = 60;   // solo giro 1
const int FRONT_FORCE_GIRO_CM = 40;   // solo giro 1: forzar el giro hacia el lado más abierto
const int FORZADO_DEBOUNCE    = 3;    // lecturas seguidas (reset duro)
// El servo vuelve a recto a GIRO_ANTICIPO_DEG del rumbo de la recta siguiente:
// lo que falta lo completan la inercia y el gyro PID (que apunta a n x 90° exactos).
// Si el giro se pasa de 90, súbelo; si queda corto y el PID tarda en completarlo, bájalo.
const int GIRO_ANTICIPO_DEG   = 5;
// Salida suave: en los últimos GIRO_SUAVE_DEG del giro el servo baja del tope
// hacia el centro en proporción a lo que falta, sin bajar de GIRO_SUAVE_MIN del
// tope. 0 = como antes (tope hasta el final). Si los giros quedan abiertos o el
// carro se acerca mucho a la pared exterior al salir, baja GIRO_SUAVE_DEG.
const float GIRO_SUAVE_DEG    = 25;
const float GIRO_SUAVE_MIN    = 0.3f;
const unsigned long cooldownGiro = 1000;
const unsigned long GRACIA_ARRANQUE_MS = 500;

// ── ToF ───────────────────────────────────────────────────────────────────────
// 50 ms como TofTest4 (que lee bien la pared negra). 33 ms es el mínimo del
// modo Long y en pared oscura falla muchas más lecturas.
const uint32_t TOF_BUDGET_US  = 50000;   // 50 ms por medición -> ~20 Hz
// Tamaño de la zona de visión (ROI) en SPADs, 4-16. 16 = cono completo (~27°).
// Más chico = cono más angosto: menos riesgo de que los laterales vean el piso
// cuando se acaba la pared (con 16 leían ~60 cm en vez de "sin pared").
const uint8_t  TOF_ROI        = 8;
const int      TOF_PERIODO_MS = 50;
const unsigned long CONTROL_MS = 50;     // ritmo de filtros + lógica (un dato de ToF nuevo)
const long     SIN_PARED_CM   = 200;     // lectura inválida / muy lejos = "sin pared" (como el HC-SR04)
// Solo el FRONTAL: una lectura inválida no se vuelve "sin pared" de inmediato.
// En pared negra, o pegándole en ángulo, el ToF falla lecturas sueltas y la
// esquina (que pide pared de enfrente < FRONT_ESQUINA_MAX) no se contaba.
// Los laterales no lo usan: su "sin pared" ES la apertura de la esquina.
const unsigned long FRONT_HOLD_MS = 300;
float alpha = 0.85;                      // EMA de las distancias (igual que antes)

// ── BNO ───────────────────────────────────────────────────────────────────────
const int BNO_SIGNO = -1;   // yaw del BNO crece a la derecha; anguloGyro crece a la izquierda

// ── Pines ─────────────────────────────────────────────────────────────────────
const int PWMA = 23;
const int A1   = 18;
const int A2   = 19;
const int freqMotor = 1000;
const int resMotor  = 8;
const int SERVO_PIN = 13;
const int freqServo = 50;
const int resServo  = 16;
const int RVC_RX    = 25;
const int XSHUT_TRAS = 4;

// ═══════════════════════════════════════════════════════════════════════════════
// Tipos y estado (antes de cualquier función: el IDE declara las funciones arriba)
// ═══════════════════════════════════════════════════════════════════════════════

struct Tof {
  const char* nombre;
  int         xshut;
  uint8_t     dir;
  VL53L1X     s;
  bool        ok;
  long        cm;        // última lectura (SIN_PARED_CM si inválida)
  unsigned long msValida;  // millis() de la última lectura válida
};
Tof tofs[] = {
  {"FREN", 0,  0x30},
  {"IZQ",  15, 0x31},
  {"DER",  5,  0x32},
};
const int N_TOF = sizeof(tofs) / sizeof(tofs[0]);
Tof& tofF = tofs[0];
Tof& tofL = tofs[1];
Tof& tofR = tofs[2];

enum Estado { ESPERANDO, SIGUIENDO, GIRANDO, TERMINANDO, TERMINADO };
Estado estado = ESPERANDO;

// Rumbo
float anguloGyro     = 0;   // °, acumulado desde el arranque (sin saltos de ±180), crece a la izquierda
float anguloObjetivo = 0;   // rumbo de la recta actual: rumboRecta(turnsCompleted)
float yawPrev        = 0;
bool  yawInicial     = false;
uint32_t bnoPaquetes = 0;
unsigned long bnoUltimoMs = 0;

// PIDs
float errorWall = 0, prevErrorWall = 0;
bool  wallPrevValido = false;   // hay un errorWall anterior para la derivada
float desvio = 0;               // °, lo que la pared corre el rumbo objetivo (+ = izquierda)
float errorGyro = 0, integralGyro = 0;
unsigned long lastPIDTime = 0;
// Tasa de giro (°/s, + = izquierda), del BNO a ~100 Hz. La D del gyro PID usa
// esta tasa y no la derivada del error: así un cambio del objetivo no da patada.
float tasaGiro = 0;
bool  bnoNuevo = false;
unsigned long bnoPrevMs = 0;

// Distancias filtradas
float distL_filtrada = SIN_PARED_CM, distR_filtrada = SIN_PARED_CM, distF_filtrada = SIN_PARED_CM;

// Carrera
int  turnsCompleted     = 0;
bool direccionIzquierda = true;
bool primerGiro         = false;
int  contadorEsquina    = 0;
int  contadorForzado    = 0;
int  velocidadMotor     = 0;
int  servoActual        = -1;
unsigned long timeStart = 0, lastTurnTime = 0, terminandoEntryMs = 0;
unsigned long tControl  = 0, tLog = 0;
const char* modoServo = "";

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

void setMotor(int velocidad) {
  ledcWrite(PWMA, constrain(velocidad, 0, MOTOR_MAX));
}

void frenar() {
  digitalWrite(A1, HIGH); digitalWrite(A2, HIGH);   // freno corto del TB6612
  ledcWrite(PWMA, 255);
  delay(300);
  ledcWrite(PWMA, 0);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Sensores
// ═══════════════════════════════════════════════════════════════════════════════

float normaliza(float a) {
  while (a >  180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

// BNO085 en UART-RVC: paquetes de 19 bytes a ~100 Hz. Acumula el cambio de yaw
// en anguloGyro (sin el salto de +180 a -180).
uint8_t bnoBuf[19];
int     bnoN = 0;

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
    unsigned long ahora = millis();
    if (yawInicial) {
      float d = BNO_SIGNO * normaliza(yaw - yawPrev);
      anguloGyro += d;
      float dt = (ahora - bnoPrevMs) / 1000.0f;
      if (dt > 0.002f && dt < 0.1f) tasaGiro += (d / dt - tasaGiro) * 0.5f;   // EMA ligera
    }
    yawPrev = yaw;
    yawInicial = true;
    bnoPrevMs = ahora;
    bnoPaquetes++;
    bnoUltimoMs = ahora;
    bnoNuevo = true;
  }
}

bool arrancarTof(Tof& t) {
  digitalWrite(t.xshut, HIGH);
  delay(10);
  t.s.setTimeout(500);
  if (!t.s.init()) return false;            // nace en 0x29
  t.s.setAddress(t.dir);
  t.s.setDistanceMode(VL53L1X::Long);
  t.s.setROISize(TOF_ROI, TOF_ROI);
  t.s.setMeasurementTimingBudget(TOF_BUDGET_US);
  t.s.startContinuous(TOF_PERIODO_MS);
  t.cm = SIN_PARED_CM;
  t.msValida = 0;
  return true;
}

// ¿Sirve la lectura? WrapTargetFail = el sensor no pudo confirmar la distancia
// y puede ser un "fantasma" de algo lejano doblado a una distancia corta: sin
// pared al lado, los laterales marcaban ~560 mm con "wrap target fail". Si una
// pared REAL (negra, a 30-80 cm) también sale casi siempre con wrap fail en
// TofTest4, pon ACEPTAR_WRAP = true (pero entonces la esquina abierta puede
// verse como pared a ~56 cm).
const bool ACEPTAR_WRAP = false;
// SignalFail = regresó poca luz. Con ROI 8, una pared negra REAL a ~75 cm sale
// con signal fail pero con la distancia correcta, y sin pared sale ~160-190 cm.
// Se acepta solo hasta SIGNAL_FAIL_MAX_CM: más cerca es pared, más lejos es
// "sin pared". Queda debajo de umbralPared para no inventar paredes en la esquina.
const long SIGNAL_FAIL_MAX_CM = 90;

bool estadoUsable(uint8_t st, long cm) {
  if (st == VL53L1X::WrapTargetFail) return ACEPTAR_WRAP;
  if (st == VL53L1X::SignalFail)     return cm <= SIGNAL_FAIL_MAX_CM;
  return st == VL53L1X::RangeValid
      || st == VL53L1X::RangeValidMinRangeClipped;
}

// Lee los ToF que tengan dato nuevo. Inválida o > SIN_PARED_CM = "sin pared",
// igual que el HC-SR04 sin eco (la lógica de esquinas cuenta con ese 200).
// Excepción: el frontal conserva su última lectura válida FRONT_HOLD_MS.
void leerTofs() {
  for (int i = 0; i < N_TOF; i++) {
    Tof& t = tofs[i];
    if (!t.ok || !t.s.dataReady()) continue;
    t.s.read(false);
    long cm = t.s.ranging_data.range_mm / 10;
    bool valida = estadoUsable(t.s.ranging_data.range_status, cm)
                  && cm > 0 && cm <= SIN_PARED_CM;
    if (valida) {
      t.cm = cm;
      t.msValida = millis();
    } else if (&t != &tofF || millis() - t.msValida > FRONT_HOLD_MS) {
      t.cm = SIN_PARED_CM;
    }
  }
}

// Rumbo de la recta después de n giros: n x 90° hacia el lado de la pista.
// Antes del giro 1 no hay lado todavía y la recta es 0°.
float rumboRecta(int n) {
  if (!primerGiro) return 0;
  return (direccionIzquierda ? 90.0f : -90.0f) * n;
}

float filtroEMA(float nueva, float anterior) {
  return alpha * nueva + (1.0 - alpha) * anterior;
}

bool detectarEsquina(long distL, long distR, long distF) {
  bool paredFrente = (distF > 0 && distF < FRONT_ESQUINA_MAX);
  bool apertura = paredFrente && ((distL > umbralPared) || (distR > umbralPared));
  if (apertura) contadorEsquina = min(contadorEsquina + 1, (int)esquinaDebounce);
  else          contadorEsquina = max(contadorEsquina - 1, 0);
  return contadorEsquina >= esquinaDebounce;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PID: pared (desvío del rumbo) + gyro PID (servo), en cascada
// ═══════════════════════════════════════════════════════════════════════════════

// Pared -> desvío del rumbo objetivo. Se llama a ritmo de ToF (CONTROL_MS).
void actualizarDesvio(long distL, long distR) {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;
  if (dt < 0.01) dt = 0.01;

  // Antes del giro 1 no hay lado interior: solo rumbo.
  // Lateral ya abierto (esquina por venir) o pared interior lejana (hueco, se
  // acabó, o el ToF ve el piso): no se sigue la pared, se mantiene el rumbo.
  bool esquinaInminente = (distL > umbralPared) || (distR > umbralPared);
  long distInt = direccionIzquierda ? distL : distR;
  float desvioObj = 0;

  // Histéresis: con la pared interior justo en el límite, el modo brincaba entre
  // "hold" y "lejos" cada pocas lecturas y el desvío iba y venía entre ~8° y 0:
  // el carro serpenteaba solo en las vueltas en que quedaba a esa distancia.
  static bool enHold = false;
  if (esquinaInminente || distInt > WALL_CONFIABLE_CM + WALL_HISTERESIS_CM) enHold = false;
  else if (distInt < WALL_CONFIABLE_CM - WALL_HISTERESIS_CM)              enHold = true;

  if (!primerGiro) {
    wallPrevValido = false;
    enHold = false;
    modoServo = "rumbo";
  } else if (!enHold) {
    wallPrevValido = false;
    modoServo = esquinaInminente ? "esq" : "lejos";
  } else {
    // errorWall > 0 = lejos de la pared interior -> acercarse (girar hacia ella).
    errorWall = distInt - WALL_HOLD_CM;
    // Sin derivada en la primera lectura tras volver a ver la pared (era una patada).
    float deriv = wallPrevValido ? (errorWall - prevErrorWall) / dt : 0;
    prevErrorWall  = errorWall;
    wallPrevValido = true;
    float lado = direccionIzquierda ? 1.0f : -1.0f;   // pared interior a la izquierda = girar a la izquierda
    desvioObj = lado * (KpWall * errorWall + KdWall * deriv);
    desvioObj = constrain(desvioObj, -WALL_MAX_DESVIO_DEG, WALL_MAX_DESVIO_DEG);
    modoServo = "hold";
  }

  // Rampa: el desvío se acerca a desvioObj a lo más DESVIO_RAPIDEZ_DEG_S.
  float paso = DESVIO_RAPIDEZ_DEG_S * dt;
  desvio += constrain(desvioObj - desvio, -paso, paso);
}

// Gyro PID -> servo. Se llama con cada paquete nuevo del BNO (~100 Hz), no a
// ritmo de ToF: a 20 Hz corregía tarde y el carro serpenteaba.
// Sin re-referencia del rumbo objetivo: con el BNO el objetivo se queda en
// n x 90° exactos y la pared solo lo corre ±WALL_MAX_DESVIO_DEG.
void pidRumbo() {
  static unsigned long tAnt = 0;
  unsigned long now = millis();
  float dt = (now - tAnt) / 1000.0f;
  tAnt = now;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;

  errorGyro = (anguloObjetivo + desvio) - anguloGyro;
  errorGyro = constrain(errorGyro, -20, 20);
  integralGyro += errorGyro * dt;
  integralGyro  = constrain(integralGyro, -30, 30);
  // D sobre la tasa de giro medida (error = objetivo - ángulo -> d(error)/dt = -tasa).
  // A más velocidad, el mismo movimiento de servo gira más rápido al carro: las
  // ganancias se escalan para que el lazo se comporte igual que a PWM_REF_PID.
  float escala = constrain((float)PWM_REF_PID / max(velocidadMotor, 1), 0.5f, 1.2f);
  float outputGyro = escala * (KpGyro * errorGyro - KdGyro * tasaGiro) + KiGyro * integralGyro;

  float outputFinal = constrain(outputGyro, -25, 25);
  escribirServo(centroServo + (int)outputFinal);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Estados
// ═══════════════════════════════════════════════════════════════════════════════

void arrancarCarrera() {
  anguloGyro     = 0;
  anguloObjetivo = 0;
  wallPrevValido = false; desvio = 0;
  integralGyro = 0;
  timeStart    = millis();
  lastTurnTime = millis();
  lastPIDTime  = millis();
  digitalWrite(A1, HIGH); digitalWrite(A2, LOW);
  estado = SIGUIENDO;
  Serial.println("-> ARRANCA");
}

void siguiendo(long distL, long distR, long distF) {
  velocidadMotor = (turnsCompleted == 0) ? VEL_INICIAL : MOTOR_MAX;
  if (distF > 0 && distF < FRONT_SLOWDOWN_CM)
    velocidadMotor = min(velocidadMotor, VEL_APROX_CERRADA);
  if (turnsCompleted == 0 && distF > 0 && distF < FRONT_SLOWDOWN2_CM)
    velocidadMotor = min(velocidadMotor, VEL_APROX2);

  actualizarDesvio(distL, distR);
  setMotor(velocidadMotor);

  if (millis() - lastTurnTime <= cooldownGiro || millis() - timeStart <= GRACIA_ARRANQUE_MS) return;

  bool giroNormal  = detectarEsquina(distL, distR, distF);
  bool forzadoCond = (turnsCompleted == 0 && !primerGiro && distF > 0 && distF < FRONT_FORCE_GIRO_CM);
  contadorForzado  = forzadoCond ? min(contadorForzado + 1, FORZADO_DEBOUNCE) : 0;
  bool giroForzado = (contadorForzado >= FORZADO_DEBOUNCE);

  if (giroNormal || giroForzado) {
    estado = GIRANDO;
    if (!primerGiro) {
      direccionIzquierda = (distL > distR);
      primerGiro         = true;
    }
    if (giroForzado && !giroNormal) Serial.println("Giro 1 FORZADO (pared de enfrente)");
    Serial.printf("Giro %d %s  (L:%ld R:%ld F:%ld)\n", turnsCompleted + 1,
                  direccionIzquierda ? "izquierda" : "derecha", distL, distR, distF);
  }
}

// Se llama en cada vuelta del loop (no a ritmo de ToF): el ángulo llega a 100 Hz
// y cortar el giro tarde se paga en grados.
// delta = cuánto lleva girado respecto al rumbo de la recta que deja. Se mide
// contra n x 90° (no contra el rumbo real al empezar), así un giro anterior
// corto o largo se corrige en este.
void girando() {
  float signo = direccionIzquierda ? 1.0f : -1.0f;
  float delta = signo * (anguloGyro - rumboRecta(turnsCompleted));

  if (turnsCompleted == 0)  velocidadMotor = VEL_INICIAL;
  else if (delta < 45)      velocidadMotor = VEL_GIRO_INICIO;
  else if (delta < 70)      velocidadMotor = VEL_GIRO_MEDIO;
  else                      velocidadMotor = VEL_GIRO_FINAL;

  setMotor(velocidadMotor);

  // Servo a tope, y en los últimos GIRO_SUAVE_DEG regresa poco a poco al centro
  // (sin bajar de GIRO_SUAVE_MIN del tope). Así el carro llega al final del giro
  // casi sin girar y el gyro PID arranca tranquilo: salir del tope de golpe a
  // ~90°/s metía al PID en un baile que duraba toda la recta.
  int   tope     = direccionIzquierda ? SERVO_TOPE_IZQ : SERVO_TOPE_DER;
  float restante = 90 - delta;
  float fraccion = 1.0f;
  if (GIRO_SUAVE_DEG > 0 && restante < GIRO_SUAVE_DEG)
    fraccion = max(restante / GIRO_SUAVE_DEG, GIRO_SUAVE_MIN);
  escribirServo(centroServo + (int)lroundf((tope - centroServo) * fraccion));

  if (delta >= 90 - GIRO_ANTICIPO_DEG) {
    escribirServo(centroServo);
    wallPrevValido = false; desvio = 0;
    integralGyro = 0;
    lastTurnTime   = millis();
    turnsCompleted++;
    anguloObjetivo = rumboRecta(turnsCompleted);
    Serial.printf("Giro completado %d/%d\n", turnsCompleted, TURNS_PER_RACE);
    if (turnsCompleted >= TURNS_PER_RACE) {
      estado = TERMINANDO;
      terminandoEntryMs = millis();
      Serial.printf("-> TERMINANDO (%lu ms)\n", TERMINANDO_MS);
    } else {
      estado = SIGUIENDO;
    }
  }
}

void terminando(long distL, long distR) {
  velocidadMotor = MOTOR_MAX;
  actualizarDesvio(distL, distR);
  setMotor(velocidadMotor);
  if (millis() - terminandoEntryMs >= TERMINANDO_MS) {
    frenar();
    escribirServo(centroServo);
    estado = TERMINADO;
    Serial.printf("TERMINADO: %d giros en %.1f s\n", turnsCompleted, (millis() - timeStart) / 1000.0f);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Setup / loop
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);
  ledcAttach(PWMA, freqMotor, resMotor);
  ledcWrite(PWMA, 0);
  ledcAttach(SERVO_PIN, freqServo, resServo);
  escribirServo(centroServo);

  // ToF: todos apagados con XSHUT y se prenden uno por uno con su dirección.
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
    Serial.printf("ToF %-4s 0x%02X : %s\n", tofs[i].nombre, tofs[i].dir, tofs[i].ok ? "OK" : "NO RESPONDE");
  }

  // BNO al final: arrancar los ToF tarda y el buffer se llenaría.
  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N1, RVC_RX, -1);
  uint32_t t0 = millis();
  while (bnoPaquetes < 20 && millis() - t0 < 3000) leerBno();
  bool bnoOk = bnoPaquetes >= 20;
  Serial.println(bnoOk ? "BNO OK" : "BNO NO responde: revisa VIN, GND, PS0=3.3V, PS1=GND y SDA -> GPIO 25");

  // Sin los 3 ToF o sin BNO no se arranca: el carro se quedaría sin paredes o sin rumbo.
  if (!todosOk || !bnoOk) {
    Serial.println("NO ARRANCA: falta un sensor");
    while (true) delay(1000);
  }

  Serial.printf("Arranca en %lu s...\n", ARRANQUE_MS / 1000);
  timeStart = millis();
}

void loop() {
  leerBno();
  leerTofs();
  leerBno();   // otra vez: que el I2C de los ToF no deje acumular paquetes del BNO

  if (estado == TERMINADO) return;

  // El giro se revisa en cada vuelta (ángulo a 100 Hz) y el gyro PID corre con
  // cada dato nuevo del BNO.
  if (estado == GIRANDO) girando();
  if (bnoNuevo) {
    bnoNuevo = false;
    if (estado == SIGUIENDO || estado == TERMINANDO) pidRumbo();
  }

  unsigned long ahora = millis();
  if (ahora - tControl < CONTROL_MS) return;
  tControl = ahora;

  // Filtros a ritmo fijo, como el loop de los ultrasónicos (también durante el
  // giro, para salir de él con distancias frescas).
  distL_filtrada = filtroEMA(tofL.cm, distL_filtrada);
  distR_filtrada = filtroEMA(tofR.cm, distR_filtrada);
  distF_filtrada = filtroEMA(tofF.cm, distF_filtrada);
  long distL = (long)distL_filtrada;
  long distR = (long)distR_filtrada;
  long distF = (long)distF_filtrada;

  if (ahora - bnoUltimoMs > 200) Serial.println("!! BNO sin datos > 200 ms");

  switch (estado) {
    case ESPERANDO:
      if (ahora - timeStart >= ARRANQUE_MS) arrancarCarrera();
      break;
    case SIGUIENDO:  siguiendo(distL, distR, distF); break;
    case TERMINANDO: terminando(distL, distR);        break;
    default: break;
  }

  if (ahora - tLog >= 100) {
    tLog = ahora;
    const char* nombre = estado == ESPERANDO ? "ESPERANDO" : estado == SIGUIENDO ? "SIGUIENDO"
                       : estado == GIRANDO ? "GIRANDO" : estado == TERMINANDO ? "TERMINANDO" : "TERMINADO";
    Serial.printf("%-10s L:%3ld R:%3ld F:%3ld | ang:%7.1f obj:%7.1f des:%5.1f tasa:%6.1f | servo:%3d (%s) pwm:%3d | giros:%d/%d %s\n",
                  nombre, distL, distR, distF, anguloGyro, anguloObjetivo, desvio, tasaGiro, servoActual, modoServo,
                  velocidadMotor, turnsCompleted, TURNS_PER_RACE,
                  primerGiro ? (direccionIzquierda ? "IZQ" : "DER") : "");
  }
}
