/*
 * PurePursuit.ino — WRO Future Engineers
 * Firmware ESP32 con soporte de protocolo V2 (Pure Pursuit desde Raspberry Pi).
 *
 * Basado en Controller_PI.ino. Diferencias respecto a la versión anterior:
 *   1. Parseo del campo pp= en mensajes V2.
 *   2. Cuando piPurePursuit=true:
 *        - Se suspende el PID de paredes (ultrasonidos).
 *        - La Pi envía obs=steer_deg/35 → ESP32 aplica ppSteerGain=35.
 *        - Solo se mantiene una corrección liviana de gyro para estabilizar heading.
 *   3. Cuando piPurePursuit=false (V1 o pp=0):
 *        - Comportamiento idéntico a Controller_PI.ino.
 *   4. Timeout de Pi (>800 ms sin mensaje):
 *        - Fallback a wall PID + gyro (igual que antes).
 *
 * PINES (sin cambios):
 *   HC-SR04 izq  : TRIG=27, ECHO=32
 *   HC-SR04 der  : TRIG=26, ECHO=35
 *   Motor DC     : PWMA=23, A1=18, A2=19  (TB6612FNG)
 *   Servo SG90   : SERVO_PIN=13
 *   MPU-6050     : I2C (SDA/SCL por defecto)
 *   Serial Pi    : Serial2 RX=17, TX=16 @ 115200
 */

#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu(Wire);

// ── Pines ─────────────────────────────────────────────────────────────────────
#define TRIG_L      27
#define ECHO_L      32
#define TRIG_R      26
#define ECHO_R      35

#define PWMA        23
#define A1          18
#define A2          19
#define SERVO_PIN   13
#define TRIG_F      14   // HC-SR04 frontal (ronda de obstáculos: CRUCERO/MANIOBRA)
#define ECHO_F      33

// ── RONDA OBSTACULOS ──────────────────────────────────────────────────────────
const bool rondaObstaculos  = false;    // false = giro continuo de siempre (ronda abierta)


// ── PWM ───────────────────────────────────────────────────────────────────────
const int freqServo  = 50;
const int resServo   = 16;
const int freqMotor  = 1000;
const int resMotor   = 8;

// ── PID Paredes (ultrasónicos) ────────────────────────────────────────────────
float KpWall = 1.0;
float KiWall = 0.0;
float KdWall = 1.2;

// Ronda ABIERTA: en vez de centrar entre paredes (distL-distR -> 0), una vez que
// ya se sabe el sentido de giro de la pista, el wall PID mantiene esta distancia
// fija a la pared INTERIOR (la del lado hacia donde gira). Menos "hunting" y
// línea más corta en rectas anchas; además la pared interior no "desaparece" en
// las esquinas (esa es la exterior), así que el PID no se clava. 
const float WALL_HOLD_CM = 25.0;

// El error de UNA sola pared tiene ~la mitad de ganancia geométrica que
// distL-distR (al desplazarte lateralmente solo cambia un sensor, no dos). Sin
// re-escalar, el término de pared no le gana al gyro PID —que tras cada giro
// continuo defiende un heading viciado (~13°, el giro se pasa por inercia)— y el
// carro se ABRE hacia la pared de afuera en vez de pegarse a la de adentro.
const float WALL_HOLD_GAIN = 1.8;

float errorWall    = 0;
float prevErrorWall = 0;
float integralWall  = 0;

// ── PID Gyro ──────────────────────────────────────────────────────────────────
float KpGyro = 2.0;
float KiGyro = 0.0;
float KdGyro = 0.5;
float gyroScale = 1;

float errorGyro     = 0;
float prevErrorGyro = 0;
float integralGyro  = 0;

// ── Tiempo ────────────────────────────────────────────────────────────────────
unsigned long lastPIDTime  = 0;
unsigned long lastGyroTime = 0;

// ── Giroscopio ────────────────────────────────────────────────────────────────
float anguloGyro    = 0;
float anguloObjetivo = 0;

// ── Control ───────────────────────────────────────────────────────────────────
int velocidadMotor = 180;
int centroServo    = 80;

// ── Integración Pi → ESP32 ───────────────────────────────────────────────────
float obsBiasNorm  = 0.0;     // obs  [-1, 1] del mensaje V2
int   turnHint     = 0;       // turn {-1, 0, +1}
bool  piPriority   = false;   // prio=1: obstáculo activo en Pi
int   piMemoryFrames = 0;     // mem=N: frames de memoria restantes
bool  piPurePursuit = false;  // pp=1: Pi en modo Pure Pursuit
bool  piPasado     = false;   // pasado=1: la Pi confirma que un obstáculo quedó
                               // FÍSICAMENTE detrás del robot este frame (no que
                               // simplemente dejó de verlo) — dispara RECUPERANDO.
bool  piInteriorPass = false; // intr=1: el obstáculo actual se pasa por el mismo
                               // lado hacia el que va a girar la pista (la Pi ya
                               // sabe la dirección de giro, por visión, y el color
                               // del obstáculo) — el giro mismo resuelve el paso,
                               // no hace falta seguir bloqueando detectarEsquina().
bool  piReady      = false;

unsigned long lastPiMsgMs = 0;
const unsigned long piTimeoutMs = 800;  // ms sin mensaje → fallback

// Ganancia de visión V1 (modo fallback / obstáculo)
float visionSteerGain = 80.0;
float turnHintGain    = 7.0;

// Ganancia Pure Pursuit: obs = steer_deg / 60 → steerDeg = obs * 60 = steer_deg
const float ppSteerGain = 60.0;

// Cuánto se deflecta el servo por cada grado de PP.  steerDeg sale de la
// geometría (máx ±35°) y suele quedar corto para la mecánica del servo:
// súbelo si el carrito gira poco, bájalo si oscila/sobregira.
float ppServoGain = 1.0;
float PP_GYRO_BLEND = 0.12;  // 0 = solo vision, 1 = solo gyro. BAJO a propósito: en el modo
                            // por tramos el heading de referencia queda viciado tras cada
                            // MANIOBRA (no gira exacto 90°), y un blend alto "defiende" ese
                            // heading chueco -> el carro deriva lap a lap. Visión + wall PID
                            // re-referencian al carril/paredes reales y no tienen ese problema.
float PP_WALL_BLEND = 0.30;  // SUBIDO: el wall PID (distL-distR) centra en el carril y
                            // ayuda a des-enchuecar. Solo aplica en recta limpia
                            // (!piPriority && mem<=0), así que no choca con la esquina.

// ── Boot sincronización con Pi ────────────────────────────────────────────────
bool piReadyReceived = false;
// READY ya llegó PERO todavía no el primer V2. Entre esos dos, controlPID()
// caía en el fallback wall-PID y setMotor(velocidadMotor) -> el carro rodaba
// ~0.5 s hacia adelante sin ver el obstáculo. Con este gate el carro NO rueda
// hasta el primer V2 real de Pure Pursuit.
bool piFirstV2Received = false;
unsigned long readyMs = 0;                          // millis() cuando llegó READY
const unsigned long FIRST_V2_TIMEOUT_MS = 3000;     // si tras READY nunca llega V2, seguir igual
unsigned long bootStartMs = 0;
const unsigned long BOOT_WAIT_PI_MS = 1000;

// One-shot: la primera vez que el carro va a rodar de verdad (tras pasar el gate
// de WAIT_FIRST_V2) se re-anclan lastTurnTime y timeStart a ESE instante. Sin
// esto ambos quedaban fijados en el READY (varios segundos antes), así que el
// cooldownGiro y el gate de arranque ya estaban vencidos al primer frame de
// marcha y una lectura ancha de la zona de salida podía disparar el giro 1.
bool marchaIniciada = false;

// ── FSM estados ───────────────────────────────────────────────────────────────
// RECUPERANDO: la Pi confirma (piPasado=1) que el robot YA atravesó
// físicamente un obstáculo — no que la cámara simplemente dejó de verlo
// (perder de vista ≠ haber rebasado). En vez de que Pure Pursuit intente
// enderezar solo con lo que ve en ese instante incierto, aquí el wall PID +
// gyro PID (que YA se calculan siempre, ver controlPID()) toman el volante
// hasta que el robot vuelve a estar centrado/alineado.
// CRUCERO / MANIOBRA: solo ronda de obstáculos (rondaObstaculos=true). En lugar
// del giro continuo (GIRANDO), al llegar a una esquina el carro va DERECHO por
// ángulo (CRUCERO) hasta ~50cm de la pared y luego hace una maniobra por tramos
// (MANIOBRA): pivote hacia adelante o EN REVERSA según qué tan pegado va a la
// pared exterior del giro. Con rondaObstaculos=false nada de esto se usa.
enum Estado { SIGUIENDO, RECUPERANDO, GIRANDO, CRUCERO, MANIOBRA, TERMINANDO };
Estado estado = SIGUIENDO;

// ── Giro por tramos (ronda de obstáculos) ────────────────────────────────────
const int  FRONT_TURN_FWD_CM = 70;     // CRUCERO -> MANIOBRA si la maniobra será FORWARD
                                       // (el arco necesita espacio adelante)
const int  FRONT_TURN_REV_CM = 30;     // ... si será REVERSE (hay que estar cerca de la pared
                                       // para que el pivote en reversa no sobrepase)
const int  FRONT_CRUCERO_CM = 80;      // SIGUIENDO -> CRUCERO (recta ya limpia, esquina cerca)
const int  CRUCERO_GYRO_CM  = 80;      // dentro de CRUCERO: > esto -> visión (centerline recto);
                                       // <= esto -> pura gyro + wall PID (el centerline ya
                                       // curva para "esquivar" la pared del fondo y enchueca)
const int  MANIOBRA_OVERSHOOT_DEG = 12; // sale del pivote a (AngGiro - esto): el carro sigue
                                        // rotando por inercia y sin esto la recta nueva
                                        // arrancaba ~10-15° chueca (orillas460)
const int  HUG_CM           = 20;      // pared exterior <= esto -> FORWARD (no cabe reversear)
const int  MANIOBRA_VEL_REV  = 100;     // PWM objetivo del motor en la reversa-pivote
const int  MANIOBRA_VEL_MIN  = 80;     // PWM de arranque de la rampa (evita el golpe de corriente)
const float CRUCERO_WALL_BLEND = 0.8f; // en CRUCERO: cuánto del wall PID se mezcla para CENTRAR
                                       // en el carril (0 = solo heading, 1 = wall PID completo).
                                       // Solo aplica mientras ambas paredes existen.
const unsigned long MANIOBRA_FRENO_MS      = 300;   // coast (A1=A2=LOW) antes/después de invertir dirección
                                                    // — SIN esto el puente H se fríe por "plugging" (invertir
                                                    // con el motor girando). Reventó un TB6612 así (2026-09-01).
const unsigned long MANIOBRA_RAMP_MS       = 60;    // subir el PWM de reversa de a poco. Corto a
                                                    // propósito: el motor viene PARADO (coast de
                                                    // fase 0), así que arrancar a MANIOBRA_VEL_REV
                                                    // es un inrush normal, no "plugging". Puedes
                                                    // bajarlo más o dejarlo en 0.
const unsigned long MANIOBRA_REV_TIMEOUT_MS = 6000; // reversa no llegó a 88° -> frena y termina de frente
const unsigned long CRUCERO_TIMEOUT_MS      = 7000; // en CRUCERO tanto sin llegar a la pared -> MANIOBRA igual (red de seguridad anti-atasco)
const unsigned long MANIOBRA_BACKOFF_MS     = 1000;  // DESPUÉS de completar el pivote: retrocede este
                                                    // tiempo para tomar distancia de la recta nueva.
                                                    // NO toca la geometría del pivote (ya calibrada).
const int           MANIOBRA_BACKOFF_VEL    = 100;  // PWM del retroceso
const int           MANIOBRA_BACKOFF_MIN_CM = 50;   // SOLO retrocede si la pared exterior del giro
                                                    // (la que sigues) está a MÁS de esto. Si vas
                                                    // pegado a ella, retroceder recto no ayuda.
unsigned long cruceroEntryMs = 0;
bool cruceroCerca      = false;        // en CRUCERO: true = cerca de la pared -> pura gyro+wall (sin visión)
int  contadorFront     = 0;            // debounce del sensor frontal
bool maniobraDecidida  = false;
bool maniobraGirarDer  = false;
bool maniobraReversa   = false;
bool maniobraRetroceso = false;        // true = hay espacio (pared exterior > MANIOBRA_BACKOFF_MIN_CM) -> retrocede un poco DESPUÉS de la maniobra
long maniobraDistExt   = 0;
int  maniobraFase      = -1;           // -1=sin init  0=frenar-antes  1=pivote  2=frenar-después  3=frenar-y-reintentar-fwd  4=retroceso-post  5=frenar-tras-retroceso
unsigned long maniobraFaseMs   = 0;    // inicio de la fase actual (para las pausas de freno)
unsigned long maniobraPivoteMs = 0;    // inicio del pivote (para la rampa y el timeout de reversa)

const float wallSettleCm    = 8.0;   // |distL-distR| por debajo de esto = "centrado"
const float headingSettleDeg = 8.0;  // |errorGyro| por debajo de esto = "alineado"

// Red de seguridad: si el robot entra a RECUPERANDO cerca de una esquina real
// (donde un ultrasónico lee "sin pared" legítimamente, no por desalineación),
// wallOk puede no cumplirse NUNCA y el estado se quedaría atorado para siempre.
// Este timeout fuerza la salida aunque wallOk/headingOk no se hayan cumplido.
unsigned long recuperandoEntryMs = 0;
const unsigned long recuperandoTimeoutMs = 1500;

// ── Giros ─────────────────────────────────────────────────────────────────────
bool direccionIzquierda = true;
bool primerGiro         = false;

// Ángulo de giro objetivo — DISTINTO por tipo de ronda:
//   ronda de obstáculos (rondaObstaculos=true) : ~90° reales (pivote/maniobra)
//   ronda cerrada       (rondaObstaculos=false): 76° (el giro continuo se pasa
//                                                por inercia, así que sale antes)
const int ANG_GIRO_OBSTACULOS = 90;   // <- bájalo a 88 si se pasa en la de obstáculos
const int ANG_GIRO_CERRADA    = 76;
const int AngGiro = rondaObstaculos ? ANG_GIRO_OBSTACULOS : ANG_GIRO_CERRADA;
unsigned long lastTurnTime = 0;
int timeStart = 0;
const int cooldownGiro     = 1000;   // ms entre giros

// ── Detección de esquinas ─────────────────────────────────────────────────────
int contadorEsquina    = 0;
const int umbralPared  = 100;   // cm — pared "desaparece" → esquina
const int esquinaDebounce = 2;  // lecturas consecutivas antes de confiar (evita
                                 // falsos positivos por reflexión rasante del
                                 // ultrasónico cuando el chasis yawea fuerte)

// Ronda cerrada: el trigger de giro se ARMA solo después de confirmar que el
// carro ya está DENTRO de un pasillo (ambos laterales < umbralPared por varios
// frames). Sin esto, una lectura ancha de la zona de salida dispara un giro
// falso apenas arranca. Una vez armado se queda armado toda la carrera.
bool giroArmado      = false;
int  contadorPasillo = 0;
const int PASILLO_FRAMES = 3;   // frames seguidos con AMBAS paredes < umbralPared para armar

// Fallback: si el carro arranca pegado a una pared, o justo antes de una esquina,
// UN lateral ya lee "abierto" (>umbralPared) desde el frame 0 y el path de
// pasillo (AMBAS < umbralPared) NUNCA se cumple -> giroArmado se queda en false
// para siempre y el carro avanza sin girar nunca ("ciclado"). Este timeout lo
// arma igual pasado este tiempo desde el arranque real. NO retrasa esquinas
// reales: el path de pasillo ya arma antes cuando puede, y el cooldownGiro
// (2000 ms) impide un giro 1 prematuro de todos modos.
const unsigned long ARMA_GIRO_TIMEOUT_MS = 2500;

// Ronda cerrada: al acercarse a la pared de ENFRENTE se baja la velocidad para
// darle tiempo a detectarEsquina() de leer limpio qué lado se abre antes de
// llegar a la esquina (a full 180 el carro se pasaba antes de confirmar).
const int FRONT_SLOWDOWN_CM  = 60;    // pared de frente más cerca que esto -> frena un poco
const int VEL_APROX_CERRADA  = 140;   // velocidad reducida en la aproximación a la esquina

// ── Carrera ───────────────────────────────────────────────────────────────────
int  turnsCompleted      = 0;
bool raceFinished        = false;
const int TURNS_PER_RACE = 12;

// ── Terminando (regreso al área de salida) ───────────────────────────────────
// Al completar la última vuelta el carro NO frena de golpe: entra en TERMINANDO,
// que maneja igual que SIGUIENDO (visión + PID) pero SIN buscar esquinas y solo
// durante TERMINANDO_MS, para meterse en el área de salida y ahí sí frenar.
// Sube/baja este tiempo según la distancia que falte hasta la zona de salida.
const unsigned long TERMINANDO_MS = 1000;
unsigned long terminandoEntryMs   = 0;

// ── Filtro EMA para ultrasonidos ──────────────────────────────────────────────
float alpha         = 0.85;
float distL_filtrada = 0;
float distR_filtrada = 0;
float distF_filtrada = 0;   // sensor frontal (solo ronda de obstáculos)


// ═══════════════════════════════════════════════════════════════════════════════
// Actuadores
// ═══════════════════════════════════════════════════════════════════════════════

void escribirServo(int angulo) {
  angulo = constrain(angulo, 0, 180);
  int pulso = map(angulo, 0, 180, 500, 2500);
  int duty  = (pulso * ((1 << resServo) - 1)) / 20000;
  ledcWrite(SERVO_PIN, duty);
}

// Techo del PWM del motor — DISTINTO por tipo de ronda:
//   ronda de obstáculos (rondaObstaculos=true) : 100 (maniobras lentas y finas)
//   ronda cerrada       (rondaObstaculos=false): 180 (fiuuummmmm)
const int MOTOR_MAX = rondaObstaculos ? 100 : 180;

void setMotor(int velocidad) {
  velocidad = constrain(velocidad, 0, MOTOR_MAX);
  ledcWrite(PWMA, velocidad);
}

// Motor en coast (terminales flotando) — para frenar suave y dejar caer la
// back-EMF antes de invertir la dirección del puente H.
void motorCoast() {
  setMotor(0);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
}

void motorAdelante() { digitalWrite(A1, HIGH); digitalWrite(A2, LOW); }
void motorReversa()  { digitalWrite(A1, LOW);  digitalWrite(A2, HIGH); }

// Mediana de las últimas 3 lecturas del sensor frontal. El HC-SR04 frontal
// tira picos (55 <-> 199) por multipath / eco perdido; la mediana los rechaza,
// el EMA no. Sin esto un pico espurio disparaba MANIOBRA antes de la esquina.
// 3 (no 5): con 5 el retraso de la mediana metía tarde el slowdown y el carro
// llegaba con poca pista a la esquina.
long medianaFront(long nueva) {
  static long buf[3] = {200, 200, 200};
  static int  idx = 0;
  buf[idx] = nueva;
  idx = (idx + 1) % 3;
  long s[3];
  for (int i = 0; i < 3; i++) s[i] = buf[i];
  for (int i = 0; i < 3; i++)
    for (int j = i + 1; j < 3; j++)
      if (s[j] < s[i]) { long t = s[i]; s[i] = s[j]; s[j] = t; }
  return s[1];
}

// Decide dirección de giro (lado con hueco > umbralPared) y FORWARD vs REVERSE
// (según la distancia a la pared EXTERIOR, la que SÍ existe — el "sin pared"
// nunca se usa como número). La llama CRUCERO en el frame del trigger (para
// elegir el umbral frontal) y latchea con maniobraDecidida=true.
void decidirManiobra(long distL, long distR) {
  bool derAbierta = (distR > umbralPared);
  bool izqAbierta = (distL > umbralPared);

  // ── DIRECCIÓN de giro ── se decide en la 1ª esquina por la pared que ABRE y
  //    se LATCHEA (primerGiro): todas las esquinas de la pista son del mismo
  //    sentido, así que no se re-evalúa. Como CRUCERO ya exige paredAbierta para
  //    disparar, aquí SIEMPRE llega con una abierta -> nunca se usa el fallback.
  if (primerGiro) {
    maniobraGirarDer = !direccionIzquierda;                 // ya latcheada
  } else {
    if      (derAbierta && !izqAbierta) maniobraGirarDer = true;
    else if (izqAbierta && !derAbierta) maniobraGirarDer = false;
    else                               maniobraGirarDer = (distR > distL);  // fallback (no debería pasar)
    primerGiro = true;                                      // latcheado para el resto de la carrera
  }
  direccionIzquierda = !maniobraGirarDer;   // para el dir= del ACK

  // ── FWD vs REVERSE ── SIEMPRE fresco, según la pared EXTERIOR del giro
  //    (giro der -> exterior = izq/distL; giro izq -> exterior = der/distR).
  //    Si esa pared saliera "abierta" (raro en la exterior), usa la otra.
  long distExt;
  if (maniobraGirarDer)  distExt = (distL <= umbralPared) ? distL : distR;
  else                   distExt = (distR <= umbralPared) ? distR : distL;
  maniobraDistExt  = distExt;
  maniobraReversa  = (maniobraDistExt >= HUG_CM);

  // ── ¿RETROCEDER un poco DESPUÉS de la maniobra? ── solo si la pared exterior
  //    (la que sigo) tiene holgura: > MANIOBRA_BACKOFF_MIN_CM. Si voy pegado a
  //    ella, retroceder recto no me separa de la recta nueva, solo raspa.
  maniobraRetroceso = (maniobraDistExt > MANIOBRA_BACKOFF_MIN_CM);

  maniobraDecidida = true;
}

// Arranca el regreso al área de salida. Se llama al completar la última vuelta
// EN LUGAR de frenar en seco (raceFinished=true): el carro sigue manejando como
// en SIGUIENDO durante TERMINANDO_MS y después frena (ver terminando()).
void iniciarTerminando() {
  estado            = TERMINANDO;
  terminandoEntryMs = millis();
  Serial.print("-> TERMINANDO ");
  Serial.print(turnsCompleted);
  Serial.print("/");
  Serial.print(TURNS_PER_RACE);
  Serial.print(" (");
  Serial.print(TERMINANDO_MS);
  Serial.println(" ms hacia el area de salida)");
}

// Cierre de MANIOBRA: endereza, deja el puente en adelante, resetea ángulos
// (recta nueva desde 0) y vuelve a SIGUIENDO. Cuenta el giro.
void finalizarManiobra() {
  motorAdelante();
  escribirServo(centroServo);
  setMotor(0);
  velocidadMotor = 180;
  integralWall = 0; prevErrorWall = 0;
  integralGyro = 0; prevErrorGyro = 0;
  anguloGyro       = 0;
  anguloObjetivo   = 0;          // recta nueva: referencia desde cero
  lastTurnTime     = millis();
  maniobraDecidida = false;
  maniobraFase     = -1;
  estado           = SIGUIENDO;
  turnsCompleted++;
  if (turnsCompleted >= TURNS_PER_RACE) iniciarTerminando();
  Serial.print("MANIOBRA completada ");
  Serial.print(turnsCompleted);
  Serial.print("/");
  Serial.println(TURNS_PER_RACE);
}


// ═══════════════════════════════════════════════════════════════════════════════
// Sensores
// ═══════════════════════════════════════════════════════════════════════════════

long leerDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // 6000 us: 100 cm ida+vuelta = 5882 us, cubre todo el rango útil (umbralPared
  // = 100 cm incluido). El caso "sin eco" es justo el de la esquina y quema el
  // timeout completo cada loop -> con 6000 en vez de 7000 el loop respira un
  // poco más rápido cuando más importa.
  long dur  = pulseIn(echo, HIGH, 6000);
  long dist = dur * 0.034 / 2;
  if (dist == 0 || dist > 200) dist = 200;
  return dist;
}

float filtroEMA(float nueva, float anterior) {
  return alpha * nueva + (1.0 - alpha) * anterior;
}

void actualizarGyro() {
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0;
  lastGyroTime = now;

  float gz = mpu.getGyroZ() / gyroScale;
  if (abs(gz) < 1.0) gz = 0;
  anguloGyro += gz * dt;
}

bool detectarEsquina(long distL, long distR) {
  bool apertura = (distL > umbralPared) || (distR > umbralPared);
  // Histéresis en vez de reset duro: un solo frame < umbralPared (ruido /
  // geometría al borde del umbral) ya NO borra la cuenta, solo la baja 1 ->
  // necesita esquinaDebounce frames NO-apertura seguidos para des-armar,
  // simétrico con el armado. Satura en esquinaDebounce para no acumular de más
  // durante toda la aproximación.
  if (apertura) contadorEsquina = min(contadorEsquina + 1, (int)esquinaDebounce);
  else          contadorEsquina = max(contadorEsquina - 1, 0);
  return contadorEsquina >= esquinaDebounce;
}


// ═══════════════════════════════════════════════════════════════════════════════
// Protocolo serial con Raspberry Pi
// ═══════════════════════════════════════════════════════════════════════════════

void parsePiMessage(String line) {
  line.trim();

  // ── Handshake ──────────────────────────────────────────────────────────────
  if (line.startsWith("READY")) {
    piReadyReceived = true;
    piReady         = true;
    lastPiMsgMs     = millis();
    if (readyMs == 0) readyMs = millis();
    Serial2.println("ACK:READY");
    return;
  }

  // ── Protocolo V2 (Pure Pursuit) y V1 (obstáculo) ─────────────────────────
  // Formato: V2,obs=+0.350,turn=0,state=pp_follow,prio=0,mem=0,pp=1
  //      o:  V1,obs=+0.123,turn=0,state=avoid_red,prio=1,mem=18,pp=0
  //          (ambos se parsean igual — solo difiere el campo pp=)
  if (line.startsWith("V1,") || line.startsWith("V2,")) {

    // obs
    int idx = line.indexOf("obs=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 4, end) : line.substring(idx + 4);
      obsBiasNorm = constrain(s.toFloat(), -1.0, 1.0);
    }

    // turn
    idx = line.indexOf("turn=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 5, end) : line.substring(idx + 5);
      int v = s.toInt();
      turnHint = (v > 0) ? 1 : (v < 0) ? -1 : 0;
    }

    // prio
    idx = line.indexOf("prio=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 5, end) : line.substring(idx + 5);
      piPriority = (s.toInt() != 0);
    }

    // mem
    idx = line.indexOf("mem=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 4, end) : line.substring(idx + 4);
      piMemoryFrames = max(0L, s.toInt());
    }

    // pp  — campo nuevo en V2; ausente en mensajes V1 → pp=false por defecto
    idx = line.indexOf(",pp=");
    if (idx >= 0) {
      String s = line.substring(idx + 4);
      // solo toma el dígito antes de cualquier coma extra
      int end = s.indexOf(',');
      if (end >= 0) s = s.substring(0, end);
      piPurePursuit = (s.toInt() != 0);
    } else {
      piPurePursuit = false;   // mensaje V1 sin campo pp → modo obstáculo
    }

    // pasado — evento de un solo frame: el robot ya atravesó físicamente el
    // obstáculo. Ausente en V1 → false por defecto.
    idx = line.indexOf("pasado=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 7, end) : line.substring(idx + 7);
      piPasado = (s.toInt() != 0);
    } else {
      piPasado = false;
    }

    // intr — el obstáculo actual se pasa por el mismo lado hacia el que va
    // a girar la pista (ver corner_lines.py en la Pi). Ausente en V1 o si
    // la Pi aún no confirmó la dirección de giro → false por defecto
    // (mismo comportamiento de siempre: sigue bloqueando).
    idx = line.indexOf("intr=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 5, end) : line.substring(idx + 5);
      piInteriorPass = (s.toInt() != 0);
    } else {
      piInteriorPass = false;
    }

    piReady           = true;
    piFirstV2Received  = true;   // desde aquí el carro ya puede rodar
    lastPiMsgMs = millis();
    // Regresamos el heading integrado del gyro para que la Pi pueda mantener
    // su mapa rodante de obstáculos alineado al doblar (obstacle_memory.py).
    Serial2.print("ACK:V2,ang=");
    Serial2.print(anguloGyro, 2);
    Serial2.print(",est=");
    // MANIOBRA -> "G" (la Pi hace su manejo de giro: borra memoria, resetea
    // line_tracker). CRUCERO -> "C" (la Pi lo trata igual que "S"; solo sirve
    // para verlo en el journalctl). RECUPERANDO -> "R". SIGUIENDO -> "S".
    Serial2.print((estado == GIRANDO || estado == MANIOBRA) ? "G"
                  : (estado == RECUPERANDO ? "R"
                     : (estado == CRUCERO ? "C" : "S")));
    // Dirección de giro de la pista: '?' hasta el 1er GIRANDO, luego L/R
    // (direccionIzquierda se fija ahí con distL>distR). La Pi la usa para el
    // manejo de conos exteriores de esquina — fiable de la esquina 2 en
    // adelante (la 1 la sigue estimando por visión). primerGiro arranca en
    // false cada corrida porque el ESP se resetea entre runs.
    Serial2.print(",dir=");
    Serial2.print(!primerGiro ? "?" : (direccionIzquierda ? "L" : "R"));
    // ── DEBUG: estado interno del ESP para verlo en el journalctl de la Pi ──
    // (la Pi loguea el ACK crudo; sus parsers ignoran campos que no conocen).
    //   fase  : maniobraFase  (-1 sin init, 0 frenar-antes, 1 pivote, 2 frenar-desp, 3 frenar-y-fwd)
    //   rev   : maniobraReversa (1 = pivote en reversa)
    //   gd    : maniobraGirarDer (1 = giro a la derecha)
    //   dL/dR : ultrasónicos laterales filtrados (cm)
    //   dF    : ultrasónico frontal filtrado (cm)  — 0 si rondaObstaculos=false
    Serial2.print(",fase="); Serial2.print(maniobraFase);
    Serial2.print(",rev=");  Serial2.print(maniobraReversa ? 1 : 0);
    Serial2.print(",gd=");   Serial2.print(maniobraGirarDer ? 1 : 0);
    Serial2.print(",dL=");   Serial2.print((long)distL_filtrada);
    Serial2.print(",dR=");   Serial2.print((long)distR_filtrada);
    Serial2.print(",dF=");   Serial2.print((long)distF_filtrada);
    Serial2.print(",cerca="); Serial2.print(cruceroCerca ? 1 : 0);  // CRUCERO: 1 = gyro+wall (sin visión)
    Serial2.println();
    return;
  }

  // ── Modo legado: solo píxel X (controlPI.py original) ─────────────────────
  bool numeric = true;
  for (unsigned int i = 0; i < line.length(); i++) {
    char c = line.charAt(i);
    if (!(c >= '0' && c <= '9')) { numeric = false; break; }
  }
  if (numeric && line.length() > 0) {
    int x = line.toInt();
    if (x >= 0 && x <= 640) {
      obsBiasNorm   = constrain((float(x) - 320.0) / 320.0, -1.0, 1.0);
      piPurePursuit = false;
      piReady       = true;
      lastPiMsgMs   = millis();
      Serial2.println("ACK:X");
    }
  }
}

void readPiSerial() {
  while (Serial2.available() > 0) {
    String line = Serial2.readStringUntil('\n');
    if (line.length() > 0) parsePiMessage(line);
  }
}


// ═══════════════════════════════════════════════════════════════════════════════
// PID + control de servo
// ═══════════════════════════════════════════════════════════════════════════════

void controlPID(long distL, long distR) {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;
  if (dt < 0.01) dt = 0.01;

  // ── Siempre calculamos wall y gyro (se usan en fallback y logs) ───────────
  bool wallHold = (!rondaObstaculos && primerGiro);

  // Ventana entre "se perdió la pared que venía siguiendo" y "entra GIRANDO"
  // (~esquinaDebounce frames): la distancia de ese lado ya vale 200 (saturada),
  // así que si dejamos el wall PID trabajando, errorWall se clava en el clamp
  // ±50 y suelta un steerazo (+ golpe de derivada KdWall) ANTES de girar ->
  // GIRANDO arranca desde una pose perturbada. En la ronda abierta congelamos
  // el término de pared en esa ventana; el gyro PID mantiene el rumbo hasta el
  // giro. (En la de obstáculos NO se toca: RECUPERANDO/CRUCERO usan errorWall.)
  bool esquinaInminente = !rondaObstaculos
                          && ((distL > umbralPared) || (distR > umbralPared));

  if (esquinaInminente) {
    errorWall     = 0;
    prevErrorWall = 0;   // sin patada de derivada al congelar
  } else if (wallHold) {
    // Ronda abierta con sentido de giro ya conocido: seguir la pared INTERIOR a
    // WALL_HOLD_CM en vez de centrar. Giro izq -> interior = distL; giro der ->
    // interior = distR. El signo se elige para que quede IGUAL que distL-distR:
    // errorWall > 0 -> el carro vira y distL baja / distR sube.
    //   giro izq (interior=distL): errorWall = distL - HOLD
    //       distL > HOLD (abierto) -> +  -> vira a la interior (izq), distL baja. OK
    //   giro der (interior=distR): errorWall = HOLD - distR
    //       distR > HOLD (abierto) -> -  -> vira a la interior (der), distR baja. OK
    long  distInt = direccionIzquierda ? distL : distR;
    float e       = direccionIzquierda ? (distInt - WALL_HOLD_CM)
                                       : (WALL_HOLD_CM - distInt);
    errorWall = e * WALL_HOLD_GAIN;   // compensa la mitad de ganancia de 1 pared
  } else {
    // Antes del 1er giro (dirección aún desconocida) o ronda de obstáculos:
    // comportamiento de siempre, centrar entre ambas paredes.
    errorWall = distL - distR;
  }
  errorWall = constrain(errorWall, -50, 50);

  // Ronda abierta en hold: el heading de referencia (anguloObjetivo) que dejó el
  // último giro continuo está viciado — el giro se pasa por inercia, así que el
  // gyro PID se pasa la recta "corrigiéndolo" y empuja al carro hacia la pared
  // de AFUERA. Mientras las DOS paredes existan, re-referencia anguloObjetivo
  // poco a poco al heading actual: el gyro pasa a ser solo amortiguador y la
  // pared interior manda. (Mismo truco que usa CRUCERO en la ronda de obstáculos.)
  if (wallHold && distL <= umbralPared && distR <= umbralPared) {
    anguloObjetivo += (anguloGyro - anguloObjetivo) * 0.05f;
  }
  integralWall += errorWall * dt;
  integralWall  = constrain(integralWall, -40, 40);
  float derivWall  = (errorWall - prevErrorWall) / dt;
  float outputWall = KpWall * errorWall + KiWall * integralWall + KdWall * derivWall;
  prevErrorWall = errorWall;

  errorGyro = anguloObjetivo - anguloGyro;
  errorGyro = constrain(errorGyro, -20, 20);
  integralGyro += errorGyro * dt;
  integralGyro  = constrain(integralGyro, -30, 30);
  float derivGyro  = (errorGyro - prevErrorGyro) / dt;
  float outputGyro = KpGyro * errorGyro + KiGyro * integralGyro + KdGyro * derivGyro;
  prevErrorGyro = errorGyro;

  // ── Decisión según modo ───────────────────────────────────────────────────
  bool piAlive = (millis() - lastPiMsgMs) <= piTimeoutMs;
  float outputVision = 0.0;
  float outputFinal  = 0.0;

  if (!piAlive) {
    // Pi desconectada → fallback autónomo: paredes + gyro
    piPriority    = false;
    piMemoryFrames = 0;
    piPurePursuit = false;
    turnHint      = 0;
    obsBiasNorm   = 0.0;
    outputFinal   = outputWall + outputGyro;

  } else if (!rondaObstaculos) {
    // ── Ronda ABIERTA (sin obstáculos) ─────────────────────────────────────
    // Se ignora por completo la visión / Pure Pursuit de la Pi: el carro se
    // maneja SOLO con wall PID (centra entre paredes) + gyro PID (mantiene el
    // heading hacia anguloObjetivo). Los giros los dispara el propio ESP32 con
    // detectarEsquina(); la Pi solo se usa para el ACK del heading.
    piPurePursuit  = false;
    piPriority     = false;
    piMemoryFrames = 0;
    turnHint       = 0;
    obsBiasNorm    = 0.0;
    outputFinal    = outputWall + outputGyro;

  } else if (estado == RECUPERANDO || (estado == CRUCERO && cruceroCerca)) {
    // RECUPERANDO, y CRUCERO SOLO cuando ya está cerca de la pared (cruceroCerca):
    // control por gyro hacia anguloObjetivo + wall PID, SIN visión — porque ahí el
    // centerline ya curva para "esquivar" la pared del fondo y enchueca el carro.
    // CRUCERO lejos cae al branch piPurePursuit de abajo (visión, centerline recto).
    // Recalcular error SIN el cap de ±20 usado en controlPID general.
    float errorGyroRecup = anguloObjetivo - anguloGyro;
    errorGyroRecup = constrain(errorGyroRecup, -60, 60);   // más margen real

    float outputRecup = KpGyro * errorGyroRecup + KdGyro * ((errorGyroRecup - prevErrorGyro) / dt);
    prevErrorGyro = errorGyroRecup;
    outputRecup = constrain(outputRecup, -60, 60);   // más rango de servo

    // CRUCERO: además CENTRA en el carril con el wall PID (KiWall=0, así que
    // outputWall es solo P+D, sin integral rancio). PERO solo mientras AMBAS
    // paredes existen; en cuanto una se abre (esquina) errorWall = distL - distR
    // se dispararía y clavaría el servo -> ahí, pura gyro.
    float wallCorr = 0.0;
    if (estado == CRUCERO && distL <= umbralPared && distR <= umbralPared) {
      wallCorr = constrain(outputWall * CRUCERO_WALL_BLEND, -25.0f, 25.0f);
      // El heading de referencia post-MANIOBRA está viciado (no gira exacto 90°).
      // Mientras las paredes centran, re-referencia anguloObjetivo hacia el heading
      // ACTUAL poco a poco -> el gyro deja de "defender" el heading chueco y solo
      // amortigua; las paredes son la referencia real. Sin esto el carro derivaba
      // lap a lap.
      anguloObjetivo += (anguloGyro - anguloObjetivo) * 0.05f;
    }

    int servoRecup = constrain(centroServo + (int)(outputRecup + wallCorr), 20, 150);
    escribirServo(servoRecup);
    setMotor(velocidadMotor);

    Serial.print(estado == CRUCERO ? " | Mode:CRUCERO" : " | Mode:RECUPERANDO");
    Serial.print(" | ErrGyro:"); Serial.print(errorGyroRecup);
    Serial.print(" | Wall:");    Serial.print(wallCorr);
    Serial.print(" | Servo:");   Serial.print(servoRecup);
    return;

  } else if (piPurePursuit) {
    // ── Modo Pure Pursuit ────────────────────────────────────────────────────
    // La Pi ya calculó el ángulo de dirección óptimo siguiendo la centerline.
    // obs = steer_deg / ppSteerGain  →  steerDeg = obs * ppSteerGain = steer_deg
    //   steerDeg > 0 = derecha,  steerDeg < 0 = izquierda  (convención de la Pi).
    float steerDeg = obsBiasNorm * ppSteerGain;
    steerDeg = constrain(steerDeg, -ppSteerGain, ppSteerGain);

    // Corrección de heading SOLO en recta limpia (sin obstáculo activo ni en
    // memoria) — cancela la deriva del centerline sin pelear contra PP
    // cuando sí está esquivando algo.
    float headingCorr = 0.0;
    float wallCorr     = 0.0;

    if (!piPriority && piMemoryFrames <= 0) {
        headingCorr = outputGyro * PP_GYRO_BLEND;   // peso bajo, no domina
        wallCorr    = outputWall * PP_WALL_BLEND;
    }

    int servoAngle = centroServo - (int)((steerDeg * ppServoGain) - headingCorr - wallCorr);
    servoAngle = constrain(servoAngle, 20, 150);
    escribirServo(servoAngle);
    setMotor(velocidadMotor);

    // ── Debug UART ────────────────────────────────────────────────────────────
    Serial.print(" | Mode:PP");
    Serial.print(" | Steer:");  Serial.print(steerDeg);
    Serial.print(" | Servo:");  Serial.print(servoAngle);
    return;

  } else {
    // ── Modo V1: obstáculo / fallback PID ────────────────────────────────────
    // Comportamiento idéntico a Controller_PI.ino
    float localGain = visionSteerGain;
    if (piPriority) localGain *= 1.20;
    outputVision = (obsBiasNorm * localGain) + (float(turnHint) * turnHintGain);
    outputFinal  = outputWall + outputGyro + outputVision;
  }

  outputFinal = constrain(outputFinal, -25, 25);
  escribirServo(centroServo + (int)outputFinal);
  setMotor(velocidadMotor);

  // ── Debug UART ────────────────────────────────────────────────────────────
  Serial.print(" | Mode:");
  Serial.print(!rondaObstaculos ? "ABIERTA"
               : (piPurePursuit ? "PP" : (piAlive ? "V1" : "FALLBACK")));
  Serial.print(" | Wall:");   Serial.print(outputWall);
  Serial.print(" | eWall:");  Serial.print(errorWall);
  Serial.print(esquinaInminente ? "(esq)" : (wallHold ? "(hold)" : "(center)"));
  Serial.print(" | Gyro:");   Serial.print(outputGyro);
  Serial.print(" | Vis:");    Serial.print(outputVision);
  Serial.print(" | Servo:");  Serial.print(centroServo + (int)outputFinal);
}


// ═══════════════════════════════════════════════════════════════════════════════
// Terminando — regreso al área de salida tras la última vuelta
// ═══════════════════════════════════════════════════════════════════════════════
// Mismo control que SIGUIENDO (controlPID: PP/visión + wall/gyro PID) pero SIN
// detectarEsquina() y acotado a TERMINANDO_MS. Al vencer el tiempo frena y marca
// la carrera como terminada (raceFinished) — el loop() ya deja el carro parado.
void terminando(long distL, long distR) {
  velocidadMotor = 180;
  controlPID(distL, distR);

  if (millis() - terminandoEntryMs >= TERMINANDO_MS) {
    raceFinished = true;
    setMotor(0);
    escribirServo(centroServo);
    Serial.println("TERMINANDO completado -> STOP");
  }
}


// ═══════════════════════════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 17, 16);   // RX=17, TX=16 → Raspberry Pi

  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);

  float oz = mpu.getGyroZoffset();
  Serial.print("Offset Z: ");
  Serial.println(oz);
  gyroScale = (abs(oz) > 7.0) ? 2.0 : 1.0;
  Serial.println(gyroScale == 2.0 ? "Offset sucio → /2" : "Offset limpio");

  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);

  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  digitalWrite(A1, HIGH); digitalWrite(A2, LOW);

  ledcAttach(PWMA,      freqMotor, resMotor);
  ledcAttach(SERVO_PIN, freqServo, resServo);
  escribirServo(centroServo);
  delay(200);

  distL_filtrada = leerDistancia(TRIG_L, ECHO_L);
  distR_filtrada = leerDistancia(TRIG_R, ECHO_R);
  distF_filtrada = leerDistancia(TRIG_F, ECHO_F);

  lastPIDTime  = millis();
  lastGyroTime = millis();
  lastPiMsgMs  = 0;
  bootStartMs  = millis();
  anguloObjetivo = 0;

  // Esperar READY de la Pi (igual que Controller_PI.ino)
  Serial.println("Esperando READY desde Pi...");
  piReadyReceived = false;
  unsigned long waitStart = millis();
  while (!piReadyReceived) { // && (millis() - waitStart < 30000)) {
    if (Serial2.available()) {
      String line = Serial2.readStringUntil('\n');
      line.trim();
      if (line.indexOf("READY") >= 0) {
        piReadyReceived = true;
        piReady         = true;
        timeStart = millis();
        readyMs   = millis();
        Serial.println("Recibido READY. Esperando primer V2...");
      }
    }
    delay(50);
  }
  if (!piReadyReceived) {
    Serial.println("Timeout Pi — continuando sin señal Pi.");
  }
  Serial.println("Sistema listo (PurePursuit)");
}


// ═══════════════════════════════════════════════════════════════════════════════
// Loop
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {
  readPiSerial();

  // Espera boot si la Pi todavía no respondió
  if (!piReadyReceived) {
    if ((millis() - bootStartMs) < BOOT_WAIT_PI_MS) {
      escribirServo(centroServo);
      setMotor(0);
      Serial.println(" | Estado:WAIT_PI");
      delay(20);
      return;
    }
    // Pasó el timeout de arranque → continuar de todas formas
  }

  // READY llegó PERO todavía no el primer V2 -> NO rodar. Si no, controlPID()
  // cae en el fallback wall-PID y el carro avanza ~0.5 s sin ver el obstáculo.
  // Con la Pi mandando el primer V2 justo tras READY, esto son unos ms; el
  // timeout evita quedar atorado para siempre si la Pi muere tras el READY.
  if (piReadyReceived && !piFirstV2Received
      && (millis() - readyMs) < FIRST_V2_TIMEOUT_MS) {
    escribirServo(centroServo);
    setMotor(0);
    Serial.println(" | Estado:WAIT_FIRST_V2");
    delay(10);
    return;
  }

  if (raceFinished) {
    setMotor(0);
    escribirServo(centroServo);
    Serial.println(" | TERMINADO giros=12/12");
    delay(20);
    return;
  }

  // Arranque REAL del carro (ya pasó WAIT_PI y WAIT_FIRST_V2): re-ancla timeStart
  // aquí para que (millis()-timeStart) mida desde que empieza a rodar, no desde
  // el READY. La protección de fondo contra el giro-falso de arranque es el
  // latch giroArmado (abajo), no un grace por tiempo.
  if (!marchaIniciada) {
    marchaIniciada = true;
    timeStart      = millis();
  }

  mpu.update();
  actualizarGyro();

  long distL_raw = leerDistancia(TRIG_L, ECHO_L);
  long distR_raw = leerDistancia(TRIG_R, ECHO_R);
  distL_filtrada = filtroEMA(distL_raw, distL_filtrada);
  distR_filtrada = filtroEMA(distR_raw, distR_filtrada);

  long distL = (long)distL_filtrada;
  long distR = (long)distR_filtrada;

  // Sensor frontal: en ronda de obstáculos alimenta CRUCERO/MANIOBRA; en ronda
  // cerrada sirve para frenar un poco al acercarse a la pared de enfrente
  // (FRONT_SLOWDOWN_CM). Mediana de 5 (rechaza picos) + un EMA suave encima.
  long distF_med = medianaFront(leerDistancia(TRIG_F, ECHO_F));
  distF_filtrada = filtroEMA(distF_med, distF_filtrada);
  long distF = (long)distF_filtrada;

  switch (estado) {

    case SIGUIENDO: {
      velocidadMotor = 180;

      // Ronda cerrada: si la pared de ENFRENTE ya está cerca, baja la velocidad
      // en la aproximación para que detectarEsquina() alcance a confirmar qué
      // lado se abre antes de que el carro se pase la esquina.
      if (!rondaObstaculos && distF > 0 && distF < FRONT_SLOWDOWN_CM) {
        velocidadMotor = VEL_APROX_CERRADA;
      }

      // La Pi confirma que el robot ya atravesó físicamente el obstáculo
      // (evento de un solo frame) -> entrar a RECUPERANDO. Ya no depende de
      // que la cámara simplemente haya dejado de verlo.
      // En ronda ABIERTA (!rondaObstaculos) no hay obstáculos: se ignora el
      // pulso y el carro sigue en puro wall+gyro PID.
      if (piPasado && rondaObstaculos) {
        estado = RECUPERANDO;
        recuperandoEntryMs = millis();
        integralWall  = 0; prevErrorWall  = 0;
        integralGyro  = 0; prevErrorGyro  = 0;

        // Consumir el pulso.  piPasado es un evento de UN frame en la Pi, pero
        // en el ESP32 se queda en 1 hasta que llega el siguiente mensaje V2
        // (~70-150 ms) y el loop() corre cientos de veces en ese lapso.  Sin
        // esto, si RECUPERANDO sale rápido (headingOk ya se cumple porque el
        // rebase fue casi recto), SIGUIENDO vuelve a entrar a RECUPERANDO en la
        // iteración siguiente con el MISMO pulso viejo -> rebote de estado y
        // reseteo repetido de integrales / patada en la derivada del servo.
        // Un rebase nuevo real llega en otro mensaje y vuelve a poner piPasado=1.
        piPasado = false;

        controlPID(distL, distR);   // ya toma el branch RECUPERANDO (estado ya cambió)
        break;
      }

      controlPID(distL, distR);

      // No girar si hay obstáculo activo en Pi.  (El gate de heading ya no
      // hace falta aquí — mientras el chasis sigue desalineado, ese trabajo
      // lo hace el estado RECUPERANDO, que ni siquiera llega a evaluar
      // detectarEsquina() porque vive en otro case del switch.)
      // Excepción: si el obstáculo se pasa por el mismo lado hacia el que
      // ya se sabe que va a girar la pista (piInteriorPass), el giro mismo
      // resuelve el paso — no tiene caso seguir bloqueando. piInteriorPass
      // es false por defecto (sin dirección confirmada aún, o exterior),
      // así que sin eso el comportamiento es idéntico al de siempre.
      bool bloqueadoPorObstaculo = (piPriority || (piMemoryFrames > 0)) && !piInteriorPass;

      // 2026-08-28: gate de alineación. En un latiguazo de esquiva el chasis
      // queda ladeado (visto en pista a +37deg) y un ultrasónico lateral lee
      // "sin pared" -> detectarEsquina() disparaba una falsa esquina.
      // 25° (no 15): con 15 el carro no lograba mantenerse recto y NO detectaba
      // la esquina real -> se iba de frente. 25 bloquea el latiguazo (37°) pero
      // permite esquinas reales con algo de error de heading. (Con
      // INTERIOR_PASS_ENABLED=False el bloqueo por obstáculo ya tapa el caso
      // original; esto es red de seguridad.)
      bool chasisAlineado = fabs(anguloGyro) < 25.0f;

      if ((millis() - lastTurnTime > cooldownGiro)
          && millis() - timeStart > 500)
      {
        if (rondaObstaculos) {
          // Ronda de obstáculos: NO giro continuo. Si la recta ya está limpia
          // (sin obstáculo mío) y nos acercamos a la esquina -> CRUCERO (control
          // por ángulo hasta la pared). El obstáculo "beyond" de la recta
          // siguiente no cuenta: la Pi ya lo excluye de prio/mem.
          if (!piPriority && piMemoryFrames <= 0
              && distF > 0 && distF < FRONT_CRUCERO_CM) {
            contadorFront++;
            if (contadorFront >= esquinaDebounce) {
              contadorFront   = 0;
              cruceroEntryMs  = millis();
              cruceroCerca    = false;   // fuerza el edge-detect de la 1ª frame de CRUCERO
              estado          = CRUCERO;
              Serial.println("-> CRUCERO");
            }
          } else {
            contadorFront = 0;
          }
        } else {
          // Ronda cerrada: giro continuo. El trigger NO se habilita hasta que el
          // carro confirmó estar en un pasillo (ambas paredes < umbralPared por
          // PASILLO_FRAMES) -> la zona de salida ancha no dispara el giro 1.
          if (!giroArmado) {
            if (distL < umbralPared && distR < umbralPared) contadorPasillo++;
            else                                            contadorPasillo = 0;
            if (contadorPasillo >= PASILLO_FRAMES) {
              giroArmado = true;
              Serial.println("Giro ARMADO (pasillo confirmado)");
            } else if (millis() - timeStart > ARMA_GIRO_TIMEOUT_MS) {
              // Nunca se confirmó pasillo (arrancó pegado a una pared / antes de
              // una esquina). Armar por tiempo para que el carro no se quede
              // sin girar nunca.
              giroArmado = true;
              Serial.println("Giro ARMADO (timeout)");
            }
          }

          if (giroArmado && !bloqueadoPorObstaculo && detectarEsquina(distL, distR)) {
            estado     = GIRANDO;
            anguloGyro = 0;
            if (!primerGiro) {
              direccionIzquierda = (distL > distR);
              primerGiro         = true;
            }
            piPurePursuit = false;   // suspender PP durante el giro
            Serial.println(direccionIzquierda ? "Giro izquierda" : "Giro derecha");
          }
        }
      }
      break;
    }

    case RECUPERANDO: {
      velocidadMotor = 180;
      controlPID(distL, distR);   // toma el branch RECUPERANDO de controlPID()

      bool wallOk    = abs(errorWall) < wallSettleCm;
      bool headingOk = abs(errorGyro) < headingSettleDeg;
      bool timedOut  = (millis() - recuperandoEntryMs) > recuperandoTimeoutMs;

      if (piPriority) {
        // Reapareció un obstáculo (o uno nuevo) → vuelve a esquivar
        estado = SIGUIENDO;
      } else if ((headingOk)) {
        // Ya centrado y alineado → visión retoma el control normal.
        // timedOut: red de seguridad si wallOk nunca se cumple (p.ej. cerca
        // de una esquina real, donde un lado lee "sin pared" legítimamente).
        // Ronda de obstáculos: si además NO queda obstáculo mío, pasa a CRUCERO
        // (va derecho por ángulo hacia la esquina). Con objeto presente -> a
        // SIGUIENDO para esquivarlo, luego RECUPERANDO, luego CRUCERO.
        if (rondaObstaculos && !piPriority && piMemoryFrames <= 0) {
          cruceroEntryMs = millis();
          cruceroCerca   = false;   // fuerza el edge-detect de la 1ª frame de CRUCERO
          estado         = CRUCERO;
          Serial.println("-> CRUCERO (post-recup)");
        } else {
          estado = SIGUIENDO;
        }
      }
      break;
    }

    case GIRANDO: {
      float delta = abs(anguloGyro);

      if      (delta < 45) velocidadMotor = 165;
      else if (delta < 70) velocidadMotor = 145;
      else                 velocidadMotor = 120;

      setMotor(velocidadMotor);
      escribirServo(direccionIzquierda ? 150 : 20);

      if (delta >= AngGiro) {
        escribirServo(centroServo);
        velocidadMotor = 180;

        // Resetear integrales
        integralWall = 0; prevErrorWall = 0;
        integralGyro = 0; prevErrorGyro = 0;

        anguloObjetivo = anguloGyro;
        lastTurnTime   = millis();
        estado         = SIGUIENDO;
        turnsCompleted++;

        if (turnsCompleted >= TURNS_PER_RACE) {
          iniciarTerminando();
        }

        Serial.print("Giro completado ");
        Serial.print(turnsCompleted);
        Serial.print("/");
        Serial.println(TURNS_PER_RACE);
      }
      break;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // CRUCERO — solo ronda de obstáculos. Recta ya limpia + esquina cerca:
    // va DERECHO por ángulo (mismo control que RECUPERANDO en controlPID) hasta
    // ~FRONT_TURN_CM de la pared, luego MANIOBRA. Si aparece un obstáculo mío,
    // vuelve a SIGUIENDO para esquivarlo.
    // ═══════════════════════════════════════════════════════════════════════════
    case CRUCERO: {
      velocidadMotor = 180;
      if (piPasado) piPasado = false;   // pulso viejo/rezagado: CRUCERO ya mantiene heading
      // Cerca de la pared -> pura gyro+wall (controlPID lo enruta con cruceroCerca).
      // Lejos -> visión (el centerline todavía va recto).
      bool _cercaAntes = cruceroCerca;
      cruceroCerca = (distF > 0 && distF <= CRUCERO_GYRO_CM);
      if (cruceroCerca && !_cercaAntes) {
        // Acabamos de cortar visión. El heading ACTUAL es el que visión dejó
        // (recto), NO el anguloObjetivo viejo (que quedó chueco tras una maniobra
        // que se pasó/quedó corta). Adóptalo como referencia para no volver a él.
        anguloObjetivo = anguloGyro;
        integralGyro   = 0;
        prevErrorGyro  = 0;
      }
      controlPID(distL, distR);

      if (piPriority || piMemoryFrames > 0) {
        estado = SIGUIENDO;             // apareció obstáculo mío -> a esquivarlo
        contadorFront = 0;
        break;
      }

      // Preview de la decisión para elegir el UMBRAL frontal: REVERSE necesita
      // estar cerca de la pared (30), FORWARD necesita espacio para el arco (60).
      bool _revPrev;
      {
        bool _da = (distR > umbralPared), _ia = (distL > umbralPared);
        long _de;
        if      (_da && !_ia) _de = distL;
        else if (_ia && !_da) _de = distR;
        else                  _de = ((distR > distL) ? distL : distR);
        _revPrev = (_de >= HUG_CM);
      }
      int _umbralFront = _revPrev ? FRONT_TURN_REV_CM : FRONT_TURN_FWD_CM;

      // La maniobra SOLO dispara si de verdad estás en una esquina = una pared
      // lateral ABIERTA (>umbralPared). Sin esto, un pico de ruido de dF disparaba
      // la maniobra en medio de la recta (carro encajonado, ninguna pared abierta)
      // y decidirManiobra caía al fallback y giraba al lado equivocado.
      bool paredAbierta = (distL > umbralPared || distR > umbralPared);
      bool enLaPared    = (distF > 0 && distF <= _umbralFront && paredAbierta);
      bool cruceroLargo = (millis() - cruceroEntryMs) > CRUCERO_TIMEOUT_MS;  // red de seguridad

      if (enLaPared) contadorFront++;
      else           contadorFront = 0;

      if (contadorFront >= esquinaDebounce || cruceroLargo) {
        contadorFront = 0;
        decidirManiobra(distL, distR);   // decisión DEFINITIVA, latcheada
        maniobraFase  = -1;              // MANIOBRA hará el phase-init
        piPurePursuit = false;
        estado        = MANIOBRA;
        Serial.print("-> MANIOBRA dir="); Serial.print(maniobraGirarDer ? "DER" : "IZQ");
        Serial.print(maniobraReversa ? " REVERSA" : " FORWARD");
        Serial.print(" distExt="); Serial.print(maniobraDistExt);
        Serial.print(" distF=");   Serial.print(distF);
        Serial.println(cruceroLargo ? " TIMEOUT" : "");
      }
      break;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // MANIOBRA — solo ronda de obstáculos. Reemplaza al giro continuo.
    //   decide (1 vez): dirección = lado con hueco (>umbralPared). FORWARD vs
    //   REVERSE según la distancia a la pared EXTERIOR (la que SÍ existe).
    //   Máquina de fases (los coast evitan freír el puente H por "plugging",
    //   reventó un TB6612 así 2026-09-01):
    //     0 COAST         : (solo REV) frena — venía de frente de CRUCERO —
    //                       MANIOBRA_FRENO_MS, luego arranca reversa -> fase 1
    //     1 PIVOTE        : servo (contrario si REV) + motor, con rampa, hasta EXIT_DEG
    //                       (REV atascada -> fase 3: frena y termina de frente)
    //     2 COAST-FIN     : frena tras el pivote. Sin retroceso -> cierra.
    //                       Con retroceso -> fase 4.
    //     4 RETROCESO-POST: motor en reversa MANIOBRA_BACKOFF_MS — toma distancia
    //                       de la recta nueva. SOLO si maniobraRetroceso (pared
    //                       exterior > MANIOBRA_BACKOFF_MIN_CM). -> fase 5
    //     5 COAST         : frena tras el retroceso, luego cierra
    //     3 FRENAR-Y-FWD  : coast, luego re-arranca el pivote de frente
    //   FWD sin retroceso: phase-init -> fase 1 directo (mismo sentido, sin coast)
    //   -> EXIT_DEG -> cierra. FWD con retroceso: fase 1 -> fase 2 (coast para
    //   invertir a reversa) -> fase 4 -> fase 5 -> cierra.
    //   Fin: endereza, resetea (recta nueva desde 0), turnsCompleted++, SIGUIENDO.
    // ═══════════════════════════════════════════════════════════════════════════
    case MANIOBRA: {
      if (!maniobraDecidida) decidirManiobra(distL, distR);   // safety (normalmente CRUCERO ya decidió)

      if (maniobraFase < 0) {   // phase-init (una vez por maniobra)
        // REV -> coast (0) -> pivote en reversa (1)
        // FWD -> pivote de frente (1) directo (mismo sentido que CRUCERO, sin coast)
        if (maniobraReversa) {
          maniobraFase   = 0;
          maniobraFaseMs = millis();
          motorCoast();
        } else {
          motorAdelante();
          anguloGyro       = 0;
          maniobraPivoteMs = millis();
          maniobraFase     = 1;
        }
      }

      float delta = abs(anguloGyro);
      const int EXIT_DEG = AngGiro - MANIOBRA_OVERSHOOT_DEG;   // sale antes: la inercia completa

      // ── Fase 0: FRENAR (el motor viene de frente de CRUCERO) antes de invertir ─
      if (maniobraFase == 0) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - maniobraFaseMs >= MANIOBRA_FRENO_MS) {
          motorReversa();                 // motor parado -> arranca en reversa
          anguloGyro       = 0;
          maniobraPivoteMs = millis();
          maniobraFase     = 1;           // pivote en reversa
        }
        break;
      }

      // ── Fase 1: PIVOTE ───────────────────────────────────────────────────
      if (maniobraFase == 1) {
        // reversa atascada -> frenar y terminar de frente
        if (maniobraReversa && delta < EXIT_DEG
            && (millis() - maniobraPivoteMs) > MANIOBRA_REV_TIMEOUT_MS) {
          maniobraReversa = false;
          maniobraFase    = 3;
          maniobraFaseMs  = millis();
          motorCoast();
          Serial.println("MANIOBRA: reversa timeout -> freno -> forward");
          break;
        }

        if (maniobraReversa) {
          unsigned long tR = millis() - maniobraPivoteMs;
          int vel = (tR < MANIOBRA_RAMP_MS)
                    ? (int)map((long)tR, 0, (long)MANIOBRA_RAMP_MS,
                               MANIOBRA_VEL_MIN, MANIOBRA_VEL_REV)
                    : MANIOBRA_VEL_REV;
          if (delta > EXIT_DEG - 20) vel = min(vel, MANIOBRA_VEL_MIN);   // frena el último tramo
          motorReversa();
          escribirServo(maniobraGirarDer ? 150 : 20);   // servo CONTRARIO al giro
          setMotor(vel);
        } else {
          if      (delta < 45)            velocidadMotor = 165;
          else if (delta < EXIT_DEG - 20) velocidadMotor = 145;
          else                            velocidadMotor = 100;   // último tramo: crawl
          motorAdelante();
          escribirServo(maniobraGirarDer ? 20 : 150);    // servo hacia el giro
          setMotor(velocidadMotor);
        }

        if (delta >= EXIT_DEG) {
          if (maniobraReversa || maniobraRetroceso) {
            // REV: hay que frenar antes de volver a adelante.
            // FWD con retroceso: frenar antes de invertir a reversa (fase 4).
            maniobraFase   = 2;
            maniobraFaseMs = millis();
            motorCoast();
            escribirServo(centroServo);
          } else {
            finalizarManiobra();          // FWD sin retroceso: mismo sentido, cierra ya
          }
        }
        break;
      }

      // ── Fase 2: FRENAR tras el pivote ───────────────────────────────────
      //   sin retroceso -> cierra. con retroceso -> fase 4 (retroceso-post).
      if (maniobraFase == 2) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - maniobraFaseMs >= MANIOBRA_FRENO_MS) {
          if (maniobraRetroceso) {
            motorReversa();               // motor parado -> arranca en reversa
            maniobraFaseMs = millis();
            maniobraFase   = 4;
          } else {
            finalizarManiobra();
          }
        }
        break;
      }

      // ── Fase 4: RETROCESO-POST — toma distancia de la recta nueva ─────────
      //   servo centrado, retrocede recto MANIOBRA_BACKOFF_MS. NO toca el
      //   heading final (el pivote ya llegó a EXIT_DEG).
      if (maniobraFase == 4) {
        motorReversa();
        escribirServo(centroServo);
        setMotor(MANIOBRA_BACKOFF_VEL);
        if (millis() - maniobraFaseMs >= MANIOBRA_BACKOFF_MS) {
          motorCoast();
          maniobraFaseMs = millis();
          maniobraFase   = 5;
        }
        break;
      }

      // ── Fase 5: FRENAR tras el retroceso, luego cerrar ───────────────────
      if (maniobraFase == 5) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - maniobraFaseMs >= MANIOBRA_FRENO_MS) finalizarManiobra();
        break;
      }

      // ── Fase 3: FRENAR tras timeout de reversa, luego pivote de frente ───
      if (maniobraFase == 3) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - maniobraFaseMs >= MANIOBRA_FRENO_MS) {
          motorAdelante();
          maniobraPivoteMs = millis();
          maniobraFase     = 1;          // vuelve al pivote, ahora de frente
        }
        break;
      }
      break;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // TERMINANDO — tras la última vuelta: maneja como SIGUIENDO (sin esquinas)
    // durante TERMINANDO_MS para entrar al área de salida, luego frena.
    // ═══════════════════════════════════════════════════════════════════════════
    case TERMINANDO: {
      terminando(distL, distR);
      break;
    }
  }

  // ── Log periódico ─────────────────────────────────────────────────────────
  Serial.print(" | Estado:");
  if      (estado == GIRANDO)     Serial.print("GIRANDO");
  else if (estado == RECUPERANDO) Serial.print("RECUPERANDO");
  else if (estado == CRUCERO)     Serial.print("CRUCERO");
  else if (estado == MANIOBRA)    Serial.print("MANIOBRA");
  else if (estado == TERMINANDO)  Serial.print("TERMINANDO");
  else                             Serial.print("SIGUIENDO");
  Serial.print(" | PP:");       Serial.print(piPurePursuit ? 1 : 0);
  Serial.print(" | L:");        Serial.print(distL);
  Serial.print(" | R:");        Serial.print(distR);
  Serial.print(" | F:"); Serial.print(distF);
  Serial.print(" | Ang:");      Serial.print(anguloGyro);
  Serial.print(" | Obj:");      Serial.print(anguloObjetivo);
  Serial.print(" | obs:");      Serial.print(obsBiasNorm, 3);
  Serial.print(" | turn:");     Serial.print(turnHint);
  Serial.print(" | prio:");     Serial.print(piPriority ? 1 : 0);
  Serial.print(" | mem:");      Serial.print(piMemoryFrames);
  Serial.print(" | intr:");     Serial.print(piInteriorPass ? 1 : 0);
  Serial.print(" | giros:");    Serial.print(turnsCompleted);
  Serial.print("/");            Serial.println(TURNS_PER_RACE);
}
