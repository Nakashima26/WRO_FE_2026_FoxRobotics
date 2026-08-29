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

// ── PWM ───────────────────────────────────────────────────────────────────────
const int freqServo  = 50;
const int resServo   = 16;
const int freqMotor  = 1000;
const int resMotor   = 8;

// ── PID Paredes (ultrasónicos) ────────────────────────────────────────────────
float KpWall = 1.0;
float KiWall = 0.0;
float KdWall = 1.2;

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

// Ganancia Pure Pursuit: obs = steer_deg / 35 → steerDeg = obs * 35 = steer_deg
const float ppSteerGain = 60.0;

// Cuánto se deflecta el servo por cada grado de PP.  steerDeg sale de la
// geometría (máx ±35°) y suele quedar corto para la mecánica del servo:
// súbelo si el carrito gira poco, bájalo si oscila/sobregira.
float ppServoGain = 1.0;
float PP_GYRO_BLEND = 0.4;   // 0 = solo vision, 1 = solo gyro. Empieza bajo y sube si sigue derivando.
float PP_WALL_BLEND = .15;

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

// ── FSM estados ───────────────────────────────────────────────────────────────
// RECUPERANDO: la Pi confirma (piPasado=1) que el robot YA atravesó
// físicamente un obstáculo — no que la cámara simplemente dejó de verlo
// (perder de vista ≠ haber rebasado). En vez de que Pure Pursuit intente
// enderezar solo con lo que ve en ese instante incierto, aquí el wall PID +
// gyro PID (que YA se calculan siempre, ver controlPID()) toman el volante
// hasta que el robot vuelve a estar centrado/alineado.
enum Estado { SIGUIENDO, RECUPERANDO, GIRANDO };
Estado estado = SIGUIENDO;

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
int  AngGiro            =88;
unsigned long lastTurnTime = 0;
int timeStart = 0;
const int cooldownGiro     = 2000;   // ms entre giros

// ── Detección de esquinas ─────────────────────────────────────────────────────
int contadorEsquina    = 0;
const int umbralPared  = 100;   // cm — pared "desaparece" → esquina
const int esquinaDebounce = 2;  // lecturas consecutivas antes de confiar (evita
                                 // falsos positivos por reflexión rasante del
                                 // ultrasónico cuando el chasis yawea fuerte)

// ── Carrera ───────────────────────────────────────────────────────────────────
int  turnsCompleted      = 0;
bool raceFinished        = false;
const int TURNS_PER_RACE = 12;

// ── Filtro EMA para ultrasonidos ──────────────────────────────────────────────
float alpha         = 0.85;
float distL_filtrada = 0;
float distR_filtrada = 0;


// ═══════════════════════════════════════════════════════════════════════════════
// Actuadores
// ═══════════════════════════════════════════════════════════════════════════════

void escribirServo(int angulo) {
  angulo = constrain(angulo, 0, 180);
  int pulso = map(angulo, 0, 180, 500, 2500);
  int duty  = (pulso * ((1 << resServo) - 1)) / 20000;
  ledcWrite(SERVO_PIN, duty);
}

void setMotor(int velocidad) {
  velocidad = constrain(velocidad, 0, 100);
  ledcWrite(PWMA, velocidad);
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

  long dur  = pulseIn(echo, HIGH, 7000);
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
  if (apertura) contadorEsquina++;
  else          contadorEsquina = 0;
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
    Serial2.println(estado == GIRANDO ? "G" : (estado == RECUPERANDO ? "R" : "S"));
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
  errorWall = distL - distR;
  errorWall = constrain(errorWall, -50, 50);
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

  } else if (estado == RECUPERANDO) {
    // Recalcular error SIN el cap de ±20 usado en controlPID general
    float errorGyroRecup = anguloObjetivo - anguloGyro;
    errorGyroRecup = constrain(errorGyroRecup, -60, 60);   // más margen real

    float outputRecup = KpGyro * errorGyroRecup + KdGyro * ((errorGyroRecup - prevErrorGyro) / dt);
    prevErrorGyro = errorGyroRecup;

    outputRecup = constrain(outputRecup, -60, 60);   // más rango de servo
    int servoRecup = constrain(centroServo + (int)outputRecup, 20, 150);   // usar límites físicos reales
    escribirServo(servoRecup);
    setMotor(velocidadMotor);

    Serial.print(" | Mode:RECUPERANDO");
    Serial.print(" | ErrGyro:"); Serial.print(errorGyroRecup);
    Serial.print(" | OutRecup:"); Serial.print(outputRecup);
    Serial.print(" | Servo:"); Serial.print(servoRecup);
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
  Serial.print(piPurePursuit ? "PP" : (piAlive ? "V1" : "FALLBACK"));
  Serial.print(" | Wall:");   Serial.print(outputWall);
  Serial.print(" | Gyro:");   Serial.print(outputGyro);
  Serial.print(" | Vis:");    Serial.print(outputVision);
  Serial.print(" | Servo:");  Serial.print(centroServo + (int)outputFinal);
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

  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  digitalWrite(A1, HIGH); digitalWrite(A2, LOW);

  ledcAttach(PWMA,      freqMotor, resMotor);
  ledcAttach(SERVO_PIN, freqServo, resServo);
  escribirServo(centroServo);
  delay(200);

  distL_filtrada = leerDistancia(TRIG_L, ECHO_L);
  distR_filtrada = leerDistancia(TRIG_R, ECHO_R);

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

  mpu.update();
  actualizarGyro();

  long distL_raw = leerDistancia(TRIG_L, ECHO_L);
  long distR_raw = leerDistancia(TRIG_R, ECHO_R);
  distL_filtrada = filtroEMA(distL_raw, distL_filtrada);
  distR_filtrada = filtroEMA(distR_raw, distR_filtrada);

  long distL = (long)distL_filtrada;
  long distR = (long)distR_filtrada;

  switch (estado) {

    case SIGUIENDO: {
      velocidadMotor = 180;

      // La Pi confirma que el robot ya atravesó físicamente el obstáculo
      // (evento de un solo frame) -> entrar a RECUPERANDO. Ya no depende de
      // que la cámara simplemente haya dejado de verlo.
      if (piPasado) {
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

      // Gate de alineación. En un latiguazo de esquiva el chasis queda ladeado
      // (visto en pista a ~37° de ERROR de heading) y un ultrasónico lateral lee
      // "sin pared" -> detectarEsquina() disparaba una falsa esquina y el carro
      // giraba CON la inclinación de la esquiva -> +88° extra -> choque
      // (orillas419). 25° bloquea el latiguazo pero permite esquinas reales con
      // algo de error de heading.
      // 2026-08-29: (a) AHORA SÍ está en el if de abajo -- estaba calculado y
      // sin usar. (b) Es |anguloObjetivo - anguloGyro| (error vs la recta
      // ACTUAL), NO |anguloGyro|: tras cada giro anguloObjetivo queda en ~±88
      // (no en 0), y en RECTA el PID mantiene anguloGyro ~= anguloObjetivo ->
      // |anguloGyro| vale ~88 en recta y |anguloGyro|<25 habría bloqueado
      // TODAS las esquinas menos la primera. El error vs anguloObjetivo sí es
      // "qué tan ladeado voy respecto a mi recta".
      bool chasisAlineado = fabs(anguloObjetivo - anguloGyro) < 25.0f;

      if ((millis() - lastTurnTime > cooldownGiro)
          && !bloqueadoPorObstaculo
          && chasisAlineado
          && detectarEsquina(distL, distR)
          && millis() - timeStart > 3000)
      {
        estado     = GIRANDO;
        anguloGyro = 0;

        if (!primerGiro) {
          direccionIzquierda = (distL > distR);
          primerGiro         = true;
        }

        // Suspender PP durante el giro — el servo lo controla GIRANDO
        piPurePursuit = false;

        Serial.println(direccionIzquierda ? "Giro izquierda" : "Giro derecha");
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
        estado = SIGUIENDO;
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
          raceFinished = true;
        }

        Serial.print("Giro completado ");
        Serial.print(turnsCompleted);
        Serial.print("/");
        Serial.println(TURNS_PER_RACE);
      }
      break;
    }
  }

  // ── Log periódico ─────────────────────────────────────────────────────────
  Serial.print(" | Estado:");
  if      (estado == GIRANDO)     Serial.print("GIRANDO");
  else if (estado == RECUPERANDO) Serial.print("RECUPERANDO");
  else                             Serial.print("SIGUIENDO");
  Serial.print(" | PP:");       Serial.print(piPurePursuit ? 1 : 0);
  Serial.print(" | L:");        Serial.print(distL);
  Serial.print(" | R:");        Serial.print(distR);
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
