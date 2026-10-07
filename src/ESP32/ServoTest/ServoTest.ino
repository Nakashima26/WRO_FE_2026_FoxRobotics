/*
 * ServoTest — salta el servo 0 -> 180 -> 0 de golpe (sin barrido).
 * Mismo pinout y PWM que PurePursuit.ino. El motor queda APAGADO
 * (PWMA = 0, A1/A2 en LOW) para que el carro no se mueva.
 *
 * PINES (iguales a PurePursuit.ino):
 *   HC-SR04 izq  : TRIG=27, ECHO=32
 *   HC-SR04 der  : TRIG=26, ECHO=35
 *   HC-SR04 front: TRIG=14, ECHO=33
 *   Motor DC     : PWMA=23, A1=18, A2=19  (TB6612FNG)
 *   Servo SG90   : SERVO_PIN=13
 */

// ── Pines ─────────────────────────────────────────────────────────────────────
#define TRIG_L      27
#define ECHO_L      32
#define TRIG_R      26
#define ECHO_R      35

#define PWMA        23
#define A1          18
#define A2          19
#define SERVO_PIN   13
#define TRIG_F      14
#define ECHO_F      33

// ── PWM ───────────────────────────────────────────────────────────────────────
const int freqServo  = 50;
const int resServo   = 16;
const int freqMotor  = 1000;
const int resMotor   = 8;

const unsigned long ESPERA_MS = 1000;  // tiempo en cada extremo

void escribirServo(int angulo) {
  angulo = constrain(angulo, 0, 180);
  int pulso = map(angulo, 0, 180, 500, 2500);
  int duty  = (pulso * ((1 << resServo) - 1)) / 20000;
  ledcWrite(SERVO_PIN, duty);
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  digitalWrite(TRIG_L, LOW); digitalWrite(TRIG_R, LOW); digitalWrite(TRIG_F, LOW);

  // Motor apagado
  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);
  ledcAttach(PWMA, freqMotor, resMotor);
  ledcWrite(PWMA, 0);

  ledcAttach(SERVO_PIN, freqServo, resServo);
  escribirServo(90);
  delay(1000);
}

void loop() {
  Serial.println("0");
  escribirServo(0);
  delay(ESPERA_MS);

  Serial.println("180");
  escribirServo(180);
  delay(ESPERA_MS);
}
