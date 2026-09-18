// Salto directo 20° ↔ 150° (sin pasos intermedios)
#define SERVO_PIN 13
const int freqServo = 50;
const int resServo  = 16;

const int ANG_MIN = 20;
const int ANG_MAX = 150;
const int HOLD_MS = 600;

void escribirServo(int angulo) {
  angulo = constrain(angulo, ANG_MIN, ANG_MAX);
  int pulso = map(angulo, 0, 180, 500, 2400);
  int duty  = (pulso * ((1 << resServo) - 1)) / 20000;
  ledcWrite(SERVO_PIN, duty);
}

void setup() {
  Serial.begin(115200);
  ledcAttach(SERVO_PIN, freqServo, resServo);
  Serial.println("Servo jump 20 <-> 150");
}

void loop() {
  escribirServo(ANG_MIN);
  Serial.println(ANG_MIN);
  delay(HOLD_MS);

  escribirServo(ANG_MAX);
  Serial.println(ANG_MAX);
  delay(HOLD_MS);
}
