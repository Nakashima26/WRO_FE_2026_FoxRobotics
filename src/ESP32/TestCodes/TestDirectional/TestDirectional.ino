// Barrido continuo del servo de dirección: 20° ↔ 150°
// Pin: GPIO 13 (mismo que PurePursuit / Controller_PI)

#define SERVO_PIN 13
const int freqServo = 50;
const int resServo  = 16;

void escribirServo(int angulo) {
  angulo = constrain(angulo, 20, 150);
  int pulso = map(angulo, 0, 180, 500, 2400);  // us
  int duty  = (pulso * ((1 << resServo) - 1)) / 20000;
  ledcWrite(SERVO_PIN, duty);
}

void setup() {
  Serial.begin(115200);
  ledcAttach(SERVO_PIN, freqServo, resServo);
  escribirServo(80);  // centro
  delay(500);
  Serial.println("Servo sweep 20 <-> 150");
}

void loop() {
  for (int a = 20; a <= 150; a++) {
    escribirServo(a);
    Serial.println(a);
    delay(15);
  }
  delay(300);

  for (int a = 150; a >= 20; a--) {
    escribirServo(a);
    Serial.println(a);
    delay(15);
  }
  delay(300);
}
