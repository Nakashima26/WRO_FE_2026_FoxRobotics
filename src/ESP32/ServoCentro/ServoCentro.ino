/*
 * ServoCentro — para encontrar el centro de la dirección. Mandas un ángulo y
 * el servo se queda ahí; cuando las llantas se vean rectas, ese es el
 * centroServo. El motor queda APAGADO. Monitor a 115200 (fin de línea:
 * "Nueva línea").
 *
 * PINES (iguales a PurePursuit.ino):
 *   Servo: GPIO 13
 *   Motor: PWMA=23, AIN1=18, AIN2=19 (apagado)
 *
 * Comandos:
 *   un número (0-180) + Enter -> ir a ese ángulo
 *   a / d -> -1 / +1 grado
 *   z / c -> -5 / +5 grados
 *
 * Arranca en 90. Para ver si está recto: mira las llantas desde arriba contra
 * una regla o una línea del piso, o empuja el carro 1-2 m y fíjate si se va
 * de lado.
 */

const int SERVO_PIN = 13;
const int freqServo = 50;
const int resServo  = 16;

const int PWMA = 23;
const int A1   = 18;
const int A2   = 19;

int angulo = 90;

void escribirServo(int a) {
  a = constrain(a, 0, 180);
  int pulso = map(a, 0, 180, 500, 2500);
  int duty  = (pulso * ((1 << resServo) - 1)) / 20000;
  ledcWrite(SERVO_PIN, duty);
}

void mover(int a) {
  angulo = constrain(a, 0, 180);
  escribirServo(angulo);
  Serial.printf("servo: %d   (pulso %d us)\n", angulo, (int)map(angulo, 0, 180, 500, 2500));
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Motor apagado
  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);
  pinMode(PWMA, OUTPUT); digitalWrite(PWMA, LOW);

  ledcAttach(SERVO_PIN, freqServo, resServo);
  Serial.println("ServoCentro: número+Enter = ángulo, a/d = -1/+1, z/c = -5/+5");
  mover(angulo);
}

String entrada;

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if      (ch == 'a') mover(angulo - 1);
    else if (ch == 'd') mover(angulo + 1);
    else if (ch == 'z') mover(angulo - 5);
    else if (ch == 'c') mover(angulo + 5);
    else if (isDigit(ch)) entrada += ch;
    else if ((ch == '\n' || ch == '\r') && entrada.length() > 0) {
      mover(entrada.toInt());
      entrada = "";
    }
  }
}
