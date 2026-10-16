/*
 * DistanceTest — al prenderse espera ARRANQUE_AUTO_MS (2 s) y avanza solo en
 * línea recta una distancia fija (1 m por defecto) contando con el encoder,
 * frena y dice cuánto midió el encoder. Compara ESE número con la cinta
 * métrica. Sin librerías. Monitor a 115200 (fin de línea: "Nueva línea").
 * El resultado se repite cada 2 s en el monitor, por si lo abres después.
 *
 * PINES:
 *   Driver TB6612: PWMA=23, AIN1=18, AIN2=19 (igual que PurePursuit)
 *   Servo:         GPIO 13
 *   Encoder:       azul (A) -> GPIO 34, morado (B) -> GPIO 36,
 *                  pull-ups 4.7k a 3.3V, café=5V, verde=GND
 *
 * Comandos:
 *   g -> avanzar la distancia objetivo
 *   x -> parar ya
 *   un número + Enter -> distancia objetivo en cm (p. ej. 200)
 *   a / d -> centro del servo -1 / +1 (si el carro se va de lado)
 *
 * Cómo medir: marca en el piso la defensa delantera, manda 'g' y espera a que
 * imprima "Encoder midió". Mide con la cinta dónde quedó la misma defensa.
 * Lo que importa es que la cinta y "Encoder midió" coincidan; que se pase un
 * poco del objetivo por inercia es normal.
 */

const int PWMA = 23;
const int A1   = 18;
const int A2   = 19;
const int freqMotor = 1000;
const int resMotor  = 8;

const int SERVO_PIN = 13;
const int freqServo = 50;
const int resServo  = 16;
int centroServo = 75;   // medido con ServoCentro (rango útil 0-156)
const bool USAR_SERVO = true;   // false = servo sin señal (para descartar que su ruido afecte al encoder)

const int ENC_A = 34;
const int ENC_B = 36;

const float DIAMETRO_MM  = 42.7f;
const float PULSOS_MOTOR = 3.0f;        // 12 CPR = 3 pulsos por canal
const float RELACION     = 50.0f * 2;   // caja 50:1 x engranes 15T/30T
const float CUENTAS_POR_VUELTA = PULSOS_MOTOR * 4 * RELACION;   // 1200
const float MM_POR_CUENTA = PI * DIAMETRO_MM / CUENTAS_POR_VUELTA;
const int   SIGNO = -1;                 // con azul->34 y morado->36, adelante cuenta negativo

// Perfil: arranca suave, va a PWM_CRUCERO y baja a PWM_LENTO en los últimos
// FRENADO_MM para pasarse lo menos posible.
const int           PWM_ARRANQUE = 70;
const int           PWM_CRUCERO  = 120;
const int           PWM_LENTO    = 70;
const unsigned long RAMPA_MS     = 300;
const float         FRENADO_MM   = 150.0f;
const unsigned long TIMEOUT_MS   = 10000;   // por si el encoder no cuenta
const unsigned long ARRANQUE_AUTO_MS = 2000; // espera tras prender antes de avanzar solo

// Tabla de cuadratura: índice = (estado anterior << 2) | estado nuevo, con
// estado = (A << 1) | B. 0 = sin cambio, ±1 = un paso, 2 = salto imposible.
const int8_t TABLA[16] = {  0, -1, +1,  2,
                           +1,  0,  2, -1,
                           -1,  2,  0, +1,
                            2, +1, -1,  0 };

volatile int32_t  cuenta = 0;
volatile uint32_t errores = 0;
volatile uint8_t  estado = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isrEncoder() {
  uint8_t nuevo = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  portENTER_CRITICAL_ISR(&mux);
  int8_t paso = TABLA[(estado << 2) | nuevo];
  if (paso == 2) errores++;
  else cuenta += paso;
  estado = nuevo;
  portEXIT_CRITICAL_ISR(&mux);
}

float distanciaMm() {
  portENTER_CRITICAL(&mux);
  int32_t c = cuenta;
  portEXIT_CRITICAL(&mux);
  return SIGNO * c * MM_POR_CUENTA;
}

void escribirServo(int angulo) {
  if (!USAR_SERVO) return;
  angulo = constrain(angulo, 0, 180);
  int pulso = map(angulo, 0, 180, 500, 2500);
  int duty  = (pulso * ((1 << resServo) - 1)) / 20000;
  ledcWrite(SERVO_PIN, duty);
}

// Solo toca los pines cuando el PWM cambia: escribir el LEDC en cada vuelta del
// loop (miles de veces por segundo) era la diferencia con MotorTest.
int pwmActual = -1;

void adelante(int pwm) {
  if (pwm == pwmActual) return;
  if (pwmActual < 0) { digitalWrite(A1, HIGH); digitalWrite(A2, LOW); }
  ledcWrite(PWMA, pwm);
  pwmActual = pwm;
}

// Freno corto del TB6612 (AIN1 = AIN2 = HIGH) y luego coast.
void frenar() {
  digitalWrite(A1, HIGH); digitalWrite(A2, HIGH);
  ledcWrite(PWMA, 255);
  delay(300);
  ledcWrite(PWMA, 0);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);
  pwmActual = -1;
}

// ---------------- Recorrido ----------------
float objetivoMm = 400.0f;
bool  andando = false;
unsigned long tArranque = 0;
char  resultado[120] = "";   // último resultado, se repite en el monitor

void arrancar() {
  portENTER_CRITICAL(&mux);
  cuenta = 0; errores = 0;
  portEXIT_CRITICAL(&mux);
  escribirServo(centroServo);
  andando = true;
  tArranque = millis();
  Serial.printf("-- avanzando %.0f cm --\n", objetivoMm / 10.0f);
}

void terminar(const char* motivo) {
  frenar();
  andando = false;
  delay(500);   // que se detenga del todo antes de leer
  portENTER_CRITICAL(&mux);
  uint32_t e = errores;
  portEXIT_CRITICAL(&mux);
  snprintf(resultado, sizeof(resultado), "-- %s. Encoder midió: %.1f cm (objetivo %.0f cm)  errores: %lu --",
           motivo, distanciaMm() / 10.0f, objetivoMm / 10.0f, (unsigned long)e);
  Serial.println(resultado);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  estado = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrEncoder, CHANGE);

  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);
  ledcAttach(PWMA, freqMotor, resMotor);
  if (USAR_SERVO) ledcAttach(SERVO_PIN, freqServo, resServo);
  escribirServo(centroServo);

  Serial.println("DistanceTest: g avanzar, x parar, número+Enter = objetivo en cm, a/d centro servo");
  Serial.printf("Objetivo: %.0f cm. Arranca solo en %lu s...\n",
                objetivoMm / 10.0f, ARRANQUE_AUTO_MS / 1000);
  delay(ARRANQUE_AUTO_MS);
  arrancar();
}

String entrada;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'g' && !andando) arrancar();
    else if (c == 'x' && andando) terminar("Parado a mano");
    else if (c == 'a' || c == 'd') {
      centroServo += (c == 'a') ? -1 : 1;
      escribirServo(centroServo);
      Serial.printf("centro servo: %d\n", centroServo);
    }
    else if (isDigit(c)) entrada += c;
    else if ((c == '\n' || c == '\r') && entrada.length() > 0) {
      objetivoMm = entrada.toInt() * 10.0f;
      entrada = "";
      Serial.printf("Objetivo: %.0f cm\n", objetivoMm / 10.0f);
    }
  }

  if (!andando) {
    static unsigned long tRepite = 0;
    if (resultado[0] && millis() - tRepite >= 2000) {
      tRepite = millis();
      Serial.println(resultado);
    }
    return;
  }

  float d = distanciaMm();
  unsigned long t = millis() - tArranque;

  if (d >= objetivoMm)    { terminar("Llegó");   return; }
  if (t > TIMEOUT_MS)     { terminar("TIMEOUT: ¿el encoder cuenta?"); return; }

  int pwm;
  if (objetivoMm - d < FRENADO_MM) pwm = PWM_LENTO;
  else if (t < RAMPA_MS)           pwm = map(t, 0, RAMPA_MS, PWM_ARRANQUE, PWM_CRUCERO);
  else                             pwm = PWM_CRUCERO;
  adelante(pwm);
}
