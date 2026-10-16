/*
 * RectaBnoTest — como DistanceTest, pero manteniendo el rumbo con el BNO085:
 * al prenderse espera ARRANQUE_AUTO_MS, guarda el rumbo inicial y avanza 1 m
 * corrigiendo la dirección con un PID sobre el error de yaw. Frena por
 * encoder y dice cuánto midió y con qué error de rumbo terminó.
 * Sin librerías. Monitor a 115200 (fin de línea: "Nueva línea").
 *
 * PINES:
 *   Driver TB6612: PWMA=23, AIN1=18, AIN2=19
 *   Servo:         GPIO 13
 *   Encoder:       azul (A) -> GPIO 34, morado (B) -> GPIO 36 (pull-ups 4.7k a 3.3V)
 *   BNO085 (UART-RVC): VIN=3.3V, GND, PS0=3.3V, PS1=GND, SDA -> GPIO 25
 *
 * ANTES DE PONERLO EN EL PISO — revisa el signo con PRUEBA_EN_MANO = true:
 *   el motor no arranca y el servo corrige en vivo. Gira el carro con la mano
 *   ~20° a la izquierda: las llantas deben girar a la DERECHA (para regresar),
 *   y al revés. Si corrigen hacia el mismo lado que lo giraste, cambia
 *   SIGNO_PID a -1. Luego pon PRUEBA_EN_MANO = false.
 *
 * Comandos (opcionales, si tienes el monitor abierto):
 *   g -> avanzar otra vez   x -> parar
 *   un número + Enter -> distancia objetivo en cm
 */

#include "driver/pulse_cnt.h"

// ---------------- Ajustes ----------------
const bool PRUEBA_EN_MANO = false;    // true = solo servo, sin motor (revisar el signo)
const int  SIGNO_PID      = -1;      // verificado en mano: con +1 corregía hacia el lado equivocado

float KpYaw = 2.0f;                  // grados de servo por grado de error (como KpGyro)
float KiYaw = 0.0f;
float KdYaw = 0.5f;                  // grados de servo por (grado/s) de error
const int SERVO_LIM = 30;            // máximo desvío del servo respecto al centro

int centroServo = 75;                // medido con ServoCentro (rango útil 0-156)
const int SERVO_MIN = 2;             // margen contra los topes mecánicos
const int SERVO_MAX = 154;

const int           PWM_ARRANQUE = 70;
const int           PWM_CRUCERO  = 120;
const int           PWM_LENTO    = 70;
const unsigned long RAMPA_MS     = 300;
const float         FRENADO_MM   = 150.0f;
const unsigned long TIMEOUT_MS   = 10000;
const unsigned long ARRANQUE_AUTO_MS = 2000;

// ---------------- Pines ----------------
const int PWMA = 23;
const int A1   = 18;
const int A2   = 19;
const int freqMotor = 1000;
const int resMotor  = 8;

const int SERVO_PIN = 13;
const int freqServo = 50;
const int resServo  = 16;

const int ENC_A = 34;
const int ENC_B = 36;
const int RVC_RX = 25;

// ---------------- Encoder ----------------
const float DIAMETRO_MM  = 42.7f;
const float CUENTAS_POR_VUELTA = 3.0f * 4 * 50.0f * 2;   // 12 CPR x4 x caja 50:1 x engranes 2:1 = 1200
const float MM_POR_CUENTA = PI * DIAMETRO_MM / CUENTAS_POR_VUELTA;
const int   SIGNO_ENC = -1;          // con azul->34 y morado->36, adelante cuenta negativo

const int8_t TABLA[16] = {  0, -1, +1,  2,
                           +1,  0,  2, -1,
                           -1,  2,  0, +1,
                            2, +1, -1,  0 };

volatile int32_t  cuenta = 0;
volatile uint32_t errores = 0;
volatile uint8_t  estado = 0;
// Diagnóstico: pasos en cada sentido y flancos por canal. Con la llanta
// girando siempre hacia adelante, los pasos "atrás" deberían ser ~0.
volatile uint32_t pasosMas = 0, pasosMenos = 0, flancosA = 0, flancosB = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isrEncoder() {
  uint8_t nuevo = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  portENTER_CRITICAL_ISR(&mux);
  uint8_t viejo = estado;
  int8_t paso = TABLA[(viejo << 2) | nuevo];
  if (paso == 2) errores++;
  else {
    cuenta += paso;
    if (paso > 0) pasosMas++;
    else if (paso < 0) pasosMenos++;
  }
  if ((viejo ^ nuevo) & 2) flancosA++;
  if ((viejo ^ nuevo) & 1) flancosB++;
  estado = nuevo;
  portEXIT_CRITICAL_ISR(&mux);
}

// La distancia la cuenta el PCNT (contador por hardware x4, filtro de glitches
// de 1 us). En el piso, con el motor jalando, la ISR se perdía ~25% de las
// cuentas (ISR 40.1 cm vs PCNT 52.8 cm vs cinta ~55 cm); la ISR se queda solo
// como comparación en el diagnóstico.
pcnt_unit_handle_t pcnt = NULL;

bool iniciarPcnt() {
  pcnt_unit_config_t ucfg = {};
  ucfg.low_limit  = -30000;
  ucfg.high_limit =  30000;
  ucfg.flags.accum_count = 1;          // acumula al desbordar los límites
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

// Cuentas del PCNT. Con esta configuración de canales cuenta con el mismo
// signo que la tabla de la ISR (adelante = negativo), así que usa SIGNO_ENC.
int32_t leePcnt() {
  int v = 0;
  pcnt_unit_get_count(pcnt, &v);
  return v;
}

// Distancia según la ISR (cm), solo para comparar.
float isrCm() {
  portENTER_CRITICAL(&mux);
  int32_t c = cuenta;
  portEXIT_CRITICAL(&mux);
  return SIGNO_ENC * c * MM_POR_CUENTA / 10.0f;
}

// "adelante / atrás / A / B / err / isr" de lo que lleva el recorrido.
void diagEncoder(char* s, size_t n) {
  portENTER_CRITICAL(&mux);
  uint32_t mas = pasosMas, menos = pasosMenos, fa = flancosA, fb = flancosB, e = errores;
  portEXIT_CRITICAL(&mux);
  uint32_t adel = (SIGNO_ENC > 0) ? mas : menos;
  uint32_t atras = (SIGNO_ENC > 0) ? menos : mas;
  snprintf(s, n, "isr:%.1f cm  pasos adel:%lu atras:%lu  flancos A:%lu B:%lu  err:%lu",
           isrCm(), (unsigned long)adel, (unsigned long)atras, (unsigned long)fa, (unsigned long)fb,
           (unsigned long)e);
}

float distanciaMm() {
  return SIGNO_ENC * leePcnt() * MM_POR_CUENTA;
}

// ---------------- BNO085 (UART-RVC) ----------------
uint8_t  buf[19];
int      nBuf = 0;
float    yaw = 0;
bool     yawNuevo = false;
uint32_t paquetes = 0;

float normaliza(float a) {
  while (a >  180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

void leerBno() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (nBuf == 0 && b != 0xAA) continue;
    if (nBuf == 1 && b != 0xAA) { nBuf = 0; continue; }
    buf[nBuf++] = b;
    if (nBuf < 19) continue;
    nBuf = 0;
    uint8_t suma = 0;
    for (int i = 2; i < 18; i++) suma += buf[i];
    if (suma != buf[18]) continue;
    yaw = (int16_t)(buf[3] | (buf[4] << 8)) * 0.01f;
    yawNuevo = true;
    paquetes++;
  }
}

// ---------------- Actuadores (solo escriben cuando el valor cambia) ----------------
// El servo solo lee un pulso cada 20 ms (50 Hz): escribirlo más seguido no
// sirve de nada y solo le quita tiempo a las interrupciones del encoder.
const uint32_t SERVO_PERIODO_MS = 20;
int servoActual = -1;
uint32_t tServoAnt = 0;
// Escribe ya, sin esperar el periodo (centro al arrancar y al terminar).
void escribirServoYa(int angulo) {
  angulo = constrain(angulo, SERVO_MIN, SERVO_MAX);
  if (angulo == servoActual) return;
  int pulso = map(angulo, 0, 180, 500, 2500);
  ledcWrite(SERVO_PIN, (pulso * ((1 << resServo) - 1)) / 20000);
  servoActual = angulo;
  tServoAnt = millis();
}

void escribirServo(int angulo) {
  if (millis() - tServoAnt < SERVO_PERIODO_MS) return;
  escribirServoYa(angulo);
}

int pwmActual = -1;
void adelante(int pwm) {
  if (pwm == pwmActual) return;
  if (pwmActual < 0) { digitalWrite(A1, HIGH); digitalWrite(A2, LOW); }
  ledcWrite(PWMA, pwm);
  pwmActual = pwm;
}

void frenar() {
  digitalWrite(A1, HIGH); digitalWrite(A2, HIGH);   // freno corto del TB6612
  ledcWrite(PWMA, 255);
  delay(300);
  ledcWrite(PWMA, 0);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);
  pwmActual = -1;
}

// ---------------- PID de rumbo ----------------
float yawObjetivo = 0;
float errAnt = 0, integral = 0;
uint32_t tPidAnt = 0;

void resetPid() {
  yawObjetivo = yaw;
  errAnt = 0; integral = 0;
  tPidAnt = millis();
}

// Se llama con cada paquete nuevo del BNO (~100 Hz).
void pid() {
  uint32_t ahora = millis();
  float dt = (ahora - tPidAnt) / 1000.0f;
  tPidAnt = ahora;
  if (dt <= 0 || dt > 0.2f) dt = 0.01f;

  float err = normaliza(yawObjetivo - yaw);
  integral = constrain(integral + err * dt, -20.0f, 20.0f);
  float deriv = (err - errAnt) / dt;
  errAnt = err;

  float out = KpYaw * err + KiYaw * integral + KdYaw * deriv;
  out = constrain(out, -SERVO_LIM, SERVO_LIM);
  escribirServo(centroServo + SIGNO_PID * (int)lroundf(out));
}

// ---------------- Recorrido ----------------
float objetivoMm = 1000.0f;
bool  andando = false;
unsigned long tArranque = 0;
char  resultado[300] = "";

void arrancar() {
  portENTER_CRITICAL(&mux);
  cuenta = 0; errores = 0;
  pasosMas = 0; pasosMenos = 0; flancosA = 0; flancosB = 0;
  portEXIT_CRITICAL(&mux);
  pcnt_unit_clear_count(pcnt);
  resetPid();
  andando = true;
  tArranque = millis();
  Serial.printf("-- avanzando %.0f cm, rumbo objetivo %.1f --\n", objetivoMm / 10.0f, yawObjetivo);
}

void terminar(const char* motivo) {
  frenar();
  escribirServoYa(centroServo);
  andando = false;
  delay(500);
  leerBno();
  char diag[120];
  diagEncoder(diag, sizeof(diag));
  snprintf(resultado, sizeof(resultado),
           "-- %s. Encoder midió: %.1f cm (objetivo %.0f cm)  rumbo final: %+.1f°  %s --",
           motivo, distanciaMm() / 10.0f, objetivoMm / 10.0f, normaliza(yaw - yawObjetivo), diag);
  Serial.println(resultado);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // PCNT primero: su configuración de pines apaga las interrupciones GPIO,
  // y attachInterrupt de abajo las vuelve a prender.
  if (!iniciarPcnt()) {
    Serial.println("PCNT no arrancó: sin él no hay distancia");
    while (true) delay(1000);
  }

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  estado = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrEncoder, CHANGE);

  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);
  ledcAttach(PWMA, freqMotor, resMotor);
  ledcAttach(SERVO_PIN, freqServo, resServo);
  escribirServoYa(centroServo);

  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N1, RVC_RX, -1);

  Serial.println("RectaBnoTest: esperando al BNO...");
  uint32_t t0 = millis();
  while (paquetes < 20 && millis() - t0 < 3000) leerBno();   // ~0.2 s de paquetes buenos
  if (paquetes < 20) {
    Serial.println("BNO NO responde: revisa VIN, GND, PS0=3.3V, PS1=GND y SDA -> GPIO 25");
    while (true) delay(1000);
  }

  if (PRUEBA_EN_MANO) {
    resetPid();
    Serial.println("PRUEBA_EN_MANO: motor apagado. Gira el carro con la mano y mira hacia dónde corrige el servo.");
    return;
  }

  Serial.printf("Objetivo: %.0f cm. Arranca solo en %lu s...\n", objetivoMm / 10.0f, ARRANQUE_AUTO_MS / 1000);
  uint32_t tEspera = millis();
  while (millis() - tEspera < ARRANQUE_AUTO_MS) leerBno();   // sigue leyendo para no llenar el buffer
  arrancar();
}

String entrada;

void loop() {
  leerBno();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'g' && !andando && !PRUEBA_EN_MANO) arrancar();
    else if (c == 'x' && andando) terminar("Parado a mano");
    else if (isDigit(c)) entrada += c;
    else if ((c == '\n' || c == '\r') && entrada.length() > 0) {
      objetivoMm = entrada.toInt() * 10.0f;
      entrada = "";
      Serial.printf("Objetivo: %.0f cm\n", objetivoMm / 10.0f);
    }
  }

  if (PRUEBA_EN_MANO) {
    if (yawNuevo) { yawNuevo = false; pid(); }
    static uint32_t tImp = 0;
    if (millis() - tImp >= 200) {
      tImp = millis();
      Serial.printf("error rumbo: %+6.1f°   servo: %3d (centro %d)\n",
                    normaliza(yawObjetivo - yaw), servoActual, centroServo);
    }
    return;
  }

  if (!andando) {
    static unsigned long tRepite = 0;
    if (resultado[0] && millis() - tRepite >= 2000) {
      tRepite = millis();
      Serial.println(resultado);
    }
    return;
  }

  if (yawNuevo) { yawNuevo = false; pid(); }

  float d = distanciaMm();
  unsigned long t = millis() - tArranque;

  if (d >= objetivoMm) { terminar("Llegó"); return; }
  if (t > TIMEOUT_MS)  { terminar("TIMEOUT"); return; }

  int pwm;
  if (objetivoMm - d < FRENADO_MM) pwm = PWM_LENTO;
  else if (t < RAMPA_MS)           pwm = map(t, 0, RAMPA_MS, PWM_ARRANQUE, PWM_CRUCERO);
  else                             pwm = PWM_CRUCERO;
  adelante(pwm);

  static uint32_t tImp = 0;
  if (millis() - tImp >= 200) {
    tImp = millis();
    char diag[120];
    diagEncoder(diag, sizeof(diag));
    Serial.printf("dist: %6.1f cm   rumbo: %+5.1f°   servo: %3d   %s\n",
                  d / 10.0f, normaliza(yawObjetivo - yaw), servoActual, diag);
  }
}
