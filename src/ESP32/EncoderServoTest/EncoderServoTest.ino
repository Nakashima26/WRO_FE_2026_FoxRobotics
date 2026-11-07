/*
 * EncoderServoTest — aísla por qué el encoder cuenta de menos cuando el servo
 * se mueve. Con la llanta EN EL AIRE, al prenderse espera 2 s, pone el motor
 * a PWM fijo y corre 3 fases de 4 s:
 *   A) servo quieto en el centro
 *   B) servo moviéndose (centro ±20, cambia cada 100 ms)
 *   C) servo quieto otra vez
 * El encoder se cuenta DOS veces a la vez, con los mismos pines:
 *   ISR  = interrupciones + tabla de cuadratura (como RectaBnoTest)
 *   PCNT = contador por hardware del ESP32, con filtro de glitches de 1 us
 * Al final imprime una tabla con las cuentas por segundo de cada fase.
 * Monitor a 115200.
 *
 * Cómo leerla (el motor va al mismo PWM en las 3 fases):
 *   - ISR baja en B pero PCNT no      -> problema de software de la ISR -> usar PCNT
 *   - ISR y PCNT bajan igual en B, con
 *     "atras" ~0 y pocos errores      -> el motor de verdad va más lento (el 5V
 *                                         del servo afecta al driver)
 *   - ISR baja en B con muchos "atras"
 *     o errores                       -> la señal del encoder llega sucia (eléctrico)
 *   - Nada baja en B                  -> el servo no es el problema
 *
 * PINES: driver PWMA=23, AIN1=18, AIN2=19 | servo 13 | encoder A=34, B=36
 */

#include "driver/pulse_cnt.h"

const int PWMA = 23;
const int A1   = 18;
const int A2   = 19;
const int SERVO_PIN = 13;
const int ENC_A = 34;
const int ENC_B = 36;

const int PWM_PRUEBA   = 80;
int       centroServo  = 75;
const int SERVO_AMPLITUD = 20;
const uint32_t FASE_MS = 4000;

// ---------------- Conteo por ISR (igual que los otros sketches) ----------------
const int8_t TABLA[16] = {  0, -1, +1,  2,
                           +1,  0,  2, -1,
                           -1,  2,  0, +1,
                            2, +1, -1,  0 };
volatile int32_t  cuenta = 0;
volatile uint32_t errores = 0, pasosMas = 0, pasosMenos = 0;
volatile uint8_t  estado = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isrEncoder() {
  uint8_t nuevo = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  portENTER_CRITICAL_ISR(&mux);
  int8_t paso = TABLA[(estado << 2) | nuevo];
  if (paso == 2) errores++;
  else {
    cuenta += paso;
    if (paso > 0) pasosMas++;
    else if (paso < 0) pasosMenos++;
  }
  estado = nuevo;
  portEXIT_CRITICAL_ISR(&mux);
}

// ---------------- Conteo por hardware (PCNT, x4) ----------------
pcnt_unit_handle_t pcnt = NULL;

bool iniciarPcnt() {
  pcnt_unit_config_t ucfg = {};
  ucfg.low_limit  = -30000;
  ucfg.high_limit =  30000;
  ucfg.flags.accum_count = 1;          // acumula al desbordar los límites
  if (pcnt_new_unit(&ucfg, &pcnt) != ESP_OK) return false;

  pcnt_glitch_filter_config_t fcfg = {};
  fcfg.max_glitch_ns = 1000;           // ignora pulsos de menos de 1 us
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

  // Decodificación x4 (la del ejemplo rotary_encoder de ESP-IDF).
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

int32_t leePcnt() {
  int v = 0;
  pcnt_unit_get_count(pcnt, &v);
  return v;
}

// ---------------- Servo ----------------
void escribirServo(int angulo) {
  angulo = constrain(angulo, 2, 154);
  int pulso = map(angulo, 0, 180, 500, 2500);
  ledcWrite(SERVO_PIN, (pulso * ((1 << 16) - 1)) / 20000);
}

// ---------------- Fases ----------------
struct Medida {
  int32_t  isr, pcnt;
  uint32_t mas, menos, err;
};

const char* NOMBRES[3] = {"A servo quieto  ", "B servo moviendo", "C servo quieto  "};
Medida ini[3], fin[3];
bool  terminado = false;

// Sin el tipo Medida en la firma: el IDE de Arduino declara las funciones
// arriba de todo, antes del struct, y no lo encontraría.
void foto(int f, bool esFin) {
  Medida& m = esFin ? fin[f] : ini[f];
  portENTER_CRITICAL(&mux);
  m.isr = cuenta; m.mas = pasosMas; m.menos = pasosMenos; m.err = errores;
  portEXIT_CRITICAL(&mux);
  m.pcnt = leePcnt();
}

void correrFase(int f) {
  foto(f, false);
  uint32_t t0 = millis(), tServo = 0;
  bool lado = false;
  while (millis() - t0 < FASE_MS) {
    if (f == 1 && millis() - tServo >= 100) {
      tServo = millis();
      lado = !lado;
      escribirServo(centroServo + (lado ? SERVO_AMPLITUD : -SERVO_AMPLITUD));
    }
    delay(1);
  }
  if (f == 1) escribirServo(centroServo);
  foto(f, true);
}

void imprimirTabla() {
  Serial.println("fase               ISR/s   PCNT/s   pasos atras   errores");
  for (int f = 0; f < 3; f++) {
    float s = FASE_MS / 1000.0f;
    Serial.printf("%s  %7.0f  %7.0f   %11lu   %7lu\n", NOMBRES[f],
                  abs(fin[f].isr - ini[f].isr) / s, abs(fin[f].pcnt - ini[f].pcnt) / s,
                  // con la llanta siempre hacia adelante, "atras" son los pasos del sentido minoritario
                  (unsigned long)min(fin[f].mas - ini[f].mas, fin[f].menos - ini[f].menos),
                  (unsigned long)(fin[f].err - ini[f].err));
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // PCNT primero: su configuración de pines apaga las interrupciones GPIO,
  // y attachInterrupt de abajo las vuelve a prender.
  if (!iniciarPcnt()) Serial.println("PCNT no arrancó");

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  estado = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrEncoder, CHANGE);

  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);
  ledcAttach(PWMA, 1000, 8);
  ledcWrite(PWMA, 0);
  ledcAttach(SERVO_PIN, 50, 16);
  escribirServo(centroServo);

  Serial.println("EncoderServoTest: llanta EN EL AIRE. Arranca en 2 s...");
  delay(2000);

  digitalWrite(A1, HIGH); digitalWrite(A2, LOW);
  ledcWrite(PWMA, PWM_PRUEBA);
  delay(1000);   // que el motor llegue a su velocidad

  for (int f = 0; f < 3; f++) {
    Serial.printf("fase %s...\n", NOMBRES[f]);
    correrFase(f);
  }

  ledcWrite(PWMA, 0);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);
  terminado = true;
  imprimirTabla();
}

void loop() {
  static uint32_t t = 0;
  if (terminado && millis() - t >= 3000) {
    t = millis();
    imprimirTabla();
  }
}
