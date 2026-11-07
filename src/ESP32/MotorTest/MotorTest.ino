/*
 * MotorTest — mueve el motor desde el monitor serie y mide qué tan rápido va
 * con el encoder (contador por hardware PCNT: la ISR se perdía ~25% de las
 * cuentas con el motor cargado). Monitor a 115200 (fin de línea: "Nueva línea").
 *
 * PINES:
 *   Driver TB6612: PWMA=23, AIN1=18, AIN2=19 (igual que PurePursuit)
 *   Encoder:       café=5V, verde=GND,
 *                  azul (A) -> GPIO 34, morado (B) -> GPIO 36, ambos con
 *                  pull-up 4.7k a 3.3V
 *
 * Comandos:
 *   w -> adelante   s -> reversa   x -> parar (coast)
 *   + / - -> PWM ±10
 *   un número (0-255) + Enter -> ese PWM
 *   r -> distancia y velocidad máxima en 0
 *
 * Imprime cada 100 ms: PWM, rpm de la llanta, rpm del eje del motor,
 * velocidad (cm/s), la máxima desde el último 'r' y distancia (cm). Con la
 * llanta en el aire, adelante debe dar rpm positivas; si salen negativas,
 * cambia SIGNO.
 *
 * Velocidad máxima real: en el PISO (con carga), 'r', luego "255" + Enter y
 * 'w'; deja que acelere ~1 m y lee "max". En el aire sale más alta (sin carga).
 */

#include "driver/pulse_cnt.h"

const int PWMA = 23;
const int A1   = 18;
const int A2   = 19;
const int freqMotor = 1000;
const int resMotor  = 8;
const int PWM_MAX   = 255;      // bájalo si el motor no aguanta el voltaje de la batería

const int ENC_A = 34;
const int ENC_B = 36;

const float DIAMETRO_MM  = 42.7f;
const float PULSOS_MOTOR = 3.0f;        // 12 CPR = 3 pulsos por canal
const float RELACION     = 50.092f * 1.25;   // caja 50:1 x engranes 15T/30T (18T/27T -> 50.0f * 1.5)
const float CUENTAS_POR_VUELTA = PULSOS_MOTOR * 4 * RELACION;   // 1200 por vuelta de llanta
int SIGNO = -1;                         // con azul->34 y morado->36, adelante cuenta negativo

// ---------------- Encoder (PCNT x4, filtro de glitches de 1 us) ----------------
// Los límites de ±30000 no cortan la cuenta: accum_count + los watch points la
// acumulan al desbordar.
pcnt_unit_handle_t pcnt = NULL;

bool iniciarPcnt() {
  pcnt_unit_config_t ucfg = {};
  ucfg.low_limit  = -30000;
  ucfg.high_limit =  30000;
  ucfg.flags.accum_count = 1;
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

int32_t leeCuenta() {
  int v = 0;
  pcnt_unit_get_count(pcnt, &v);
  return v;
}

// ---------------- Motor ----------------
int pwm = 80;
int dir = 0;   // 1 adelante, -1 reversa, 0 coast

void aplicaMotor() {
  if (dir == 0) {
    ledcWrite(PWMA, 0);
    digitalWrite(A1, LOW); digitalWrite(A2, LOW);
    return;
  }
  digitalWrite(A1, dir > 0 ? HIGH : LOW);
  digitalWrite(A2, dir > 0 ? LOW : HIGH);
  ledcWrite(PWMA, pwm);
}

// Coast 300 ms antes de invertir, como MANIOBRA_FRENO_MS en PurePursuit.
void cambiaDir(int d) {
  if (dir != 0 && d != dir) { dir = 0; aplicaMotor(); delay(300); }
  dir = d;
  aplicaMotor();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!iniciarPcnt()) {
    Serial.println("PCNT no arrancó: sin él no hay encoder");
    while (true) delay(1000);
  }

  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  ledcAttach(PWMA, freqMotor, resMotor);
  cambiaDir(0);

  Serial.println("MotorTest: w/s/x motor, +/- PWM, número+Enter = PWM, r distancia y máx en 0");
}

String  entrada;
int32_t cuentaAnt = 0;   // para las rpm
float   cmsMax = 0;      // velocidad máxima (cm/s, en valor absoluto) desde el último 'r'

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') cambiaDir(1);
    else if (c == 's') cambiaDir(-1);
    else if (c == 'x') cambiaDir(0);
    else if (c == '+') { pwm = min(pwm + 10, PWM_MAX); aplicaMotor(); }
    else if (c == '-') { pwm = max(pwm - 10, 0);       aplicaMotor(); }
    else if (c == 'r') {
      pcnt_unit_clear_count(pcnt);
      cuentaAnt = 0;
      cmsMax = 0;
      Serial.println("-- distancia y máx en 0 --");
    }
    else if (isDigit(c)) entrada += c;
    else if ((c == '\n' || c == '\r') && entrada.length() > 0) {
      pwm = constrain(entrada.toInt(), 0, PWM_MAX);
      entrada = "";
      aplicaMotor();
    }
  }

  static uint32_t tImprime = 0;
  uint32_t ahora = millis();
  if (ahora - tImprime >= 100) {
    int32_t c = leeCuenta();

    float dtMin = (ahora - tImprime) / 60000.0f;
    float rpm   = SIGNO * (c - cuentaAnt) / CUENTAS_POR_VUELTA / dtMin;
    float cms   = rpm * PI * DIAMETRO_MM / 10.0f / 60.0f;
    float cm    = SIGNO * c / CUENTAS_POR_VUELTA * PI * DIAMETRO_MM / 10.0f;
    cuentaAnt = c;
    tImprime  = ahora;
    cmsMax    = max(cmsMax, fabsf(cms));

    Serial.printf("pwm:%3d dir:%2d   llanta:%6.1f rpm   motor:%6.0f rpm   %6.1f cm/s (max %5.1f)   dist:%7.1f cm\n",
                  pwm, dir, rpm, rpm * RELACION, cms, cmsMax, cm);
  }
}
