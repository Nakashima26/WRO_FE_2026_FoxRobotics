/*
 * RectanguloTest — recorre un rectángulo con encoder (PCNT) + BNO085:
 *   recta 4.5 m, giro 90° der, recta 3.5 m, giro 90° der,
 *   recta 4.5 m, giro 90° der, recta 3.5 m, giro 90° der -> frena.
 * Cada recta mantiene el rumbo con el mismo PID de RectaBnoTest y mide la
 * distancia con el PCNT. En cada giro el rumbo objetivo se mueve 90° y el
 * servo va a tope (SERVO_TOPE_DER/IZQ) hasta que falten
 * GIRO_TOL_DEG para el rumbo nuevo, sin frenar.
 * Los rumbos objetivo son absolutos (inicial, -90, -180, -270), así que los
 * errores de un giro no se acumulan: la recta siguiente los corrige.
 * Al terminar imprime una tabla por tramo (se repite cada 3 s).
 * Monitor a 115200 (fin de línea: "Nueva línea").
 *
 * Ojo: el giro no es en el lugar, es un arco. La distancia de cada recta se
 * cuenta DESPUÉS de terminar el giro anterior, así que el rectángulo real
 * queda más grande que 4.5 x 3.5 por lo que avanza cada arco (la tabla dice
 * cuánto avanzó en cada giro, columna "dist").
 *
 * PINES (igual que RectaBnoTest):
 *   Driver TB6612: PWMA=23, AIN1=18, AIN2=19
 *   Servo:         GPIO 13
 *   Encoder:       azul (A) -> GPIO 34, morado (B) -> GPIO 36 (pull-ups 4.7k a 3.3V)
 *   BNO085 (UART-RVC): VIN=3.3V, GND, PS0=3.3V, PS1=GND, SDA -> GPIO 25
 *
 * ANTES DE PONERLO EN EL PISO — PRUEBA_EN_MANO = true (motor apagado):
 *   1) Gira el carro con la mano ~90° a la DERECHA (visto desde arriba, como
 *      las manecillas del reloj). "giro der" debe marcar ~+90. Si marca ~-90,
 *      cambia YAW_DERECHA de signo.
 *   2) Como en RectaBnoTest: al girarlo, las llantas deben corregir hacia el
 *      lado contrario (SIGNO_PID ya está verificado = -1).
 *   Luego PRUEBA_EN_MANO = false.
 *
 * Comandos: g -> correr otra vez   x -> parar
 */

#include "driver/pulse_cnt.h"

// ---------------- Ajustes ----------------
const bool PRUEBA_EN_MANO = false;   // true = solo servo, sin motor (revisar signos)
const int  SIGNO_PID      = -1;      // verificado en mano con RectaBnoTest
const int  YAW_DERECHA    = +1;      // +1: el yaw del BNO sube al girar a la derecha (con -1 giró a la izquierda en el piso)

float KpYaw = 2.0f;                  // grados de servo por grado de error
float KiYaw = 0.0f;
float KdYaw = 0.5f;                  // grados de servo por (grado/s) de error
const int SERVO_LIM      = 30;       // máximo desvío del servo en las rectas

int centroServo = 75;                // medido con ServoCentro (rango útil 0-156)
const int SERVO_MIN = 0;
const int SERVO_MAX = 156;
// Giros a tope: 0 = todo a la derecha, 156 = todo a la izquierda. Si el servo
// zumba/se calienta pegado al tope, súbelo/bájalo un par de grados.
const int SERVO_TOPE_DER = 0;
const int SERVO_TOPE_IZQ = 156;

// Rectángulo: largo, ancho (mm). Se recorre largo, giro, ancho, giro, x2.
const float LARGO_MM = 4500.0f;
const float ANCHO_MM = 3500.0f;
const float GIRO_DEG = 90.0f;
const float GIRO_TOL_DEG = 8.0f;     // suelta el tope cuando falten menos de esto para el rumbo nuevo;
                                     // lo que falte/sobre lo corrige el PID de la recta.
                                     // Si se pasa del giro, súbelo (anticipa más).

// Con 120 de crucero iba lentísimo. Si con 200 el PID de rumbo empieza a
// serpentear en las rectas, baja KdYaw/KpYaw o regresa el crucero a ~160.
const int           PWM_ARRANQUE = 90;
const int           PWM_CRUCERO  = 200;
const int           PWM_LENTO    = 160;  // últimos FRENADO_MM de cada recta (entra lento al giro)
const int           PWM_GIRO     = 180;
const unsigned long RAMPA_MS     = 500;  // más velocidad -> rampa más larga (sin golpe de corriente)
const float         FRENADO_MM   = 300.0f;  // a 200 hace falta más distancia para bajar a PWM_LENTO
const unsigned long TIMEOUT_RECTA_MS = 40000;
const unsigned long TIMEOUT_GIRO_MS  = 8000;
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

// ---------------- Encoder (PCNT) ----------------
const float DIAMETRO_MM  = 42.7f;
const float CUENTAS_POR_VUELTA = 3.0f * 4 * 50.0f * 2;   // 12 CPR x4 x caja 50:1 x engranes 2:1 = 1200
const float MM_POR_CUENTA = PI * DIAMETRO_MM / CUENTAS_POR_VUELTA;
const int   SIGNO_ENC = -1;          // adelante cuenta negativo (igual en ISR y PCNT)

// Contador por hardware x4 con filtro de glitches de 1 us. La ISR se perdía
// ~25% de las cuentas en el piso; el PCNT no. Los límites de ±30000 (~3.3 m)
// NO cortan la cuenta: accum_count + los watch points la acumulan, por eso
// las rectas de 4.5 m funcionan. No quitar esas dos cosas.
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

float distanciaMm() {
  int v = 0;
  pcnt_unit_get_count(pcnt, &v);
  return SIGNO_ENC * v * MM_POR_CUENTA;
}

void ceroDistancia() {
  pcnt_unit_clear_count(pcnt);
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
const uint32_t SERVO_PERIODO_MS = 20;   // el servo solo lee un pulso cada 20 ms
int servoActual = -1;
uint32_t tServoAnt = 0;

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
float rumboInicial = 0;
float yawObjetivo = 0;
float errAnt = 0, integral = 0;
uint32_t tPidAnt = 0;

float errorRumbo() { return normaliza(yawObjetivo - yaw); }

// Cambia el rumbo objetivo sin "patada" de la derivada: errAnt arranca con el
// error nuevo, así el salto de 90° no se lee como velocidad de giro.
void ponObjetivo(float objetivo) {
  yawObjetivo = normaliza(objetivo);
  errAnt = errorRumbo();
  integral = 0;
  tPidAnt = millis();
}

// Se llama con cada paquete nuevo del BNO (~100 Hz).
void pid() {
  uint32_t ahora = millis();
  float dt = (ahora - tPidAnt) / 1000.0f;
  tPidAnt = ahora;
  if (dt <= 0 || dt > 0.2f) dt = 0.01f;

  float err = errorRumbo();
  integral = constrain(integral + err * dt, -20.0f, 20.0f);
  float deriv = (err - errAnt) / dt;
  errAnt = err;

  float out = KpYaw * err + KiYaw * integral + KdYaw * deriv;
  out = constrain(out, -SERVO_LIM, SERVO_LIM);
  escribirServo(centroServo + SIGNO_PID * (int)lroundf(out));
}

// ---------------- Secuencia ----------------
enum Tipo { RECTA, GIRO };
struct Tramo {
  Tipo  tipo;
  float mm;          // solo rectas
  int   giros;       // cuántos giros de 90° lleva el rumbo objetivo en este tramo
};
const int N_TRAMOS = 8;
const Tramo TRAMOS[N_TRAMOS] = {
  {RECTA, LARGO_MM, 0}, {GIRO, 0, 1},
  {RECTA, ANCHO_MM, 1}, {GIRO, 0, 2},
  {RECTA, LARGO_MM, 2}, {GIRO, 0, 3},
  {RECTA, ANCHO_MM, 3}, {GIRO, 0, 4},
};

// Resultado de cada tramo para la tabla final.
struct Medida {
  float distMm;      // lo que avanzó el encoder en el tramo
  float errFin;      // error de rumbo al cerrar el tramo
  float errMax;      // peor |error| de rumbo durante la recta (no aplica a giros)
  uint32_t ms;
  bool  ok;
};
Medida medidas[N_TRAMOS];

int  tramo = -1;            // -1 = parado
bool terminado = false;
const char* motivoFin = "";
unsigned long tTramo = 0;

void empezarTramo(int i) {
  tramo = i;
  const Tramo& t = TRAMOS[i];
  ceroDistancia();
  tTramo = millis();
  ponObjetivo(rumboInicial + YAW_DERECHA * GIRO_DEG * t.giros);
  medidas[i] = {0, 0, 0, 0, false};
  if (t.tipo == RECTA)
    Serial.printf("-- tramo %d/%d: recta %.1f m, rumbo %+.0f° --\n", i + 1, N_TRAMOS, t.mm / 1000.0f, GIRO_DEG * t.giros);
  else
    Serial.printf("-- tramo %d/%d: giro 90° der -> rumbo %+.0f° --\n", i + 1, N_TRAMOS, GIRO_DEG * t.giros);
}

void cerrarTramo(bool ok) {
  Medida& m = medidas[tramo];
  m.distMm = distanciaMm();
  m.errFin = errorRumbo();
  m.ms = millis() - tTramo;
  m.ok = ok;
}

void arrancar() {
  leerBno();
  rumboInicial = yaw;
  terminado = false;
  for (int i = 0; i < N_TRAMOS; i++) medidas[i] = {0, 0, 0, 0, false};
  Serial.printf("-- rectángulo %.1f x %.1f m, rumbo inicial %.1f --\n",
                LARGO_MM / 1000.0f, ANCHO_MM / 1000.0f, rumboInicial);
  pwmActual = -1;
  empezarTramo(0);
}

void imprimirTabla() {
  Serial.printf("== %s ==\n", motivoFin);
  Serial.println("tramo  tipo    objetivo   dist(cm)  err fin  err max  tiempo(s)");
  for (int i = 0; i < N_TRAMOS; i++) {
    const Tramo& t = TRAMOS[i];
    const Medida& m = medidas[i];
    if (!m.ok && m.ms == 0) { Serial.printf("%3d    (no se corrió)\n", i + 1); continue; }
    if (t.tipo == RECTA)
      Serial.printf("%3d    recta   %6.0f cm  %8.1f  %+6.1f°  %6.1f°  %7.1f%s\n", i + 1, t.mm / 10.0f,
                    m.distMm / 10.0f, m.errFin, m.errMax, m.ms / 1000.0f, m.ok ? "" : "  <- cortado");
    else
      Serial.printf("%3d    giro    %+6.0f°    %8.1f  %+6.1f°  %6s   %7.1f%s\n", i + 1, GIRO_DEG * t.giros,
                    m.distMm / 10.0f, m.errFin, "-", m.ms / 1000.0f, m.ok ? "" : "  <- cortado");
  }
  // Tras 4 giros de 90° el carro debe apuntar igual que al salir.
  Serial.printf("rumbo final vs inicial: %+.1f°\n\n", normaliza(yaw - rumboInicial));
}

void terminar(const char* motivo) {
  if (tramo >= 0) cerrarTramo(false);
  frenar();
  escribirServoYa(centroServo);
  tramo = -1;
  terminado = true;
  motivoFin = motivo;
  delay(500);
  leerBno();
  imprimirTabla();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!iniciarPcnt()) {
    Serial.println("PCNT no arrancó: sin él no hay distancia");
    while (true) delay(1000);
  }

  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  digitalWrite(A1, LOW); digitalWrite(A2, LOW);
  ledcAttach(PWMA, freqMotor, resMotor);
  ledcAttach(SERVO_PIN, freqServo, resServo);
  escribirServoYa(centroServo);

  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N1, RVC_RX, -1);

  Serial.println("RectanguloTest: esperando al BNO...");
  uint32_t t0 = millis();
  while (paquetes < 20 && millis() - t0 < 3000) leerBno();
  if (paquetes < 20) {
    Serial.println("BNO NO responde: revisa VIN, GND, PS0=3.3V, PS1=GND y SDA -> GPIO 25");
    while (true) delay(1000);
  }

  if (PRUEBA_EN_MANO) {
    rumboInicial = yaw;
    ponObjetivo(rumboInicial);
    Serial.println("PRUEBA_EN_MANO: motor apagado. Gira el carro 90° a la DERECHA: 'giro der' debe dar ~+90.");
    return;
  }

  Serial.printf("Arranca solo en %lu s...\n", ARRANQUE_AUTO_MS / 1000);
  uint32_t tEspera = millis();
  while (millis() - tEspera < ARRANQUE_AUTO_MS) leerBno();
  arrancar();
}

void loop() {
  leerBno();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'g' && tramo < 0 && !PRUEBA_EN_MANO) arrancar();
    else if (c == 'x' && tramo >= 0) terminar("Parado a mano");
  }

  if (PRUEBA_EN_MANO) {
    if (yawNuevo) { yawNuevo = false; pid(); }
    static uint32_t tImp = 0;
    if (millis() - tImp >= 200) {
      tImp = millis();
      Serial.printf("giro der: %+6.1f°   error rumbo: %+6.1f°   servo: %3d (centro %d)\n",
                    YAW_DERECHA * normaliza(yaw - rumboInicial), errorRumbo(), servoActual, centroServo);
    }
    return;
  }

  if (tramo < 0) {
    static unsigned long tRepite = 0;
    if (terminado && millis() - tRepite >= 3000) {
      tRepite = millis();
      imprimirTabla();
    }
    return;
  }

  const Tramo& t = TRAMOS[tramo];
  // En las rectas manda el PID; en los giros el servo va a tope (abajo).
  if (yawNuevo) { yawNuevo = false; if (t.tipo == RECTA) pid(); }
  float d = distanciaMm();
  float err = errorRumbo();
  unsigned long dt = millis() - tTramo;

  if (t.tipo == RECTA) {
    // En los primeros 30 cm el carro todavía está saliendo del giro, no cuenta para el peor error.
    if (d > 300.0f) medidas[tramo].errMax = max(medidas[tramo].errMax, fabsf(err));

    if (d >= t.mm) {
      cerrarTramo(true);
      if (tramo + 1 < N_TRAMOS) empezarTramo(tramo + 1);
      return;
    }
    if (dt > TIMEOUT_RECTA_MS) { terminar("TIMEOUT en recta"); return; }

    int pwm;
    if (t.mm - d < FRENADO_MM)                 pwm = PWM_LENTO;
    else if (tramo == 0 && dt < RAMPA_MS)      pwm = map(dt, 0, RAMPA_MS, PWM_ARRANQUE, PWM_CRUCERO);
    else if (dt < RAMPA_MS)                    pwm = map(dt, 0, RAMPA_MS, PWM_GIRO, PWM_CRUCERO);
    else                                       pwm = PWM_CRUCERO;
    adelante(pwm);
  } else {
    if (fabsf(err) < GIRO_TOL_DEG) {
      cerrarTramo(true);
      if (tramo + 1 < N_TRAMOS) empezarTramo(tramo + 1);
      else { tramo = -1; terminar("Rectángulo completo"); }   // tramo=-1: ya se cerró como ok
      return;
    }
    if (dt > TIMEOUT_GIRO_MS) { terminar("TIMEOUT en giro (¿YAW_DERECHA al revés o servo topado?)"); return; }
    // Tope del lado que el PID corregiría: mismo signo que pid(), así
    // YAW_DERECHA decide el lado y SIGNO_PID la polaridad del servo.
    escribirServoYa(SIGNO_PID * err > 0 ? SERVO_TOPE_IZQ : SERVO_TOPE_DER);
    adelante(PWM_GIRO);
  }

  static uint32_t tImp = 0;
  if (millis() - tImp >= 200) {
    tImp = millis();
    Serial.printf("tramo %d %s  dist: %6.1f cm   error rumbo: %+6.1f°   servo: %3d\n",
                  tramo + 1, t.tipo == RECTA ? "recta" : "giro ", d / 10.0f, err, servoActual);
  }
}
