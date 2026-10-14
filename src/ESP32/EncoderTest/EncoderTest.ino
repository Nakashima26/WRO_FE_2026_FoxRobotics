/*
 * EncoderTest — muestra la distancia recorrida (cm) y las RPM de la llanta
 * con el encoder del motor. La distancia sube al empujar el robot hacia
 * adelante y baja hacia atrás. Sin librerías. Monitor a 115200.
 *
 * Motorreductor N20 50:1 con encoder Hall de 12 CPR (RobotShop):
 *   12 CPR = 3 pulsos por canal -> 3 x 4 (cuadratura) x 50 x 2 (engranes)
 *   = 1200 cuentas por vuelta de llanta.
 *
 * PINES (colores del cable del motor):
 *   Café     -> 5V         (+ del encoder; pide mínimo 3.5V)
 *   Verde    -> GND
 *   Azul     -> GPIO 34    (señal A) + pull-up 4.7k a 3.3V
 *   Morado   -> GPIO 36    (señal B, "SP") + pull-up 4.7k a 3.3V
 *   Negro / Rojo son el motor: NUNCA al ESP.
 *   Las salidas son de colector abierto y 34/36 no tienen pull-up interna:
 *   sin las 4.7k a 3.3V (no a 5V) no se lee nada.
 *
 * Comandos:
 *   r -> distancia en 0
 *   c -> comprobar: marca la llanta, manda 'r', gírala VUELTAS_CALIB vueltas
 *        exactas en el sentido de AVANZAR y manda 'c'. Compara las cuentas
 *        medidas con las ~1200 esperadas y dice si hay que cambiar SIGNO.
 *
 * Imprime cada 100 ms la distancia (cm) y las RPM de la llanta. Si aparece
 * "errores", el encoder está metiendo ruido o se saltó pasos.
 */

const int ENC_A = 34;
const int ENC_B = 36;

const float DIAMETRO_MM   = 42.7f;
const float PULSOS_MOTOR  = 3.0f;        // 12 CPR = 3 pulsos por canal por vuelta del motor
const float RELACION      = 50.0f * 2;   // caja 50:1 x engranes 15T/30T
const float VUELTAS_CALIB = 1.0f;        // más vueltas = comprobación más precisa

const float CUENTAS_POR_VUELTA = PULSOS_MOTOR * 4 * RELACION;   // 1200
int SIGNO = 1;                       // -1 si adelante baja (lo dice 'c')

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

// Una sola ISR para los dos canales: lee ambos y decide con la tabla.
void IRAM_ATTR isrEncoder() {
  uint8_t nuevo = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  portENTER_CRITICAL_ISR(&mux);
  int8_t paso = TABLA[(estado << 2) | nuevo];
  if (paso == 2) errores++;
  else cuenta += paso;
  estado = nuevo;
  portEXIT_CRITICAL_ISR(&mux);
}

int32_t leeCuenta() {
  portENTER_CRITICAL(&mux);
  int32_t c = cuenta;
  portEXIT_CRITICAL(&mux);
  return c;
}

int32_t cuentaAnt = 0;   // para las RPM

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  estado = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrEncoder, CHANGE);

  Serial.println("EncoderTest: r -> distancia en 0 | c -> comprobar con vueltas de llanta");
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'r') {
      portENTER_CRITICAL(&mux);
      cuenta = 0; errores = 0;
      portEXIT_CRITICAL(&mux);
      cuentaAnt = 0;
      Serial.println("-- distancia en 0 --");
    }
    if (ch == 'c') {
      int32_t c = leeCuenta();
      if (abs(c) < 10) {
        Serial.println("-- muy pocas cuentas: manda 'r', gira la llanta y luego 'c' --");
      } else {
        float medidas = abs(c) / VUELTAS_CALIB;
        Serial.printf("-- %.0f cuentas por vuelta (esperadas %.0f, %+.1f%%) --\n",
                      medidas, CUENTAS_POR_VUELTA,
                      100.0f * (medidas - CUENTAS_POR_VUELTA) / CUENTAS_POR_VUELTA);
        Serial.printf("   -> %.1f pulsos por vuelta del motor (ficha: %.0f)\n",
                      medidas / (4 * RELACION), PULSOS_MOTOR);
        if ((c > 0) != (SIGNO > 0)) {
          SIGNO = -SIGNO;
          Serial.printf("   adelante contaba al revés: SIGNO = %d (cámbialo arriba)\n", SIGNO);
        }
      }
    }
  }

  static uint32_t tImprime = 0;
  uint32_t ahora = millis();
  if (ahora - tImprime >= 100) {
    int32_t c = leeCuenta();
    portENTER_CRITICAL(&mux);
    uint32_t e = errores;
    portEXIT_CRITICAL(&mux);

    int32_t delta = c - cuentaAnt;
    float dtMin = (ahora - tImprime) / 60000.0f;
    cuentaAnt = c;
    tImprime = ahora;

    float vueltas = SIGNO * c / CUENTAS_POR_VUELTA;
    float cm      = vueltas * PI * DIAMETRO_MM / 10.0f;
    float rpm     = SIGNO * delta / CUENTAS_POR_VUELTA / dtMin;
    Serial.printf("distancia: %8.1f cm   rpm: %7.1f", cm, rpm);
    if (e > 0) Serial.printf("   errores: %lu", (unsigned long)e);
    Serial.println();
  }
}
