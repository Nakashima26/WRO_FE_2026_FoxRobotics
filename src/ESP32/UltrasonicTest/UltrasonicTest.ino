/*
 * UltrasonicTest — prueba de lectura de los 3 HC-SR04 (izq, der, frontal).
 *
 * Pines = los de PurePursuit.ino (carro viejo). Si en el carro nuevo / PCB RevA
 * están en otros pines, cambiar solo el bloque de #define.
 *
 * Cada sensor se dispara por separado con SEPARACION_MS entre uno y otro para
 * que el eco de uno no lo lea el otro (crosstalk).
 *
 * Imprime una línea por ciclo:
 *   L=  45cm (2650us)  R= 200cm (----)  F=  80cm (4705us)
 * "----" = sin eco (timeout). Cada VENTANA ciclos imprime estadísticas por
 * sensor: min / max / media / desviación / % de lecturas sin eco.
 *
 * Comandos por Serial (115200):
 *   r  -> imprimir crudo on/off (para graficar con el Serial Plotter)
 *   l / d / f -> activar/desactivar izquierdo / derecho / frontal
 *   s  -> reiniciar estadísticas
 *
 * OJO: el ECHO del HC-SR04 sale a 5 V. El ESP32 necesita divisor (o un
 * HC-SR04P / RCWL a 3.3 V). Pin 35 es solo entrada: sirve para ECHO, no TRIG.
 */

// ── Pines ─────────────────────────────────────────────────────────────────────
#define TRIG_L 27
#define ECHO_L 32
#define TRIG_R 26
#define ECHO_R 35
#define TRIG_F 14
#define ECHO_F 33

// ── Parámetros ────────────────────────────────────────────────────────────────
const unsigned long TIMEOUT_US   = 12000;  // ≈ 2 m. PurePursuit usa 7000 (≈119 cm)
const unsigned long SEPARACION_MS = 25;    // espera entre sensores (crosstalk)
const int           VENTANA      = 40;     // ciclos por bloque de estadísticas

struct Sensor {
  const char *nombre;
  int trig, echo;
  bool activo;
  // estadísticas de la ventana
  int n, sinEco;
  float suma, suma2, minimo, maximo;
};

Sensor sensores[] = {
  {"L", TRIG_L, ECHO_L, true},
  {"R", TRIG_R, ECHO_R, true},
  {"F", TRIG_F, ECHO_F, true},
};
const int N_SENS = sizeof(sensores) / sizeof(sensores[0]);

bool modoCrudo = false;
int  ciclo     = 0;

void resetStats() {
  for (int i = 0; i < N_SENS; i++) {
    Sensor &s = sensores[i];
    s.n = 0; s.sinEco = 0;
    s.suma = 0; s.suma2 = 0;
    s.minimo = 9999; s.maximo = 0;
  }
  ciclo = 0;
}

// Devuelve la duración del eco en us (0 = sin eco).
unsigned long medirUs(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  return pulseIn(echo, HIGH, TIMEOUT_US);
}

void imprimirStats() {
  Serial.println(F("---- estadisticas ----"));
  for (int i = 0; i < N_SENS; i++) {
    Sensor &s = sensores[i];
    if (!s.activo) continue;
    int total = s.n + s.sinEco;
    Serial.print(s.nombre);
    if (s.n > 0) {
      float media = s.suma / s.n;
      float var   = s.suma2 / s.n - media * media;
      Serial.printf("  min=%5.1f  max=%5.1f  media=%5.1f  desv=%4.1f cm",
                    s.minimo, s.maximo, media, var > 0 ? sqrt(var) : 0.0f);
    } else {
      Serial.print(F("  (ninguna lectura valida)"));
    }
    Serial.printf("  sinEco=%d/%d (%.0f%%)\n", s.sinEco, total,
                  total ? 100.0f * s.sinEco / total : 0.0f);
  }
  Serial.println(F("----------------------"));
}

void leerComandos() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'r': modoCrudo = !modoCrudo; break;
      case 'l': sensores[0].activo = !sensores[0].activo; break;
      case 'd': sensores[1].activo = !sensores[1].activo; break;
      case 'f': sensores[2].activo = !sensores[2].activo; break;
      case 's': resetStats(); Serial.println(F("stats reiniciadas")); break;
      default: break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < N_SENS; i++) {
    pinMode(sensores[i].trig, OUTPUT);
    digitalWrite(sensores[i].trig, LOW);
    pinMode(sensores[i].echo, INPUT);
  }
  resetStats();
  delay(500);
  Serial.println(F("UltrasonicTest listo. Comandos: r=crudo  l/d/f=on/off sensor  s=reset stats"));
}

void loop() {
  leerComandos();

  for (int i = 0; i < N_SENS; i++) {
    Sensor &s = sensores[i];
    if (!s.activo) continue;

    unsigned long us = medirUs(s.trig, s.echo);
    float cm = us * 0.0343f / 2.0f;

    if (us == 0) {
      s.sinEco++;
    } else {
      s.n++;
      s.suma += cm; s.suma2 += cm * cm;
      if (cm < s.minimo) s.minimo = cm;
      if (cm > s.maximo) s.maximo = cm;
    }

    if (modoCrudo) {
      // Formato Serial Plotter: L:45.2 R:200.0 F:80.1  (sin eco -> 200)
      Serial.printf("%s:%.1f ", s.nombre, us ? cm : 200.0f);
    } else if (us) {
      Serial.printf("%s=%4.0fcm (%5luus)  ", s.nombre, cm, us);
    } else {
      Serial.printf("%s= 200cm ( ----)  ", s.nombre);
    }

    delay(SEPARACION_MS);
  }
  Serial.println();

  if (!modoCrudo && ++ciclo >= VENTANA) {
    imprimirStats();
    resetStats();
  }
}
