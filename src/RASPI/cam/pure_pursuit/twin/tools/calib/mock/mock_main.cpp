// Host mock de CalibFox: planta de motor 1er orden + encoder de cuadratura + reloj
// simulado. Protocolo: una linea de stdin = un comando; se ejecuta completo y se
// vacia stdout (el host espera "OK"/"ERR"). NO es el ESP32: valida logica, no timing.
#include <Arduino.h>
#include <deque>
#include <string>
#include <iostream>
#include <stdlib.h>

SerialMock Serial;
WireMock Wire;

static std::deque<char> g_rx;
int SerialMock::available() { return (int)g_rx.size(); }
int SerialMock::read() {
  if (g_rx.empty()) return -1;
  char c = g_rx.front(); g_rx.pop_front(); return (unsigned char)c;
}
void SerialMock::feed(const char *s) { while (*s) g_rx.push_back(*s++); }

static double envd(const char *n, double d) { const char *s = getenv(n); return s ? atof(s) : d; }

// ── planta ───────────────────────────────────────────────────────────────────
static uint64_t g_us = 1000000, g_plant_us = 1000000;
static int g_pin[64] = {0};
static uint32_t g_duty_motor = 0, g_duty_servo = 0;
static void (*g_isr)() = nullptr;
static double g_v = 0.0, g_cnt_true = 0.0;
static long g_cnt_int = 0;
static int g_phase = 0;                      // 0..3 sobre la secuencia 00,10,11,01
static const int GRAY[4] = {0, 2, 3, 1};     // AB
static unsigned g_rng = 12345;

static double P_K, P_PWM0, P_TAU, P_TAUC, P_CPMM, P_LOSS, P_G, P_SCRUB, P_REVK;

static double servo_deg() {
  double pulso = g_duty_servo * 20000.0 / 65535.0;
  return (pulso - 500.0) * 180.0 / 2000.0;
}

static void emit_count(int step) {          // step = +1 / -1
  if (P_LOSS > 0) {                         // flanco perdido: salta 2 estados, 1 sola ISR
    g_rng = g_rng * 1664525u + 1013904223u;
    if ((g_rng >> 8) / 16777216.0 < P_LOSS) step *= 2;
  }
  g_phase = ((g_phase + step) % 4 + 4) % 4;
  int ab = GRAY[g_phase];
  g_pin[34] = (ab >> 1) & 1;
  g_pin[39] = ab & 1;
  if (g_isr) g_isr();
}

static void integrate() {
  while (g_plant_us < g_us) {
    uint64_t sub = g_us - g_plant_us;
    if (sub > 1000) sub = 1000;
    double dt = sub / 1e6;
    int a1 = g_pin[18], a2 = g_pin[19];
    int dir = 0; double tau = P_TAUC; double target = 0.0;
    if (a1 && !a2) dir = 1; else if (!a1 && a2) dir = -1;
    if (dir != 0) {
      double duty = (double)g_duty_motor;
      double k = dir > 0 ? P_K : P_REVK;
      double vss = duty > P_PWM0 ? k * (duty - P_PWM0) : 0.0;
      double w = fabs(P_G * (90.0 - servo_deg()));
      double wmax = 44.5;
      vss *= 1.0 - P_SCRUB * (w / wmax) * (w / wmax);
      target = dir * vss;
      tau = (duty > 0) ? P_TAU : 0.05;      // PWM 0 con pines de direccion = freno
    } else if (a1 && a2) {
      tau = 0.05;
    }
    g_v += (target - g_v) * (dt / tau);
    g_cnt_true += g_v * dt * P_CPMM;
    while (g_cnt_int < (long)floor(g_cnt_true)) { g_cnt_int++; emit_count(+1); }
    while (g_cnt_int > (long)ceil(g_cnt_true))  { g_cnt_int--; emit_count(-1); }
    g_plant_us += sub;
  }
}

void mock_advance_us(unsigned long us) { g_us += us; integrate(); }

void pinMode(int, int) {}
void digitalWrite(int pin, int level) { integrate(); g_pin[pin] = level; }
int digitalRead(int pin) { return g_pin[pin]; }
void ledcAttach(int, int, int) {}
void ledcWrite(int pin, uint32_t duty) {
  integrate();
  if (pin == 23) g_duty_motor = duty;
  else if (pin == 13) g_duty_servo = duty;
}
void attachInterrupt(int, void (*fn)(), int) { g_isr = fn; }
unsigned long micros() { mock_advance_us(4); return (unsigned long)g_us; }
unsigned long millis() { return (unsigned long)(g_us / 1000); }
void delay(unsigned long ms) { mock_advance_us(ms * 1000UL); }
void delayMicroseconds(unsigned int us) { mock_advance_us(us); }
long pulseIn(int pin, int, unsigned long timeout_us) {
  if (pin == 33) { mock_advance_us(timeout_us); return 0; }      // frontal sin eco
  long dur = (pin == 32) ? 1160 : 580;                           // ~20 / ~10 cm
  mock_advance_us(dur + 300);
  return dur;
}

int main() {
  P_K = envd("MOCK_K", 4.2); P_PWM0 = envd("MOCK_PWM0", 18.0);
  P_TAU = envd("MOCK_TAU", 0.18); P_TAUC = envd("MOCK_TAUC", 0.10);
  P_CPMM = envd("MOCK_CPMM", 9.0); P_LOSS = envd("MOCK_LOSS", 0.0);
  P_G = envd("MOCK_G", 0.55); P_SCRUB = envd("MOCK_SCRUB", 0.25);
  P_REVK = envd("MOCK_REV_K", 3.9);
  setup();
  fflush(stdout);
  std::string line;
  while (std::getline(std::cin, line)) {
    Serial.feed(line.c_str());
    Serial.feed("\n");
    while (Serial.available() > 0) loop();
    fflush(stdout);
  }
  return 0;
}
