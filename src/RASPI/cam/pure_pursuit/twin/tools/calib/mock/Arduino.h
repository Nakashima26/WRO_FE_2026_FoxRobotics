// Mock minimo de Arduino/ESP32 para compilar CalibFox.ino en el host (zig c++/g++)
// y probar su parser de comandos, el CSV y los scripts de captura SIN el carro.
// La planta (motor + encoder) vive en mock_main.cpp; parametros por variables de
// entorno MOCK_K, MOCK_PWM0, MOCK_TAU, MOCK_TAUC, MOCK_CPMM, MOCK_LOSS, MOCK_G,
// MOCK_SCRUB, MOCK_REV_K. NO modela el ESP32 real (tiempos de Serial, I2C, ISR).
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <type_traits>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define CHANGE 4
#define IRAM_ATTR
#define ESP_ARDUINO_VERSION_MAJOR 3

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define digitalPinToInterrupt(p) (p)

#ifdef _WIN32
#define strtok_r strtok_s
#endif

#define constrain(a, lo, hi) ((a) < (lo) ? (lo) : ((a) > (hi) ? (hi) : (a)))
static inline long map(long x, long a, long b, long c, long d) {
  return (x - a) * (d - c) / (b - a) + c;
}

void pinMode(int pin, int mode);
void digitalWrite(int pin, int level);
int digitalRead(int pin);
void ledcAttach(int pin, int freq, int res);
void ledcWrite(int pin, uint32_t duty);
void attachInterrupt(int pin, void (*fn)(), int mode);
unsigned long micros();
unsigned long millis();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
long pulseIn(int pin, int level, unsigned long timeout_us);

struct SerialMock {
  void setTxBufferSize(size_t) {}
  void begin(unsigned long) {}
  int available();
  int read();
  void feed(const char *s);

  size_t print(const char *s) { fputs(s, stdout); return strlen(s); }
  size_t print(char c) { fputc(c, stdout); return 1; }
  size_t print(double d, int digits = 2) { printf("%.*f", digits, d); return 1; }
  template <class T>
  typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, char>::value, size_t>::type
  print(T v) {
    if (std::is_signed<T>::value) printf("%lld", (long long)v);
    else printf("%llu", (unsigned long long)v);
    return 1;
  }
  size_t println() { fputc('\n', stdout); return 1; }
  template <class T> size_t println(T v) { print(v); fputc('\n', stdout); return 1; }
  size_t println(double d, int digits) { print(d, digits); fputc('\n', stdout); return 1; }
};
extern SerialMock Serial;

struct WireMock {
  void begin() {}
  void setClock(uint32_t) {}
};
extern WireMock Wire;

void setup();
void loop();

void attachInterruptArg(int pin, void (*fn)(void *), void *arg, int mode);
