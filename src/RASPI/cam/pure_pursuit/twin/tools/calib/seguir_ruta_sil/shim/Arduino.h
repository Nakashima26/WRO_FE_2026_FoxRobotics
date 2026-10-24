// Lo mínimo del core de Arduino/ESP32 para compilar SeguirRuta.ino en la PC.
// El tiempo lo lleva sr_core.cpp: cada millis() avanza SR_US_POR_MILLIS µs (el loop
// del ESP32 no tiene delay, y `terminar()` espera con millis() en un while).
#pragma once

#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <type_traits>

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define SERIAL_8N1 0x800001c
#define ESP_OK 0
#ifndef PI
#define PI 3.14159265358979323846
#endif
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

typedef int esp_err_t;

template <typename A, typename B>
inline typename std::common_type<A, B>::type min(A a, B b) { return a < b ? a : b; }
template <typename A, typename B>
inline typename std::common_type<A, B>::type max(A a, B b) { return a > b ? a : b; }
template <typename T, typename L, typename H>
inline T constrain(T v, L lo, H hi) { return v < lo ? (T)lo : (v > hi ? (T)hi : v); }

unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
long map(long x, long in_min, long in_max, long out_min, long out_max);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution);
void ledcWrite(uint8_t pin, uint32_t duty);

class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t *buf, size_t n) {
    for (size_t i = 0; i < n; i++) write(buf[i]);
    return n;
  }
  size_t print(const char *s) { return write((const uint8_t *)s, strlen(s)); }
  size_t println(const char *s) { return print(s) + print("\n"); }
  size_t println() { return print("\n"); }
  size_t printf(const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return 0;
    return write((const uint8_t *)buf, (size_t)min(n, (int)sizeof(buf) - 1));
  }
};

class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
};

// Serial (USB), Serial1 (BNO RVC) y SerialBT: entrada por cola, salida a un búfer
// que Python lee (solo el USB; el BT repetiría lo mismo).
class SrSerial : public Stream {
 public:
  explicit SrSerial(int id) : id_(id) {}
  void begin(unsigned long) {}
  void begin(unsigned long, uint32_t, int, int) {}
  bool begin(const char *) { return true; }
  void setRxBufferSize(size_t) {}
  int available() override;
  int read() override;
  size_t write(uint8_t c) override;
  using Print::write;

 private:
  int id_;
};

typedef SrSerial HardwareSerial;
extern SrSerial Serial;
extern SrSerial Serial1;
