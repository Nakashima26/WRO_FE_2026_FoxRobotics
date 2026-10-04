#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <type_traits>

// Evitar <cmath>: en libc++ reexporta abs/asin y choca con los overloads Arduino.

// ── Pines / constantes Arduino ───────────────────────────────────────────────
#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define SERIAL_8N1 0x800001c

#define DEC 10
#define HEX 16

#ifndef PI
#define PI 3.14159265358979323846
#endif

// F() en SIL: cadena normal (sin flash)
#define F(x) x

// min/max: libc++ ya trae abs(float) vía <string>; no redeclarar.

template <typename A, typename B>
inline auto arduino_min(A a, B b) -> typename std::common_type<A, B>::type {
  return a < b ? a : b;
}
template <typename A, typename B>
inline auto arduino_max(A a, B b) -> typename std::common_type<A, B>::type {
  return a > b ? a : b;
}
#define min(a, b) arduino_min((a), (b))
#define max(a, b) arduino_max((a), (b))

template <typename T, typename L, typename H>
inline T constrain(T val, L lo, H hi) {
  if (val < (T)lo) return (T)lo;
  if (val > (T)hi) return (T)hi;
  return val;
}

float degrees(float rad);

unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000UL);

bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution);
void ledcWrite(uint8_t pin, uint32_t duty);

long map(long x, long in_min, long in_max, long out_min, long out_max);

// ── String mínima compatible con el firmware ─────────────────────────────────
class String {
 public:
  String() = default;
  String(const char *s) : data_(s ? s : "") {}
  String(char c) : data_(1, c) {}
  String(int v) : data_(std::to_string(v)) {}
  String(unsigned v) : data_(std::to_string(v)) {}
  String(long v) : data_(std::to_string(v)) {}
  String(unsigned long v) : data_(std::to_string(v)) {}
  String(long long v) : data_(std::to_string(v)) {}
  String(unsigned long long v) : data_(std::to_string(v)) {}
  String(float v, unsigned char decimals = 2);
  String(double v, unsigned char decimals = 2);
  String(const String &o) = default;
  String &operator=(const String &o) = default;

  unsigned int length() const { return (unsigned)data_.size(); }
  char charAt(unsigned int i) const { return data_[i]; }
  char operator[](unsigned int i) const { return data_[i]; }
  const char *c_str() const { return data_.c_str(); }

  void concat(const String &s) { data_ += s.data_; }
  void concat(const char *s) { if (s) data_ += s; }
  void concat(char c) { data_ += c; }
  String &operator+=(const String &s) {
    data_ += s.data_;
    return *this;
  }
  String &operator+=(const char *s) {
    if (s) data_ += s;
    return *this;
  }
  String &operator+=(char c) {
    data_ += c;
    return *this;
  }

  int indexOf(char c, int from = 0) const;
  int indexOf(const char *s, int from = 0) const;
  int indexOf(const String &s, int from = 0) const;
  String substring(int from) const;
  String substring(int from, int to) const;
  long toInt() const;
  float toFloat() const;
  void trim();
  bool startsWith(const char *s) const;
  bool startsWith(const String &s) const;
  bool endsWith(const char *s) const;
  bool endsWith(const String &s) const;

  friend String operator+(const String &a, const String &b);
  friend String operator+(const String &a, const char *b);
  friend String operator+(const char *a, const String &b);
  friend String operator+(const String &a, char b);
  friend bool operator==(const String &a, const String &b);
  friend bool operator!=(const String &a, const String &b);

 private:
  std::string data_;
};

class HardwareSerial {
 public:
  enum Kind { USB, UART2 };
  explicit HardwareSerial(Kind k) : kind_(k) {}

  void begin(unsigned long baud);
  void begin(unsigned long baud, uint32_t config, int rxPin, int txPin);

  size_t print(const char *s);
  size_t print(char c);
  size_t print(const String &s);
  size_t print(int v, int base = DEC);
  size_t print(unsigned int v, int base = DEC);
  size_t print(long v, int base = DEC);
  size_t print(unsigned long v, int base = DEC);
  size_t print(long long v, int base = DEC);
  size_t print(unsigned long long v, int base = DEC);
  size_t print(float v, int digits = 2);
  size_t print(double v, int digits = 2);
  size_t print(bool v);

  size_t println();
  size_t println(const char *s);
  size_t println(char c);
  size_t println(const String &s);
  size_t println(int v, int base = DEC);
  size_t println(unsigned int v, int base = DEC);
  size_t println(long v, int base = DEC);
  size_t println(unsigned long v, int base = DEC);
  size_t println(long long v, int base = DEC);
  size_t println(unsigned long long v, int base = DEC);
  size_t println(float v, int digits = 2);
  size_t println(double v, int digits = 2);
  size_t println(bool v);

  int available();
  String readStringUntil(char terminator);

  size_t writeChar(char c);
  size_t writeRaw(const char *s, size_t n);

 private:
  Kind kind_;
};

extern HardwareSerial Serial;
extern HardwareSerial Serial2;

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

void attachInterrupt(int interruptNum, void (*userFunc)(), int mode);
int digitalPinToInterrupt(int pin);

// ── API SIL (implementada en sil_core.cpp) ───────────────────────────────────
#ifdef __cplusplus
extern "C" {
#endif

typedef void (*sil_advance_cb_t)(uint64_t us);
typedef uint32_t (*sil_sonar_cb_t)(int id);
typedef float (*sil_gyro_cb_t)(void);
typedef int32_t (*sil_encoder_cb_t)(void);
typedef int (*sil_tof_cb_t)(int idx);

void sil_set_callbacks(sil_advance_cb_t advance, sil_sonar_cb_t sonar,
                       sil_gyro_cb_t gyro, sil_encoder_cb_t encoder,
                       sil_tof_cb_t tof);
int sil_setup(void);
void sil_loop(void);
uint64_t sil_now_us(void);
void sil_set_now_us(uint64_t us);
void sil_set_param(const char *name, double value);
const char *sil_last_error(void);

void sil_serial2_rx_push(const char *data, int n, uint64_t arrival_us);
int sil_serial2_tx_pop_line(char *buf, int max, uint64_t *t_done_us);
int sil_serial_tx_pop(char *buf, int max);

int sil_ledc_duty(uint8_t pin);
int sil_ledc_res(uint8_t pin);
int sil_ledc_freq(uint8_t pin);
int sil_digital(uint8_t pin);

void sil_register_sonar(uint8_t trig_pin, uint8_t echo_pin, int id);

int32_t sil_encoder_count(void);
int sil_tof_mm(int idx);
// Parámetro de arranque puesto desde Python (FirmwareSIL.set_param); def si no está.
double sil_param(const char *name, double def);

#ifdef __cplusplus
}
#endif
