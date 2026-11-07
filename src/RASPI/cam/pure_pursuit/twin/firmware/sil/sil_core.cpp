#include "Arduino.h"
#include "Wire.h"
#include "MPU6050_tockn.h"

#include <deque>
#include <map>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <stdexcept>
#include <cstdio>

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include <algorithm>

TwoWire Wire;

void attachInterrupt(int interruptNum, void (*userFunc)(), int mode) {
  (void)interruptNum;
  (void)userFunc;
  (void)mode;
}

int digitalPinToInterrupt(int pin) { return pin; }

extern void setup();
extern void loop();
extern bool piReadyReceived;

static void on_digital_write(uint8_t pin, uint8_t val);

// ── Tiempo global ────────────────────────────────────────────────────────────
static uint64_t now_us = 0;

static sil_advance_cb_t g_advance_cb = nullptr;
static sil_sonar_cb_t g_sonar_cb = nullptr;
static sil_gyro_cb_t g_gyro_cb = nullptr;
static sil_encoder_cb_t g_encoder_cb = nullptr;
static sil_tof_cb_t g_tof_cb = nullptr;

static std::map<std::string, double> g_params;
static char g_last_error[512];
static bool g_setup_done = false;
static bool g_ready_wait_armed = false;
static unsigned long g_ready_wait_start_ms = 0;

static double param_get(const char *name, double def) {
  auto it = g_params.find(name);
  return it == g_params.end() ? def : it->second;
}

void sil_set_param(const char *name, double value) { g_params[name] = value; }

const char *sil_last_error(void) {
  return g_last_error[0] ? g_last_error : nullptr;
}

static void set_error(const char *msg) {
  std::strncpy(g_last_error, msg, sizeof(g_last_error) - 1);
  g_last_error[sizeof(g_last_error) - 1] = '\0';
}

static void sil_advance_internal(uint64_t us) {
  if (us == 0) return;
  now_us += us;
  if (g_advance_cb) g_advance_cb(us);
}

void sil_set_callbacks(sil_advance_cb_t advance, sil_sonar_cb_t sonar,
                       sil_gyro_cb_t gyro, sil_encoder_cb_t encoder,
                       sil_tof_cb_t tof) {
  g_advance_cb = advance;
  g_sonar_cb = sonar;
  g_gyro_cb = gyro;
  g_encoder_cb = encoder;
  g_tof_cb = tof;
}

uint64_t sil_now_us(void) { return now_us; }
void sil_set_now_us(uint64_t us) { now_us = us; }

unsigned long millis() { return (unsigned long)(now_us / 1000ULL); }
unsigned long micros() { return (unsigned long)now_us; }

static void check_setup_timeout() {
  if (g_setup_done || !g_ready_wait_armed) return;
  if (piReadyReceived) return;
  double limit = param_get("setup_timeout_ms", 10000.0);
  if (millis() - g_ready_wait_start_ms >= (unsigned long)limit) {
    set_error("sil_setup: timeout esperando READY en Serial2");
    throw std::runtime_error(g_last_error);
  }
}

void delay(unsigned long ms) {
  check_setup_timeout();
  sil_advance_internal((uint64_t)(1000.0 * (double)ms));
}

void delayMicroseconds(unsigned int us) {
  sil_advance_internal((uint64_t)us);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float degrees(float rad) { return rad * 180.0f / (float)PI; }

// ── GPIO / LEDC ──────────────────────────────────────────────────────────────
struct PinState {
  uint8_t mode = INPUT;
  uint8_t level = LOW;
};

struct LedcState {
  bool attached = false;
  uint32_t freq = 0;
  uint8_t res = 0;
  uint32_t duty = 0;
};

static std::map<int, PinState> g_pins;
static std::map<int, LedcState> g_ledc;

void pinMode(uint8_t pin, uint8_t mode) { g_pins[pin].mode = mode; }

void digitalWrite(uint8_t pin, uint8_t val) {
  g_pins[pin].level = val;
  on_digital_write(pin, val);
}

int digitalRead(uint8_t pin) {
  auto it = g_pins.find(pin);
  return it == g_pins.end() ? LOW : (int)it->second.level;
}

int sil_digital(uint8_t pin) { return digitalRead(pin); }

bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution) {
  LedcState &s = g_ledc[pin];
  s.attached = true;
  s.freq = freq;
  s.res = resolution;
  s.duty = 0;
  return true;
}

void ledcWrite(uint8_t pin, uint32_t duty) { g_ledc[pin].duty = duty; }

int sil_ledc_duty(uint8_t pin) {
  auto it = g_ledc.find(pin);
  return it == g_ledc.end() ? 0 : (int)it->second.duty;
}

int sil_ledc_res(uint8_t pin) {
  auto it = g_ledc.find(pin);
  return it == g_ledc.end() ? 0 : (int)it->second.res;
}

int sil_ledc_freq(uint8_t pin) {
  auto it = g_ledc.find(pin);
  return it == g_ledc.end() ? 0 : (int)it->second.freq;
}

// ── Sonar ────────────────────────────────────────────────────────────────────
struct SonarDef {
  uint8_t trig;
  uint8_t echo;
  int id;
};

static std::vector<SonarDef> g_sonars;
static std::map<int, uint64_t> g_sonar_last_trigger_us;
static std::map<int, uint64_t> g_sonar_echo_hold_until_us;
static std::map<int, uint8_t> g_sonar_echo_level;
static std::map<int, uint8_t> g_sonar_prev_trig;

static int echo_pin_to_id(uint8_t echo_pin) {
  for (const auto &s : g_sonars) {
    if (s.echo == echo_pin) return s.id;
  }
  return -1;
}

static int trig_pin_to_id(uint8_t trig_pin) {
  for (const auto &s : g_sonars) {
    if (s.trig == trig_pin) return s.id;
  }
  return -1;
}

void sil_register_sonar(uint8_t trig_pin, uint8_t echo_pin, int id) {
  g_sonars.push_back({trig_pin, echo_pin, id});
}

static void register_default_sonars_once() {
  static bool done = false;
  if (done) return;
  done = true;
  sil_register_sonar(27, 32, 0);
  sil_register_sonar(26, 35, 1);
  sil_register_sonar(14, 33, 2);
}

void on_digital_write(uint8_t pin, uint8_t val) {
  register_default_sonars_once();
  int id = trig_pin_to_id(pin);
  if (id < 0) return;
  uint8_t prev = g_sonar_prev_trig[pin];
  if (prev == HIGH && val == LOW) {
    g_sonar_last_trigger_us[id] = now_us;
  }
  g_sonar_prev_trig[pin] = val;
}

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout) {
  register_default_sonars_once();
  if (state != HIGH) {
    sil_advance_internal(timeout);
    return 0;
  }

  int id = echo_pin_to_id(pin);
  if (id < 0) {
    sil_advance_internal(timeout);
    return 0;
  }

  uint64_t deadline = now_us + timeout;
  double hold_us = param_get("hc_no_echo_hold_us", 0.0);

  if (hold_us > 0 && g_sonar_echo_level[id] == HIGH) {
    uint64_t hold_end = g_sonar_echo_hold_until_us[id];
    while (now_us < hold_end && now_us < deadline) {
      uint64_t step = std::min(hold_end - now_us, deadline - now_us);
      sil_advance_internal(step);
    }
    if (now_us >= hold_end) g_sonar_echo_level[id] = LOW;
  }

  uint64_t trig_t = g_sonar_last_trigger_us[id];
  double pre_us = param_get("hc_trigger_to_echo_us", 450.0);
  uint64_t echo_start = trig_t + (uint64_t)pre_us;

  if (echo_start > now_us) {
    uint64_t target = std::min(echo_start, deadline);
    if (target > now_us) sil_advance_internal(target - now_us);
  }
  if (now_us >= deadline) return 0;

  uint32_t width = g_sonar_cb ? g_sonar_cb(id) : 0;
  uint64_t echo_end = echo_start + width;
  uint64_t total_from_trig = (echo_start - trig_t) + width;

  if (width > 0 && total_from_trig <= timeout) {
    if (echo_end > now_us) sil_advance_internal(echo_end - now_us);
    g_sonar_echo_level[id] = LOW;
    return width;
  }

  sil_advance_internal(deadline - now_us);
  if (hold_us > 0) {
    g_sonar_echo_level[id] = HIGH;
    g_sonar_echo_hold_until_us[id] = trig_t + (uint64_t)hold_us;
  }
  return 0;
}

int32_t sil_encoder_count(void) {
  return g_encoder_cb ? g_encoder_cb() : 0;
}

double sil_param(const char *name, double def) { return param_get(name, def); }

int sil_tof_mm(int idx) {
  if (g_tof_cb) return g_tof_cb(idx);
  return -1;
}

// ── MPU6050 ──────────────────────────────────────────────────────────────────
void MPU6050::calcGyroOffsets(bool, uint16_t, uint16_t) {
  double ms = param_get("mpu_calib_ms", 0.0);
  if (ms > 0) sil_advance_internal((uint64_t)(ms * 1000.0));
}

float MPU6050::getGyroZoffset() {
  return (float)param_get("gyro_z_offset", 0.3);
}

void MPU6050::update() {
  double cost = param_get("mpu_update_us", 500.0);
  sil_advance_internal((uint64_t)cost);
  gyro_z_ = g_gyro_cb ? g_gyro_cb() : 0.0f;
}

// ── String ───────────────────────────────────────────────────────────────────
String::String(float v, unsigned char decimals) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(decimals) << v;
  data_ = os.str();
}

String::String(double v, unsigned char decimals) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(decimals) << v;
  data_ = os.str();
}

int String::indexOf(char c, int from) const {
  if (from < 0) from = 0;
  size_t pos = data_.find(c, (size_t)from);
  return pos == std::string::npos ? -1 : (int)pos;
}

int String::indexOf(const char *s, int from) const {
  if (!s || from < 0) return -1;
  size_t pos = data_.find(s, (size_t)from);
  return pos == std::string::npos ? -1 : (int)pos;
}

int String::indexOf(const String &s, int from) const { return indexOf(s.c_str(), from); }

String String::substring(int from) const {
  if (from < 0) from = 0;
  if ((size_t)from >= data_.size()) return String("");
  return String(data_.c_str() + from);
}

String String::substring(int from, int to) const {
  if (from < 0) from = 0;
  if (to < from) return String("");
  if ((size_t)from >= data_.size()) return String("");
  return String(data_.substr((size_t)from, (size_t)(to - from)).c_str());
}

long String::toInt() const { return strtol(data_.c_str(), nullptr, 10); }

float String::toFloat() const { return (float)strtod(data_.c_str(), nullptr); }

void String::trim() {
  size_t start = data_.find_first_not_of(" \t\r\n");
  size_t end = data_.find_last_not_of(" \t\r\n");
  if (start == std::string::npos) {
    data_.clear();
    return;
  }
  data_ = data_.substr(start, end - start + 1);
}

bool String::startsWith(const char *s) const {
  if (!s) return false;
  return data_.compare(0, strlen(s), s) == 0;
}

bool String::startsWith(const String &s) const { return startsWith(s.c_str()); }

bool String::endsWith(const char *s) const {
  if (!s) return false;
  size_t n = strlen(s);
  if (data_.size() < n) return false;
  return data_.compare(data_.size() - n, n, s) == 0;
}

bool String::endsWith(const String &s) const { return endsWith(s.c_str()); }

String operator+(const String &a, const String &b) {
  String r(a);
  r += b;
  return r;
}

String operator+(const String &a, const char *b) {
  String r(a);
  r += b;
  return r;
}

String operator+(const char *a, const String &b) {
  String r(a);
  r += b;
  return r;
}

String operator+(const String &a, char b) {
  String r(a);
  r += b;
  return r;
}

bool operator==(const String &a, const String &b) { return a.data_ == b.data_; }

bool operator!=(const String &a, const String &b) { return !(a == b); }

// ── Serial ───────────────────────────────────────────────────────────────────
struct RxChunk {
  std::string data;
  uint64_t arrival_us;
};

static std::deque<char> g_serial_usb;
static const size_t kSerialUsbMax = 2 * 1024 * 1024;

static std::deque<RxChunk> g_serial2_rx;
static std::string g_serial_usb_line;   // línea en curso, solo para detectar "Esperando READY"
static std::string g_serial2_tx_line;   // línea en curso hacia la Pi

struct TxLine {
  std::string text;
  uint64_t done_us;
};
static std::deque<TxLine> g_serial2_lines;

// UART del ESP32 sin buffer de software: el byte sale a us_per_char y write()
// solo bloquea cuando la FIFO de hardware (fifo bytes) está llena. free_us es
// cuándo termina de salir lo que ya se escribió.
struct UartTx {
  double free_us = 0.0;
};
static UartTx g_usb_tx;
static UartTx g_uart2_tx;

static void uart_tx_char(UartTx &u, const char *per_key, const char *fifo_key) {
  double per = param_get(per_key, 86.8);
  if (per <= 0) return;
  double fifo = param_get(fifo_key, 128.0);
  if (u.free_us < (double)now_us) u.free_us = (double)now_us;
  double pending = (u.free_us - (double)now_us) / per;
  if (pending >= fifo) {
    uint64_t wait = (uint64_t)((pending - fifo + 1.0) * per);
    if (wait > 0) sil_advance_internal(wait);
    if (u.free_us < (double)now_us) u.free_us = (double)now_us;
  }
  u.free_us += per;
}

HardwareSerial Serial(HardwareSerial::USB);
HardwareSerial Serial2(HardwareSerial::UART2);

void HardwareSerial::begin(unsigned long) {}
void HardwareSerial::begin(unsigned long, uint32_t, int, int) {}

size_t HardwareSerial::writeChar(char c) {
  if (kind_ == USB) {
    g_serial_usb.push_back(c);
    while (g_serial_usb.size() > kSerialUsbMax) g_serial_usb.pop_front();
    uart_tx_char(g_usb_tx, "serial_us_per_char", "serial_fifo");
    if (c == '\n') {
      if (!g_ready_wait_armed && g_serial_usb_line.find("Esperando READY") != std::string::npos) {
        g_ready_wait_armed = true;
        g_ready_wait_start_ms = millis();
      }
      g_serial_usb_line.clear();
    } else if (g_serial_usb_line.size() < 512) {
      g_serial_usb_line.push_back(c);
    }
    return 1;
  }
  uart_tx_char(g_uart2_tx, "serial2_us_per_char", "serial2_fifo");
  if (c == '\n') {
    std::string line = g_serial2_tx_line;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    g_serial2_lines.push_back({line, (uint64_t)g_uart2_tx.free_us});
    g_serial2_tx_line.clear();
  } else {
    g_serial2_tx_line.push_back(c);
  }
  return 1;
}

size_t HardwareSerial::writeRaw(const char *s, size_t n) {
  size_t w = 0;
  for (size_t i = 0; i < n; ++i) w += writeChar(s[i]);
  return w;
}

size_t HardwareSerial::print(const char *s) { return s ? writeRaw(s, strlen(s)) : 0; }
size_t HardwareSerial::print(char c) { return writeChar(c); }
size_t HardwareSerial::print(const String &s) { return writeRaw(s.c_str(), s.length()); }
size_t HardwareSerial::print(bool v) { return print(v ? 1 : 0); }

static size_t print_ll(HardwareSerial *self, long long v, int base) {
  char buf[64];
  if (base == HEX) snprintf(buf, sizeof(buf), "%llx", (unsigned long long)v);
  else snprintf(buf, sizeof(buf), "%lld", v);
  return self->writeRaw(buf, strlen(buf));
}

size_t HardwareSerial::print(int v, int base) { return print_ll(this, v, base); }
size_t HardwareSerial::print(unsigned int v, int base) { return print((int)v, base); }
size_t HardwareSerial::print(long v, int base) { return print_ll(this, v, base); }
size_t HardwareSerial::print(unsigned long v, int base) { return print_ll(this, (long long)v, base); }
size_t HardwareSerial::print(long long v, int base) { return print_ll(this, v, base); }
size_t HardwareSerial::print(unsigned long long v, int base) { return print_ll(this, (long long)v, base); }
size_t HardwareSerial::print(float v, int digits) { return print(String(v, (unsigned char)digits)); }
size_t HardwareSerial::print(double v, int digits) { return print(String(v, (unsigned char)digits)); }

size_t HardwareSerial::println() { return writeChar('\r') + writeChar('\n'); }

#define SIL_PRINTLN1(T) \
  size_t HardwareSerial::println(T v, int base) { \
    size_t n = print(v, base); \
    n += println(); \
    return n; \
  }

SIL_PRINTLN1(int)
SIL_PRINTLN1(unsigned int)
SIL_PRINTLN1(long)
SIL_PRINTLN1(unsigned long)
SIL_PRINTLN1(long long)
SIL_PRINTLN1(unsigned long long)

size_t HardwareSerial::println(const char *s) {
  size_t n = print(s);
  n += println();
  return n;
}

size_t HardwareSerial::println(char c) {
  size_t n = print(c);
  n += println();
  return n;
}

size_t HardwareSerial::println(const String &s) {
  size_t n = print(s);
  n += println();
  return n;
}

size_t HardwareSerial::println(float v, int digits) {
  size_t n = print(v, digits);
  n += println();
  return n;
}

size_t HardwareSerial::println(double v, int digits) {
  size_t n = print(v, digits);
  n += println();
  return n;
}

size_t HardwareSerial::println(bool v) { return println(v ? 1 : 0); }

static size_t serial2_visible_count() {
  size_t n = 0;
  for (const auto &ch : g_serial2_rx) {
    if (ch.arrival_us <= now_us) n += ch.data.size();
  }
  return n;
}

// true si encontró el terminador (aunque la línea esté vacía).
static bool serial2_consume_visible(std::string *out, char until) {
  while (true) {
    bool progress = false;
    while (!g_serial2_rx.empty() && g_serial2_rx.front().arrival_us <= now_us) {
      progress = true;
      RxChunk &ch = g_serial2_rx.front();
      for (size_t i = 0; i < ch.data.size(); ++i) {
        char c = ch.data[i];
        if (c == until) {
          ch.data.erase(0, i + 1);
          if (ch.data.empty()) g_serial2_rx.pop_front();
          return true;
        }
        if (out) out->push_back(c);
      }
      g_serial2_rx.pop_front();
    }
    if (!progress) break;
    if (g_serial2_rx.empty()) break;
    uint64_t next = g_serial2_rx.front().arrival_us;
    if (next <= now_us) continue;
    sil_advance_internal(next - now_us);
  }
  return false;
}

int HardwareSerial::available() {
  if (kind_ != UART2) return 0;
  return (int)serial2_visible_count();
}

String HardwareSerial::readStringUntil(char terminator) {
  if (kind_ != UART2) return String("");
  const unsigned long timeout_ms = 1000;
  unsigned long start = millis();
  std::string out;
  while (true) {
    if (serial2_consume_visible(&out, terminator)) break;
    if (millis() - start >= timeout_ms) break;
    if (g_serial2_rx.empty()) {
      unsigned long remain = timeout_ms - (millis() - start);
      sil_advance_internal((uint64_t)remain * 1000ULL);
      break;
    }
    uint64_t next = g_serial2_rx.front().arrival_us;
    if (next > now_us) {
      unsigned long remain = timeout_ms - (millis() - start);
      uint64_t max_us = (uint64_t)remain * 1000ULL;
      uint64_t step = std::min(next - now_us, max_us);
      if (step > 0) sil_advance_internal(step);
    }
    if (millis() - start >= timeout_ms) break;
  }
  return String(out.c_str());
}

void sil_serial2_rx_push(const char *data, int n, uint64_t arrival_us) {
  if (!data || n <= 0) return;
  g_serial2_rx.push_back({std::string(data, (size_t)n), arrival_us});
}

int sil_serial2_tx_pop_line(char *buf, int max, uint64_t *t_done_us) {
  if (g_serial2_lines.empty()) return -1;
  if (g_serial2_lines.front().done_us > now_us) return -1;
  const TxLine &ln = g_serial2_lines.front();
  int n = (int)std::min((size_t)max - 1, ln.text.size());
  memcpy(buf, ln.text.c_str(), (size_t)n);
  buf[n] = '\0';
  if (t_done_us) *t_done_us = ln.done_us;
  g_serial2_lines.pop_front();
  return n;
}

int sil_serial_tx_pop(char *buf, int max) {
  if (max <= 0) return 0;
  int n = (int)std::min(g_serial_usb.size(), (size_t)max);
  auto it = g_serial_usb.begin();
  for (int i = 0; i < n; ++i, ++it) buf[i] = *it;
  g_serial_usb.erase(g_serial_usb.begin(), g_serial_usb.begin() + n);
  return n;
}

int sil_setup(void) {
  g_setup_done = false;
  g_ready_wait_armed = false;
  g_last_error[0] = '\0';
  try {
    setup();
    g_setup_done = true;
    return 0;
  } catch (const std::exception &e) {
    set_error(e.what());
    return -1;
  }
}

void sil_loop(void) {
  double oh = param_get("loop_overhead_us", 300.0);
  sil_advance_internal((uint64_t)oh);
  loop();
}
