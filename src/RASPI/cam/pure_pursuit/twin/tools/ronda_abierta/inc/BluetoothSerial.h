#pragma once
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <cstddef>
#include <cstring>
class Print {
 public:
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
  size_t printf(const char* f, ...) { char buf[512]; va_list a; va_start(a, f); int n = vsnprintf(buf, sizeof buf, f, a); va_end(a);
    if (n < 0) return 0; if (n > (int)sizeof buf - 1) n = sizeof buf - 1; return write((const uint8_t*)buf, n); }
  size_t println(const char* s) { size_t n = write((const uint8_t*)s, strlen(s)); return n + write((uint8_t)'\n'); }
};
class BluetoothSerial {
 public:
  bool begin(const char*) { return true; }
  int available() { return 0; }
  int read() { return -1; }
  size_t write(uint8_t) { return 1; }
  size_t write(const uint8_t*, size_t n) { return n; }
};
// Serial1 = BNO085 en UART-RVC: paquetes de 19 bytes cada 10 ms con el yaw (crece a la derecha).
// shortcut: el yaw llega por sil_tof_mm(99) (centésimas de grado) para no tocar sil_core; si se usa más, darle su callback.
extern "C" int sil_tof_mm(int idx);
unsigned long millis();
class RvcSerial {
 public:
  void setRxBufferSize(int) {}
  void begin(unsigned long, uint32_t = 0, int = -1, int = -1) { next_ = millis(); }
  int available() { fill(); return n_ - i_; }
  int read() { fill(); return i_ < n_ ? buf_[i_++] : -1; }
 private:
  uint8_t buf_[19]; int n_ = 0, i_ = 0; unsigned long next_ = 0;
  void fill() {
    if (i_ < n_) return;
    unsigned long now = millis();
    if (now < next_) return;
    next_ = now + 10;
    int16_t yaw = (int16_t)sil_tof_mm(99);
    uint8_t* p = buf_; p[0] = p[1] = 0xAA; for (int k = 2; k < 19; k++) p[k] = 0;
    p[3] = yaw & 0xFF; p[4] = (yaw >> 8) & 0xFF;
    uint8_t s = 0; for (int k = 2; k < 18; k++) s += p[k]; p[18] = s;
    n_ = 19; i_ = 0;
  }
};
inline RvcSerial Serial1;
