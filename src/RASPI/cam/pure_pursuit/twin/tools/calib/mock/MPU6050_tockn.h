#pragma once
#include <Arduino.h>
#include <Wire.h>

// Stub: update() cuesta ~1.8 ms de tiempo simulado (14 bytes I2C a 100 kHz).
extern void mock_advance_us(unsigned long us);

class MPU6050 {
 public:
  explicit MPU6050(WireMock &) {}
  void begin() {}
  void update() { mock_advance_us(1800); }
};
