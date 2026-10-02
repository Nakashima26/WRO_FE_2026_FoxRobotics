#pragma once

#include "Wire.h"

class MPU6050 {
 public:
  explicit MPU6050(TwoWire &) {}
  bool begin() { return true; }
  void calcGyroOffsets(bool console = false, uint16_t delayBefore = 1000,
                       uint16_t delayAfter = 3000);
  float getGyroZoffset();
  void update();
  float getGyroZ() const { return gyro_z_; }

 private:
  float gyro_z_ = 0.0f;
};
