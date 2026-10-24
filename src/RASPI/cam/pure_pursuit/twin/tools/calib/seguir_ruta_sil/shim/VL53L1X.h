// Sin ToF: init() falla y SeguirRuta corre como con --pared 0 y sin freno frontal.
#pragma once
#include <stdint.h>

class VL53L1X {
 public:
  enum DistanceMode { Short, Medium, Long, Unknown };
  struct RangingData {
    uint16_t range_mm;
    uint8_t range_status;
    float peak_signal_count_rate_MCPS;
    float ambient_count_rate_MCPS;
  };
  RangingData ranging_data{};
  void setTimeout(uint16_t) {}
  bool init() { return false; }
  void setAddress(uint8_t) {}
  bool setDistanceMode(DistanceMode) { return true; }
  void setROISize(uint8_t, uint8_t) {}
  bool setMeasurementTimingBudget(uint32_t) { return true; }
  void startContinuous(uint32_t) {}
  bool dataReady() { return false; }
  uint16_t read(bool = true) { return 0; }
};
