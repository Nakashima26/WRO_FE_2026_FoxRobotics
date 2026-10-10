#pragma once
#include <Arduino.h>

// Stub del VL53L1X de Pololu que reproduce el hallazgo de fox/tof-rate: init() deja
// 50 ms de presupuesto y el periodo efectivo es max(periodo, presupuesto).
struct RangingDataMock {
  uint16_t range_mm = 0;
  uint8_t range_status = 0;
  float peak_signal_count_rate_MCPS = 1.0f;
};

class VL53L1X {
 public:
  enum DistanceMode { Short, Medium, Long, Unknown };
  RangingDataMock rangingData;
  void setTimeout(uint16_t) {}
  bool init() { budget_ms_ = 50; mode_ = Long; return true; }
  void setDistanceMode(DistanceMode m) { mode_ = m; }
  bool setMeasurementTimingBudget(uint32_t us) { budget_ms_ = us / 1000; return true; }
  void startContinuous(uint32_t ms) { period_ = ms > budget_ms_ ? ms : budget_ms_; last_ = millis(); }
  void stopContinuous() {}
  bool dataReady() { delay(1); return millis() - last_ >= period_; }
  uint16_t read(bool = true) {
    last_ = millis();
    rangingData.range_mm = 1000;
    rangingData.range_status = (mode_ == Short && 1000 > 1300) ? 4 : 0;
    return rangingData.range_mm;
  }
 private:
  uint32_t budget_ms_ = 50, period_ = 50, last_ = 0;
  DistanceMode mode_ = Long;
};
