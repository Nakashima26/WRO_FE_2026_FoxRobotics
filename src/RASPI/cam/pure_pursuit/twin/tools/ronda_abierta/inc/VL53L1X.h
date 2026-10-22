#pragma once
#include <cstdint>
extern "C" int sil_tof_mm(int idx);
unsigned long millis();
// Dirección -> índice del twin (0=L 1=R 2=atrás 3=frente); -1 del twin = sin eco (status 4).
class VL53L1X {
 public:
  enum DistanceMode { Short, Medium, Long, Unknown };
  enum RangeStatus : uint8_t { RangeValid = 0, SigmaFail = 1, SignalFail = 2, RangeValidMinRangeClipped = 3,
    OutOfBoundsFail = 4, HardwareFail = 5, RangeValidNoWrapCheckFail = 6, WrapTargetFail = 7, None = 255 };
  struct { uint16_t range_mm; uint8_t range_status; } ranging_data{0, None};
  void setTimeout(uint16_t) {}
  bool init() { return true; }
  void setAddress(uint8_t a) { idx_ = a == 0x30 ? 3 : a == 0x31 ? 0 : a == 0x32 ? 1 : 2; }
  bool setDistanceMode(DistanceMode) { return true; }
  void setROISize(uint8_t, uint8_t) {}
  bool setMeasurementTimingBudget(uint32_t) { return true; }
  void startContinuous(uint32_t ms) { per_ = ms; next_ = millis() + ms; }
  bool dataReady() { return millis() >= next_; }
  uint16_t read(bool = true) {
    next_ = millis() + per_;
    int mm = sil_tof_mm(idx_);
#ifdef RA_FRONT_INFLA
    // frontal sin calibrar (prueba estática 2026-10-19): +30 mm cerca, ~+200 mm desde ~60 cm
    if (idx_ == 3 && mm > 0) mm += mm > 450 ? 200 : 30;
#endif
    ranging_data.range_mm = mm < 0 ? 0 : mm;
    ranging_data.range_status = mm < 0 ? OutOfBoundsFail : RangeValid;
    return ranging_data.range_mm;
  }
 private:
  int idx_ = 2; unsigned long per_ = 50, next_ = 0;
};
