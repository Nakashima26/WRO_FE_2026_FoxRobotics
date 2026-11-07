// ToF sin pared: dan una lectura cada 50 ms (2000 mm, status 2) y cada lectura detiene el
// loop sr_tof_bloqueo_us (I2C + la línea T por Bluetooth). En el carro solo F y L mandaban
// datos (sr_tof_activos = 2); con 0, init() falla y no hay ToF.
#pragma once
#include <stdint.h>

unsigned long millis();
extern int sr_tof_activos;
extern unsigned long sr_tof_bloqueo_us;
void sr_delay_us(unsigned long us);

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
  bool init() {
    if (id_ < 0) id_ = n_++;
    ult_ = millis() - (unsigned long)id_ * 23;   // fases distintas
    return id_ < sr_tof_activos;
  }
  void setAddress(uint8_t) {}
  bool setDistanceMode(DistanceMode) { return true; }
  void setROISize(uint8_t, uint8_t) {}
  bool setMeasurementTimingBudget(uint32_t) { return true; }
  void startContinuous(uint32_t) {}
  bool dataReady() { return millis() - ult_ >= 50; }
  uint16_t read(bool = true) {
    ult_ += 50;
    sr_delay_us(sr_tof_bloqueo_us);
    ranging_data = {2000, 2, 0.5f, 0.1f};
    return 2000;
  }

 private:
  int id_ = -1;
  unsigned long ult_ = 0;
  static inline int n_ = 0;
};
