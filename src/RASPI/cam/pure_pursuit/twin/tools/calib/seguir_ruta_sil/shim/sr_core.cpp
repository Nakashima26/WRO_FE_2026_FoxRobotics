// Núcleo del banco de SeguirRuta: tiempo, pines, PWM, PCNT, BNO RVC y los seriales.
// Python (sr_sil.py) pone la cuenta del encoder y el rumbo en cada paso de física, que
// pide el callback cada vez que el tiempo cruza un ms.
#include <deque>
#include <string>

#include "Arduino.h"
#include "BluetoothSerial.h"
#include "Wire.h"
#include "driver/pulse_cnt.h"

#define SR_API extern "C" __declspec(dllexport)

extern void setup();
extern void loop();

typedef void (*sr_tick_cb_t)(uint64_t now_us);

static uint64_t now_us = 0, ultimo_tick_us = 0, ultimo_bno_us = 0;
static uint64_t us_por_millis = 25;
static sr_tick_cb_t tick_cb = nullptr;
static int32_t cuenta = 0, cuenta0 = 0;
static float yaw_deg = 0.0f;
static uint8_t bno_idx = 0;
static int pin_val[64] = {0};
static uint32_t ledc_duty[64] = {0};
static std::deque<uint8_t> rx[3];
static std::string tx_usb;

SrSerial Serial(0);
SrSerial Serial1(1);
TwoWire Wire;

static void empujar_bno() {
  // Trama RVC: AA AA idx yaw(int16 0.01°) pitch roll ax ay az (mg) 3 reservados suma.
  int16_t yaw = (int16_t)lroundf(yaw_deg * 100.0f);
  uint8_t f[19] = {0xAA, 0xAA, bno_idx++, (uint8_t)(yaw & 0xFF), (uint8_t)((yaw >> 8) & 0xFF)};
  uint8_t suma = 0;
  for (int i = 2; i < 18; i++) suma += f[i];
  f[18] = suma;
  for (uint8_t b : f) rx[1].push_back(b);
}

static void avanzar(uint64_t us) {
  uint64_t fin = now_us + us;
  while (now_us < fin) {
    uint64_t sig = ultimo_tick_us + 1000;
    if (sig > fin) { now_us = fin; break; }
    now_us = sig;
    ultimo_tick_us = sig;
    if (tick_cb) tick_cb(now_us);
    if (now_us - ultimo_bno_us >= 10000) {   // el BNO manda cada 10 ms (100 Hz)
      ultimo_bno_us = now_us;
      empujar_bno();
    }
  }
}

int sr_tof_activos = 0;
unsigned long sr_tof_bloqueo_us = 0;
void sr_delay_us(unsigned long us) { avanzar(us); }

unsigned long millis() { avanzar(us_por_millis); return (unsigned long)(now_us / 1000ULL); }
unsigned long micros() { avanzar(us_por_millis); return (unsigned long)now_us; }
void delay(unsigned long ms) { avanzar((uint64_t)ms * 1000ULL); }
long map(long x, long a, long b, long c, long d) { return (x - a) * (d - c) / (b - a) + c; }
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t pin, uint8_t val) { pin_val[pin & 63] = val; }
int digitalRead(uint8_t pin) { return pin_val[pin & 63]; }
bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
void ledcWrite(uint8_t pin, uint32_t duty) { ledc_duty[pin & 63] = duty; }

esp_err_t pcnt_new_unit(const pcnt_unit_config_t *, pcnt_unit_handle_t *ret) {
  *ret = (pcnt_unit_handle_t)1;
  return ESP_OK;
}
esp_err_t pcnt_unit_clear_count(pcnt_unit_handle_t) { cuenta0 = cuenta; return ESP_OK; }
esp_err_t pcnt_unit_get_count(pcnt_unit_handle_t, int *v) { *v = cuenta - cuenta0; return ESP_OK; }

int SrSerial::available() { return (int)rx[id_].size(); }
int SrSerial::read() {
  if (rx[id_].empty()) return -1;
  uint8_t b = rx[id_].front();
  rx[id_].pop_front();
  return b;
}
// Lo que tarda el Bluetooth por byte (SerialBT.write se bloquea con la cola llena).
static uint64_t bt_us_por_byte = 0;
size_t SrSerial::write(uint8_t c) {
  if (id_ == 0) tx_usb.push_back((char)c);
  if (id_ == 2 && bt_us_por_byte) avanzar(bt_us_por_byte);
  return 1;
}

SR_API void sr_set_tick_cb(sr_tick_cb_t cb) { tick_cb = cb; }
SR_API void sr_set_us_por_millis(uint64_t us) { us_por_millis = us; }
SR_API void sr_set_tof(int activos, uint64_t bloqueo_us) {
  sr_tof_activos = activos;
  sr_tof_bloqueo_us = (unsigned long)bloqueo_us;
}
SR_API void sr_set_bt_us_por_byte(uint64_t us) { bt_us_por_byte = us; }
SR_API void sr_setup() { setup(); }
SR_API void sr_loop() { loop(); }
SR_API uint64_t sr_now_us() { return now_us; }
SR_API void sr_set_count(int32_t c) { cuenta = c; }
SR_API void sr_set_yaw(float deg) { yaw_deg = deg; }
SR_API int sr_pin(int pin) { return pin_val[pin & 63]; }
SR_API uint32_t sr_duty(int pin) { return ledc_duty[pin & 63]; }
SR_API void sr_bt_send(const char *s) {
  for (const char *p = s; *p; p++) rx[2].push_back((uint8_t)*p);
}
// Copia lo que imprimió el USB desde la última llamada; devuelve los bytes copiados.
SR_API int sr_take_output(char *buf, int cap) {
  int n = (int)min(tx_usb.size(), (size_t)cap);
  memcpy(buf, tx_usb.data(), n);
  tx_usb.erase(0, n);
  return n;
}
