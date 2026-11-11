#include <cstdint>
using sil_advance_cb_t = void (*)(uint64_t);
#include <array>
#include <cstring>
#include <cmath>
#include <algorithm>

// NativeVehicle owns all pointers; FirmwareSIL keeps their Python owners alive.
// e: fraction, count, last ds/yaw, bias, cap, acc, scrub, counts/mm, max wheel,
//    last collision time. History stores the same constant t within an advance.
using vehicle_step_t = void (*)(double *, const double *);
static vehicle_step_t g_vehicle_step = nullptr;
static double *g_vehicle = nullptr, *g_vehicle_params = nullptr, *g_encoder = nullptr;
static sil_advance_cb_t g_collision_cb = nullptr;
struct PoseSample { double t, x, y, h; };
static constexpr size_t kHistoryCapacity = 20000; // Python deque(maxlen=20000).
static std::array<PoseSample, kHistoryCapacity> g_history;
static size_t g_history_next = 0, g_history_count = 0;
extern "C" void sil_set_native_vehicle(void *step, double *state, double *params,
                                       double *encoder, sil_advance_cb_t collision) {
  g_vehicle_step = reinterpret_cast<vehicle_step_t>(step);
  g_vehicle = state; g_vehicle_params = params; g_encoder = encoder;
  g_collision_cb = collision;
  g_history_next = g_history_count = 0;
}
extern "C" size_t sil_drain_history(double *out) {
  size_t n = g_history_count;
  if (out && n) {
    size_t start = (g_history_next + kHistoryCapacity - n) % kHistoryCapacity;
    size_t first = std::min(n, kHistoryCapacity - start);
    std::memcpy(out, g_history.data() + start, first * sizeof(PoseSample));
    std::memcpy(out + 4 * first, g_history.data(), (n - first) * sizeof(PoseSample));
    g_history_count = 0;
  }
  return n;
}
extern "C" bool sil_native_enabled() { return g_vehicle_step != nullptr; }
extern "C" void sil_native_advance(uint64_t us, uint64_t now_us, const double *a) {
  double *s = g_vehicle, *e = g_encoder;
  s[11] = a[0]; s[12] = a[1]; s[13] = a[2];
  double t = now_us / 1e6, remaining = us / 1e6;
  volatile double exponent = 2.0; // CPython float ** 2 uses libm pow.
  while (remaining > 1e-12) {
    double sub = std::min(remaining, 0.001); remaining -= sub;
    double h0 = s[2], v0 = s[3]; s[10] = sub;
    g_vehicle_step(s, g_vehicle_params);
    e[2] = 0.0 + s[3] * sub;
    e[3] = -(s[2] - h0) / sub;
    double slip = e[4];
    slip += std::min(e[5], e[6] * std::fabs((s[3] - v0) / sub));
    if (e[9] > 1e-6) slip += e[7] * std::pow(std::fabs(s[9]) / e[9], exponent);
    slip = std::max(0.0, slip);
    e[0] += (s[6] * sub) * e[8] * slip;
    double ticks = std::trunc(e[0]); e[0] -= ticks; e[1] += ticks;
    g_history[g_history_next] = {t, s[0], s[1], s[2]};
    if (++g_history_next == kHistoryCapacity) g_history_next = 0;
    if (g_history_count < kHistoryCapacity) ++g_history_count;
    // Exactly the Python predicate, checked after the first qualifying substep.
    if (t - e[10] >= 0.005) {
      e[10] = t;
      if (g_collision_cb) g_collision_cb(0);
    }
  }
}
