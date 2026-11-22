# Releases

Each competition corresponds to one release. A release is a git tag on the exact code that ran, plus these notes. To get the code of a release: `git checkout <tag>`.

| Release | Event | Date | Commit | Result |
|---|---|---|---|---|
| [`v1.0-nacional`](#v10-nacional--mexico-national-final) | WRO Mexico national final | 2026-09-19 | `d5cc66d` | 1st place, documentation 26/30 |
| [`v2.0-internacional`](#v20-internacional--in-progress) | WRO international final | 2026-12-08 to 12-10 | — | In progress |

---

## v1.0-nacional — Mexico national final

**Hardware**

| Item | Value |
|---|---|
| Main computer | Raspberry Pi 4 |
| Controller | ESP32 DevKit |
| Camera | Raspberry Pi Camera v2 + NoIR wide-angle lens (~120°), 640×480, fixed gains `<1.2,1.5>` |
| Distance | 3 × HC-SR04 (left, right, front) |
| IMU | MPU-6050 (gyro Z) |
| Drive | N20 50:1 + 2:1 gear stage, LEGO differential |
| Steering | SG90 servo, rack and pinion, 14-tooth module-1 pinion, rack joints 43 mm apart, ~37° inner wheel |

**Software**
- Vision on the Pi: floor-referenced color correction → red/green/magenta detection → bird's-eye view → corner lines → obstacle memory → corridor centerline → Pure Pursuit, at ~14–18 fps.
- ESP32 nine-state machine. Gyro heading-hold on clean straights; vision steers only while dodging.
- Obstacle Challenge: stop-and-turn corners (forward arc or reverse pivot). Open Challenge: continuous gyro-gated turn.
- Start from the parking lot (`INICIO`). Parallel parking (`ESTACIONANDO`).
- Hot start: the pipeline runs disarmed before the button, so the first sign is already tracked at the start.
- Camera brightness check before arming.

**Known issues at this release:** phantom front echoes (ERR-19); heading drift built up by dodges (ERR-23); performance depends on the battery voltage window (ERR-11); the forward/reverse decision could flip at the 28 cm boundary (ERR-22, fixed after the event).

---

## v2.0-internacional — in progress

The tag `v2.0-internacional` will be placed on the exact commit raced at the international final. Full design and results: [README](../README.md) (v2) — changes since v1 in [§4.2](../README.md#42-from-v1-to-v2--what-the-national-final-taught-us).

**Hardware**

| Item | v1 → v2 |
|---|---|
| Main computer | Raspberry Pi 4 → **Raspberry Pi 5 (8 GB)**; Picamera2 with the ISP scaling the image, ~40 fps |
| Distance | 3 × HC-SR04 → **4 × VL53L1X ToF** (front, left, right, rear; ROI 8 on the sides) |
| IMU | MPU-6050 → **BNO085** in UART-RVC mode |
| Drive | open-loop N20 + LEGO differential → **N30 30:1 with 7 CPR encoder (PCNT ×4)** (N20 50:1 with 12 CPR until 2026-11-17), spur 20T→25T, feed-forward + PI speed loop, top ~1.5 m/s |
| Steering | 14T pinion → **19T pinion, corrected Ackermann**; turning radius 123 / 134 mm |
| Chassis | 176 × 125 mm, battery in the middle, Pi on top |
| Electronics | PCB RevA → **PCB RevB** (logic board + power board, separate 5 V buck for the Pi) |
| Link | UART 115 200 → 460 800 baud with checksum |

**Software**

| Area | Change | Status |
|---|---|---|
| Obstacle Challenge | **Route mode**: pillar seat map, EKF localization with the ToF sensors, dynamic-programming route, speed profile, pure pursuit on the ESP32 | Clean 12-corner races on the car (runs 41, 48, 2026-11-11) |
| Start | Blind start: parking-lot position from a ToF histogram + orange-line votes | In use; known limitation without line votes (ERR-31) |
| Open Challenge | `RondaAbierta.ino`, ESP32 only (ToF wall following + BNO085) | 12/12 corners both directions, 29.6–29.9 s |
| Digital twin | Real firmware + real Pi runtime in a simulator; 42 seeds × 5 noise levels bench; exports the race build `PurePursuitCarrera.ino` + `carrera.json` | In use for every change |
| Camera | Lens-shading table; fixed exposure per venue (`mide_exposicion.py`, runs 57–64), back to automatic exposure after the 2026-11-19 remount; BEV recalibrated | In use |
| Recording | Video + clean BEV per run, auto-stop 5 s after the car stops | In use |

The release is tagged when the validation matrix in [README §3.10.4](../README.md#3104-obstacle-challenge-v2-real-car-route-mode) passes.
