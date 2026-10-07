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

Changes since `v1.0-nacional` (details in the [README](../README.md#13-what-changed-for-the-international-final-v2) and the [engineering log](engineering-log.md)):

| Area | Change | Status |
|---|---|---|
| Steering | Ackermann corrected (rack joints 47 mm, tie rods 18.13 mm); 19-tooth pinion; 53.1° / 37.0° at full lock | Validated in CAD; track test pending |
| Computer | Raspberry Pi 5 (16 GB) + Active Cooler; same pipeline at 40–41.5 fps | Port in progress: frame-rate constants pending |
| Sensors | 4 × VL53L1X ToF (front, left, right, rear) | Bench test |
| Vision | Ratio tests against orange tape and pale floor (`RED_GR_MAX`, `GREEN_S_MIN_MED`) | Validated (replay over 6 runs) |
| Camera | Pi 5 capture fix (`format=BGRx`) + lens-shading table for our lens + gains `<1.10,1.51>` ([ERR-28](engineering-log.md#err-28--raspberry-pi-5-the-race-camera-pipeline-would-not-start), [ERR-29](engineering-log.md#err-29--raspberry-pi-5-magenta-ring-at-the-frame-edges)) | Validated on a white sheet; pillar thresholds pending |
| Corners | Forward/reverse decision with hysteresis and a shared latch | In repository; track test pending |
| Firmware | Single parking-mode switch (`PARK_MODO`), dead parameters removed | In repository |

The release is tagged when the validation matrix in [testing.md](testing.md#62-validation-matrix-for-v2-international-final) passes.
