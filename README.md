# WRO 2026 — Future Engineers | FoxRobotics Self-Driving Car

> **Team:** FoxRobotics · **Members:** Erick Blanco · Jesse Banda · César Emiliano Ahumada · **Coach:** Daniel Millan
> **Country / Region:** México — Baja California · **Season:** 2026

<table>
  <tr>
    <td align="center"><b>📗 v2 — International Final (this document)</b><br>Current car, current code</td>
    <td align="center"><a href="docs/v1-national/README.md"><b>📘 v1 — Mexico National Final</b></a><br>Archived documentation, exactly as judged (1st place)</td>
  </tr>
</table>

This README documents **the car that competes at the WRO 2026 International Final (v2)**. After winning the Mexico national final with v1 we rebuilt almost every subsystem: a shorter chassis, a new two-board PCB, an encoder motor, four ToF sensors, a BNO085 IMU, a Raspberry Pi 5, and a completely new obstacle-round strategy (a planned route followed with odometry instead of reactive steering), developed with a software-in-the-loop **digital twin**. v1 is kept as a separate, unchanged document so the iteration is visible; [§3](#3-from-v1-to-v2--what-the-national-final-taught-us) explains what we changed and why.

**Quick links:** [Videos](#14-videos) · [Engineering Journal](ENGINEERING_JOURNAL.md) · [Engineering log (decisions, incidents)](docs/engineering-log.md) · [Testing workflow](docs/testing.md) · [Releases](docs/releases.md) · [Schematics (PDF)](electrical/export/) · [Firmware](src/ESP32/PurePursuitCarrera/) · [Raspberry Pi code](src/RASPI/cam/pure_pursuit/)

---

## How this document maps to the judging rubric

The WRO Future Engineers rules (Appendix C) score documentation in five criteria. To make evaluation fast, each criterion has a home section:

| Rubric criterion | Where to find the evidence |
|---|---|
| **1. Mobility and Mechanical Design** — torque/speed reasoning, trade-offs, iterations | [§5 Mobility](#5-mobility-and-mechanical-design): gear-ratio study with measured speeds, Ackermann steering redesign with measured turning radii, chassis layout v1 → v2 |
| **2. Power and Sensor Architecture** — power budget, wiring, sensor placement, calibration, failure points | [§6 Power and sensors](#6-power-and-sensor-architecture): power tree, budget, two-board PCB with schematics, pin map, sensor placement against field geometry, calibration procedures, electrical failures we hit and their fixes |
| **3. Software Architecture and Obstacle Strategy** — state machine, justified algorithms, edge cases, tuning metrics | [§7 Software](#7-software-architecture) and [§8 Obstacle strategy](#8-obstacle-management--route-mode): module map, data flow and timing, ESP32 state machine, route planner (dynamic programming), EKF localization, pure pursuit on the ESP32, blind start, parking, 2026 surprise pillar ([§8.8](#88-2026-extra-task--the-surprise-pillar)); [§9 Digital twin](#9-digital-twin--software-in-the-loop-simulator) |
| **4. Systems Thinking and Engineering Decisions** — constraints, "we chose X instead of Y because…", risks | [§3 v1 → v2](#3-from-v1-to-v2--what-the-national-final-taught-us), [§11 Systems thinking](#11-systems-thinking-and-engineering-decisions): constraints, trade-off table, risk and failure-mode table, decision log |
| **5. Reproducibility and GitHub Quality** — structure, commits, testing workflow, versioning | [§10 Testing](#10-testing-and-validation), [§12 Build and run](#12-how-to-build-and-run), [§13 Repository](#13-repository-structure), [releases](docs/releases.md) (`v1.0-nacional` → `v2.0-internacional`) |

---

## Table of Contents

1. [The Team](#1-the-team)
2. [The Challenge and Our Goals](#2-the-challenge-and-our-goals)
3. [From v1 to v2 — what the national final taught us](#3-from-v1-to-v2--what-the-national-final-taught-us)
4. [Vehicle Overview](#4-vehicle-overview)
5. [Mobility and Mechanical Design](#5-mobility-and-mechanical-design)
6. [Power and Sensor Architecture](#6-power-and-sensor-architecture)
7. [Software Architecture](#7-software-architecture)
8. [Obstacle Management — route mode](#8-obstacle-management--route-mode)
9. [Digital Twin — software-in-the-loop simulator](#9-digital-twin--software-in-the-loop-simulator)
10. [Testing and Validation](#10-testing-and-validation)
11. [Systems Thinking and Engineering Decisions](#11-systems-thinking-and-engineering-decisions)
12. [How to Build and Run](#12-how-to-build-and-run)
13. [Repository Structure](#13-repository-structure)
14. [Videos](#14-videos)
15. [Cost Report](#15-cost-report)
16. [License](#16-license)

---

# 1. The Team

<p align="center">
  <img src="t-photos/TeamPicture.jpeg" width="560" alt="FoxRobotics team photo">
</p>

| Member | Main responsibilities |
|---|---|
| **Erick Blanco** | Mechanical design (SolidWorks), ESP32 firmware, sensor selection and design, testing on the track |
| **Jesse Banda** | Raspberry Pi software: digital twin / simulator, path planning, speed profile, parking maneuvers |
| **César Emiliano Ahumada** | PCB design (KiCad), electrical testing |
| **Daniel Millan** (coach) | Guidance and logistics. Following the WRO rules, the coach did not build or program the car. |

We are a team from Baja California, México. FoxRobotics placed **1st at the WRO 2026 Mexico national final** (2026-09-19) in Future Engineers.

---

# 2. The Challenge and Our Goals

The car must drive **three laps** autonomously around a 3 × 3 m field whose inner walls move between rounds (**Open Challenge**), and, in the **Obstacle Challenge**, also pass red pillars on the right and green pillars on the left, start from a parking lot and park in it at the end. Score ranks first; time only breaks ties. In 2026 the Obstacle Challenge also has an optional **extra task**: one pillar changes color and height, and the car must obey its original color (§8.8).

**Our goal for the international final:** keep the reliability that won the national final, and remove the limits that made v1 slow and fragile (§3). Concretely:

| Goal | Target | Why |
|---|---|---|
| Finish both challenges in every round | 0 collisions, 12/12 corners, park inside the lot | Points rank first |
| Faster obstacle round | One continuous route instead of stop-and-turn corners | v1 stopped at every corner; a lap took ~25–30 s |
| Robust to the venue | Any start position of the parking lot, any lighting | At the national final the judges moved the parking lot along the straight; the venue light shifted our colors |
| Reproducible | Measured parameters, one release per event, a simulator that runs the real code | So any change can be tested before it reaches the car |

### Design constraints (from the rules and from our own limits)

| Constraint | Value | Consequence for the design |
|---|---|---|
| Max vehicle size | 300 × 200 × 300 mm | We built much smaller (176 × 125 mm) to have room to dodge and park |
| Parking-lot length | 1.5 × car length | A shorter car also gets a shorter lot, so shorter alone is not enough: the turning radius must shrink too (§5.3) |
| Drive | Wheels physically connected, one drive motor; steering by turning the wheels | Single motor + differential + Ackermann steering |
| Start procedure | One switch to power on, one button to start | Start button on the Pi, READY LED (§7.5) |
| Wireless | Not allowed during the run | Bluetooth/Wi-Fi only on the bench; disabled for competition |
| Compute | Real-time vision + planning on board | Raspberry Pi 5 for perception and planning, ESP32 for real-time control |
| Time | ~11 weeks between the national and the international final | We rebuilt and tested in parallel using a simulator (§9) |

---

# 3. From v1 to v2 — what the national final taught us

The [v1 documentation](docs/v1-national/README.md) describes the national-final car in full. It finished every round we needed, but the logs and videos of ~1200 test runs showed where it was limited. Each problem below became one change in v2.

| # | What we saw with v1 (evidence) | Root cause | v2 change | Section |
|---|---|---|---|---|
| 1 | The obstacle round was slow: the car **stopped before every corner** and did a forward arc or a reverse pivot | v1 steered reactively from the camera; it never knew where it was on the field, so it could not plan a corner through the pillars | **Route mode**: the Pi builds a map of the pillars, plans a full lap as a curve in millimeters and the ESP32 follows it with odometry | [§8](#8-obstacle-management--route-mode) |
| 2 | Maneuvers changed with the battery charge: a fresh pack over-rotated the dodges (ERR-11) | Open-loop motor (no encoder) | Gear motor **with encoder** + speed loop on the ESP32 | [§5.2](#52-drivetrain--gear-ratio-chosen-from-measured-speed), [§7.4](#74-esp32-firmware-and-state-machine) |
| 3 | Heading drifted: 6.5 % low over 12 corners and −7.5° after each corner maneuver (ERR-23) | MPU-6050 rate integrated on a busy loop | **BNO085** with on-chip fusion (100 Hz) | [§6.4](#64-sensors--selection-placement-and-calibration) |
| 4 | Phantom walls from the front ultrasonic in ~20 % of straight frames (ERR-19) | Wide acoustic lobe, multipath | **4 × VL53L1X ToF** with a narrow, software-set field of view as the distance sensors; the 3 HC-SR04 stay mounted only as a cross-check (§6.4) | [§6.4](#64-sensors--selection-placement-and-calibration) |
| 5 | The car was long (≈240 mm with the parking-exit extension) and turned wide (R ≈ 208 mm) | Motor perpendicular to the axle, Pi in the middle, anti-Ackermann linkage | **Shorter chassis (176 mm)**, motor parallel to the axle, Ackermann linkage, bigger pinion → **R ≈ 123–134 mm measured** | [§5](#5-mobility-and-mechanical-design) |
| 6 | The Pi 4 ran at 14–18 fps and hit 84 °C with thermal throttling | Vision is compute-bound; the Pi was enclosed | **Raspberry Pi 5** on top of the car; hardware image scaling → ~40 fps | [§6.5](#65-main-computer--raspberry-pi-5) |
| 7 | Hand-wired harness and a single-floor board shaped around the Pi 4 | No room for new sensors | **Two-board stacked PCB (RevB)**: power board + logic board | [§6.3](#63-pcb-revb--two-stacked-boards) |
| 8 | Every change had to be tested on the real track, one run at a time | No simulator | **Digital twin** that compiles the real firmware and runs the real Pi code | [§9](#9-digital-twin--software-in-the-loop-simulator) |
| 9 | Documentation scored 26/30 | Missing power budget, v2 wiring, quantitative test evidence | This document: rubric map, measured tables, validation matrices, release notes | — |

**What we kept, on purpose:** the two-controller split (Pi for perception and planning, ESP32 for real-time control), the LEGO wheels and axles, the 3S LiPo, the rack-and-pinion steering concept and the bird's-eye-view (BEV) camera geometry. They worked, and changing them would have cost time without solving a problem we had measured.

---

# 4. Vehicle Overview

<table>
  <tr>
    <td align="center"><img src="v-photos/v2/front.jpeg" width="300" alt="Front view"><br><b>Front</b></td>
    <td align="center"><img src="v-photos/v2/rear.jpeg" width="300" alt="Rear view"><br><b>Rear</b></td>
    <td align="center"><img src="v-photos/v2/top.jpeg" width="300" alt="Top view"><br><b>Top</b></td>
  </tr>
  <tr>
    <td align="center"><img src="v-photos/v2/left.jpeg" width="300" alt="Left side"><br><b>Left</b></td>
    <td align="center"><img src="v-photos/v2/right.jpeg" width="300" alt="Right side"><br><b>Right</b></td>
    <td align="center"><img src="v-photos/v2/bottom.jpeg" width="300" alt="Bottom view"><br><b>Bottom</b></td>
  </tr>
</table>

**[TODO: add the six v2 photos to `v-photos/v2/` with these names (front, rear, top, left, right, bottom). The v1 photos stay in `v-photos/` for the v1 document.]**

### Key specifications (v2)

| Parameter | Value |
|---|---|
| Dimensions | **176 × 125 mm** footprint (front overhang 41.2 mm, rear 21.8 mm, measured) · height **≈120 mm** including cables (104 mm to the top of the camera mount in CAD) |
| Mass | **[TODO: weigh]** g (the simulator uses 564 g) |
| Wheelbase / rear track | 113 mm / 110 mm |
| Drive | Rear-wheel drive, one motor, printed open differential |
| Drive motor | N20 50:1 metal gear motor, 460 rpm, 12 CPR Hall encoder |
| Final drive | Spur gears 1.25 : 1 (total 62.5 : 1) — top speed ~800 mm/s, capped at 700 mm/s by the speed loop |
| Steering | SG90 servo, rack and pinion (module 1, 19 teeth), Ackermann linkage; min. turning radius **123 mm right / 134 mm left** (measured on the mat) |
| High-level computer | Raspberry Pi 5 (8 GB) — camera, vision, mapping, localization, route planning |
| Real-time controller | ESP32 DevKitC (dual core) — odometry, route following, speed loop, state machine, all actuators and sensors |
| Link | UART 460 800 baud, line protocol with XOR checksum |
| Camera | Raspberry Pi Camera v2 (IMX219) with NoIR wide-angle lens, 640 × 480 at ~40 fps, fixed exposure |
| Distance sensors | 4 × VL53L1X time-of-flight (front, left, right, rear) |
| IMU | BNO085, fused yaw over UART-RVC at 100 Hz |
| Motor driver | TB6612FNG (SparkFun ROB-14450 carrier) |
| Battery | 3S LiPo 11.1 V, 2200 mAh |
| Regulation | 2 × MINI560 5 V buck (one only for the Pi 5) + 3.3 V regulator for the sensors |
| PCB | Two stacked 2-layer boards, 117.5 × 50.6 mm each (KiCad, made by JLCPCB) |

### System block diagram

```mermaid
flowchart LR
  subgraph PWR["Power board (RevB_2)"]
    BAT["3S LiPo 11.1 V<br/>2200 mAh"] --> SW["Power switch"]
    SW --> BUCKPI["MINI560 #2<br/>5 V — Pi only"]
    SW --> BUCK["MINI560 #1<br/>5 V — logic + servo"]
    SW --> DRV["TB6612FNG<br/>motor driver"]
    BUCK --> REG33["3.3 V regulator<br/>sensors"]
  end
  subgraph PI["Raspberry Pi 5"]
    CAM["Camera IMX219<br/>NoIR wide"] --> VIS["Vision + BEV"]
    VIS --> MAP["Digital map<br/>of pillars"]
    MAP --> LOC["EKF localizer"]
    LOC --> PLAN["Route planner (DP)<br/>+ speed profile"]
  end
  subgraph ESP["ESP32"]
    ODO["Odometry<br/>encoder + yaw"] --> PP["Pure pursuit<br/>on the route"]
    PP --> SM["State machine<br/>start / race / park"]
    SPD["Speed loop<br/>FF + PI"]
  end
  BUCKPI --> PI
  BUCK --> ESP
  REG33 --> TOF["4 × VL53L1X ToF"]
  REG33 --> IMU["BNO085"]
  PLAN -- "route RTB/RTP/RTE<br/>+ V2 line (UART 460800)" --> ESP
  ESP -- "ACK: pose, ToF, state" --> LOC
  TOF -- "I²C" --> ESP
  IMU -- "UART-RVC 100 Hz" --> ESP
  ENC["Encoder 12 CPR"] -- "PCNT ×4" --> ESP
  SM --> SERVO["SG90 steering"]
  SPD --> DRV --> MOT["N20 50:1 motor"]
  BTN["Start button"] --> PI
  PI --> LED["READY LED"]
```

The **Raspberry Pi 5** answers *where should the car go*: it sees the pillars and lines, keeps a map of the field, estimates the car's pose and plans the route. The **ESP32** answers *how to drive there right now*: it integrates odometry at loop rate, steers along the route with pure pursuit, holds the speed with the encoder and runs every maneuver. The reason for this split, and why the route follower moved from the Pi to the ESP32 in v2, is in [§11.2](#112-trade-offs--we-chose-x-instead-of-y-because).

---
# 5. Mobility and Mechanical Design

## 5.1 Chassis — shorter car, battery in the middle, Pi on top

Every structural part is 3D-printed in PLA (SUNLU) on a Bambu Lab A1 and designed in SolidWorks. The wheels and axles are LEGO parts: LEGO axles keep their dimensions far better than printed shafts, which flex and misalign under load.

v2 CAD is in [`models/v2/`](models/v2/): SolidWorks parts + assembly in [`cad/`](models/v2/cad/), one STEP per part in [`step/`](models/v2/step/) and printable STL in [`stl/`](models/v2/stl/), with the parts list in [`models/v2/README.md`](models/v2/README.md). Renders of the full assembly are in [`models/v2/renders/`](models/v2/renders/). The top-level `models/` folder is the v1 car.

<p align="center"><img src="models/v2/renders/turntable.gif" width="420" alt="v2 car, full CAD assembly rotating"></p>

| Isometric | Top | Bottom |
|:---:|:---:|:---:|
| <img src="models/v2/renders/isometric.png" height="230"> | <img src="models/v2/renders/top.png" height="230"> | <img src="models/v2/renders/bottom.png" height="230"> |

For v2 we rebuilt the chassis around two problems measured on v1:

1. **Length.** v1 was 210 mm long, ≈240 mm with an extension we added so it could drive out of the parking lot. Every dodge swept a lot of floor and the parking maneuver had little room.
2. **Heat.** The Raspberry Pi 4 sat enclosed in the middle of the chassis and reached 84 °C with thermal throttling, which lowered the vision frame rate.

| Change | How | Why |
|---|---|---|
| **Motor parallel to the rear axle** | The motor lies along the axle and drives the differential through a spur pair, instead of a perpendicular bevel pair | A motor perpendicular to the axle adds its whole length to the car; parallel, it adds almost nothing |
| **Battery in the center** | The 3S pack sits crosswise in a side-loading tunnel, where the Pi used to be | The heaviest part is low and centered, which balances the load front to rear; the side tunnel makes swaps fast |
| **Raspberry Pi 5 on top** | The Pi moves from the enclosed bay to the top of the car | Free air for cooling; camera cable and SD card stay reachable |
| **Stacked PCB in the middle** | Power board below, logic board above, on standoffs (§6.3) | Half the footprint of a single board |
| **Result** | **176 × 125 mm** (measured) — 27 % shorter than v1 | Smaller turning radius and a smaller parking lot to fit in |

**Why we kept the height.** The chassis height is about the same as v1 for stability in fast turns, and the camera height (95 mm) and tilt (≈25°) stay close to v1 so the bird's-eye-view geometry carries over (it is still re-calibrated on every mount, §6.4).

**Material choice.** PLA prints fast, does not warp and is stiff enough indoors. PETG (impact) and ABS (heat) offer properties we do not need on a flat indoor mat, while PLA let us reprint a part in under an hour when a test showed a problem.

## 5.2 Drivetrain — gear ratio chosen from measured speed

**Components:** N20 50:1 metal gear motor (460 rpm no-load, 12 CPR Hall encoder) → spur pair **20 T → 25 T (1.25 : 1)** → printed open differential → LEGO rear axles → LEGO wheels Ø 42.7 mm (effective, measured by rolling 50 cm).

The involute gear profiles are generated by our own script ([`Mecanica/Engranes/gen_pinon.py`](Mecanica/Engranes/gen_pinon.py)) and exported as DXF for SolidWorks, with 0.10 mm of backlash for printing. We printed four ratios with the same center distance so we could swap them: 15/30 (2 : 1), 18/27 (1.5 : 1), 20/25 (1.25 : 1) and 23/22.

**Speed per ratio.** Wheel speed at no load = 460 rpm / ratio; linear speed = rpm × π × 42.7 mm / 60.

| Spur ratio | Total reduction | Wheel rpm (no load) | Top speed (no load) | Wheel torque vs. 2 : 1 | Notes |
|---|---:|---:|---:|---:|---|
| 2 : 1 (v1 ratio) | 100 : 1 | 230 | 514 mm/s | 1.00 | Lap-time limited on the straights |
| 1.5 : 1 | 75 : 1 | 307 | 685 mm/s | 0.75 | Evaluated in the twin |
| **1.25 : 1 (v2)** | **62.5 : 1** | **368** | **822 mm/s** | **0.63** | **Measured: PWM 255 → ~800 mm/s (97 % of no-load)** |

**Why 1.25 : 1.** We measured the car's dynamics on the competition mat (2026-10-24, battery 12.1 → 11.9 V, logs and method in [`docs/route-following-bench.md`](docs/route-following-bench.md)):

| Measurement | Result |
|---|---|
| Speed at PWM 150 / 255 | ~400 mm/s / ~800 mm/s |
| Lateral grip limit (start of sliding in a constant-radius arc) | **2.5–2.8 m/s²** |
| Coasting deceleration (motor off) | ≈ 2.7 m/s², constant |
| Short brake from PWM 150 | stops in 6–7 mm |
| Short brake from PWM 255 | wheels lock: the encoder sees 13–23 mm, the car slides 24–51 mm more |

The lateral grip, not the motor, limits the speed in every corner and dodge: at the 2.6 m/s² limit, a 300 mm radius allows √(2.6 × 0.30) ≈ 0.88 m/s. So the useful extra speed is on the straights, where the 2 : 1 ratio capped us at 514 mm/s. With 1.25 : 1 the car reaches ~800 mm/s on a straight, and the speed profile (§8.6) brakes before each curve to the grip-limited speed. We did not go further (23/22): the motor already runs near its no-load speed at 255, braking from full speed already locks the wheels, and lower ratios also cut the torque available to accelerate out of the parking lot.

**Torque check.** To accelerate the car at the grip limit, the wheels need F = m·a = 0.564 kg × 2.7 m/s² ≈ 1.5 N, i.e. a torque of 1.5 N × 21.35 mm ≈ 0.033 N·m (0.33 kg·cm) at the wheels, or 0.033 / 1.25 ≈ 0.026 N·m (0.27 kg·cm) at the gear-motor output. The vendor rates the N20 50:1 at **20 oz·in ≈ 0.14 N·m (1.44 kg·cm) stall torque at 12 V**, so the worst-case need is about **18 % of stall — a ≈5× margin**, inside the region where a brushed DC motor runs efficiently. In practice the car accelerates from the lot to cruising speed in under 0.5 m, and the measured speed reached 97 % of no-load, which means the motor is lightly loaded.

**Speed loop.** Without an encoder the speed of v1 changed with the battery voltage (a fresh pack made the maneuvers ~17 % stronger). In v2 the ESP32 converts the requested PWM into a target speed with a measured feed-forward model (forward: v ≈ 3.82 · (PWM − 45.5) mm/s) and closes a PI loop on the encoder speed (Kp 0.10 PWM per mm/s, Ki 0.60, integral clamped at ±80 PWM, encoder speed filtered with τ = 50 ms). The target is capped at **700 mm/s**, which keeps a margin against wheel lock under braking. In the simulator, the time from start to the parking lot stayed flat (29.4 s) from 11.1 V to 12.2 V with the loop, against 29.25 → 26.10 s without it.

**Encoder.** 12 counts per motor revolution × 50 × 1.25 = 750 counts per wheel revolution in theory = 5.59 counts/mm; we **measured 5.45 counts/mm** by pushing the car 1 m along a tape (the 2.5 % difference is the real rolling radius of the tire under load). The ESP32 counts it with its **PCNT hardware counter** in ×4 quadrature: the first firmware used interrupts, which lost ~25–65 % of the pulses on the floor under motor load (see [§11.3](#113-risks-failure-modes-and-mitigations)).

## 5.3 Steering — Ackermann rack and pinion

Steering uses an SG90 servo that turns a module-1 pinion; the pinion drives a printed rack, and two tie rods move the steering knuckles. A rack pushes both tie rods symmetrically, so both wheels get the correct angle at every position, unlike a single servo arm.

### v1 problems and the redesign

We simulated the linkage in Python with the exact CAD dimensions (the model matched the CAD within ~1°) before reprinting anything.

**Problem 1 — v1 was anti-Ackermann.** In a turn the inner wheel must steer *more* than the outer one. Ideal (100 %) Ackermann is reached when cot(δ_outer) − cot(δ_inner) = T / L = 60 / 112.85 = 0.532 (T = kingpin track, L = wheelbase). In v1 the outer wheel steered *more* (39.5° inner, 42.8° outer), so one front wheel always dragged sideways. With the knuckles fixed, the only free parameters were the spacing of the rack joints and the tie-rod length, which must change together for the wheels to point straight with the rack centered: linkage = √((38.66 − spacing/2)² + 9.94²).

| Rack joint spacing | Tie rod | Ackermann at 30° | 40° | 45° | 50° | 55° |
|---|---|---|---|---|---|---|
| 28.5 mm (v1) | 26.0 mm | anti-Ackermann | | | | |
| 43.0 mm | 19.83 mm | 51 % | 62 % | 69 % | 76 % | 84 % |
| **47.0 mm (v2)** | **18.13 mm** | 76 % | **89 %** | **95 %** | **102 %** | 110 % |
| 50.0 mm | 16.89 mm | 99 % | 111 % | 117 % | 124 % | 131 % |

We chose **47 mm** because it is closest to 100 % between 40° and 50°, where the car actually turns in corners and while parking. At small angles the error is under 0.7°.

**Problem 2 — v1 could not turn tightly enough.** The steering angle depends almost only on how far the rack moves. We compared shorter knuckle arms, moving the rack, and a bigger pinion; only the bigger pinion needed no new chassis or knuckles.

| | v1 | **v2** |
|---|---|---|
| Pinion | module 1, 14 teeth | **module 1, 19 teeth** (servo moved 2.5 mm away from the rack) |
| Rack travel per servo degree | 0.122 mm/° | **0.166 mm/°** |
| Inner / outer wheel at full lock (CAD) | ~37° / ~32° | **53.1° / 37.0°** (~106 % Ackermann) |
| Wheelbase | ≈147 mm | **113 mm** |
| Turning radius at the rear-axle center | ~208 mm | **123 mm right / 134 mm left (measured on the mat)** |

### Measured on the real car

The CAD predicts the geometry; the mat decides. With the car assembled we ran constant-servo circles at PWM 150 on the competition mat, 3 repetitions each, and measured the radius at the rear-axle center from odometry (2026-10-22):

| Servo command | 1 (right stop) | 4 | 8 | 16 | 75 (center) | 140 | 144 | 148 | 152 | 156 (left stop) |
|---|---|---|---|---|---|---|---|---|---|---|
| Radius (mm) | **123** | 130 | 142 | 166 | ∞ | 153 | 140 | 136 | **134** | 136 |

What this taught us:
- **The usable servo range is 1–156** (mechanical stops), with **75 = straight** (straight runs at PWM 150 approached from both sides). At 160 the servo strains against the stop with no gain in radius.
- **Left turns are ~15 % weaker than right turns.** Moving the wheels by hand at full left lock showed the play is in the *outer* wheel's linkage. The servo backlash is ±1.5 servo units and the wheels follow the servo with a **0.10 s lag**.
- The servo → wheel relation is **not linear** near the stops. The firmware therefore converts the wheel angle requested by the controller into a servo command with a **measured piecewise table per side** (`rutaServo()` in the firmware), and the simulator uses the same table.

| Wheel angle (bicycle model) | 0° | 7.6–8.7° | 15.6–18.2° | 53° |
|---|---|---|---|---|
| Servo units from center — right | 0 | 13.5 | 28.5 | 72.5 |
| Servo units from center — left | 0 | 13.5 | 28.5 | 71.5 |

**Servo load.** The bigger pinion needs ~1.36× more servo torque for the same rack force, still fine for an SG90 on a ~0.56 kg car. The servo must not stay stalled against a stop: it heats up and loads the 5 V rail, which is why software limits are set at the measured stops.

## 5.4 Camera mount

The camera sits at the front on the centerline, tilted down. We measured its real pose from a calibration image of floor crosses (perspective-n-point fit of the homography, §6.4): **tilt 24.8°, height 95 mm, 152 mm ahead of the rear axle**, equivalent pinhole horizontal FOV 74°. The near image edge sees the floor 207 mm ahead of the rear axle and the row at y = 100 px sees 963 mm ahead, which covers both pillar rows of a straight. The mount must be rigid: a few millimeters of change move the bird's-eye-view calibration, so the mount is re-checked with a calibration image after any crash.

## 5.5 Assembly order (v2)

1. **Chassis base** — reference part for everything else.
2. **Steering** — 19-tooth pinion on the servo (with the 2.5 mm spacer), servo into its pocket, rack, tie rods (trim in 0.05–0.1 mm steps until the wheels are straight), knuckles and front LEGO axles. Set the servo to 75 and check the car drives straight.
3. **Drivetrain** — gear motor into its holder parallel to the axle, 20 T pinion on the motor D-shaft, differential with the 25 T ring, rear LEGO axles with bushings and wheels.
4. **Sensors** — four ToF mounts (front, left, right, rear; ~50 mm above the floor, level), BNO085 near the center of rotation, camera mount.
5. **Electronics** — power board (RevB_2) on its standoffs, logic board (RevB_1) on top **in the keyed orientation** (see the warning in §6.3), battery into the center tunnel, Raspberry Pi 5 on top.
6. **Cables** — camera ribbon (22-to-15-pin), ToF and IMU leads, encoder, servo, motor.

---
# 6. Power and Sensor Architecture

## 6.1 Power tree

```mermaid
flowchart TD
  BAT["3S LiPo 11.1 V nominal (12.6 V full)<br/>2200 mAh, 120C"] --> SW2["SW2 power switch<br/>+ C1 470 µF / C2 0.1 µF"]
  SW2 --> V11["+11.1 V rail"]
  V11 --> TB["TB6612FNG (ROB-14450)<br/>VM = battery"] --> M["N20 drive motor"]
  V11 --> B1["MINI560 #1 → +5 V<br/>C3 100 µF + 0.1 µF"]
  V11 --> B2["MINI560 #2 → +5V_PI<br/>C5 100 µF + 0.1 µF"]
  B2 --> PI["Raspberry Pi 5 (GPIO 5 V pins)<br/>+ camera"]
  B1 --> ESP["ESP32 DevKitC (5 V pin)"]
  B1 --> SERVO["SG90 servo"]
  B1 --> LOGIC["TB6612 logic, encoder Vcc,<br/>level shifter HV side"]
  B1 --> R33["3.3 V regulator (J6)"]
  R33 --> TOF["4 × VL53L1X"]
  R33 --> BNO["BNO085"]
  R33 --> PU["Encoder pull-ups 4k7"]
```

**Why two 5 V regulators.** The Raspberry Pi throttles its CPU when its supply sags, which directly lowers the vision frame rate. We saw this on v1 when the Pi 4 was fed through thin jumper wires (`throttled=0x50005`, CPU at 600 MHz). The servo draws current spikes of up to ~0.7 A. In v2, MINI560 #2 feeds **only** the Raspberry Pi 5, so servo and logic spikes on MINI560 #1 can never reach the Pi.

**Why a separate 3.3 V regulator for the sensors.** The ESP32 DevKit's on-board LDO also feeds the ESP32 itself. Four ToF sensors plus the BNO085 and the encoder pull-ups are kept on their own 3.3 V regulator (connector J6) so their current does not heat or load the ESP32's LDO.

**Why the motor runs from the battery.** The TB6612FNG takes battery voltage directly, so motor stall currents never pass through the 5 V regulators.

## 6.2 Power budget

*Design values from datasheets. **[TODO: replace with values measured with an inline USB/DC meter on the assembled car: idle, racing, and servo at the stop.]***

| Rail | Load | Typical | Design peak | Basis |
|---|---|---:|---:|---|
| **+5V_PI** (MINI560 #2, 5 A rated) | Raspberry Pi 5 under vision load (4 cores ~55 %) | 1.3 A | 2.2 A | Raspberry Pi 5 power figures; the loop uses ~2.2 cores |
| | Camera v2 | 0.25 A | 0.25 A | Module figure |
| | **Rail total** | **1.55 A** | **2.45 A** | 49 % of the 5 A peak rating |
| **+5 V** (MINI560 #1) | ESP32 (radio off in competition) | 0.08 A | 0.25 A | Datasheet |
| | SG90 servo | 0.25 A | 0.70 A | Running / stall |
| | TB6612 logic, encoder, level shifter | 0.02 A | 0.03 A | Datasheets |
| | 3.3 V regulator → 4 × VL53L1X + BNO085 | 0.09 A | 0.18 A | ~20 mA each ranging (40 mA peak) + 13 mA |
| | 3 × HC-SR04 (cross-check) | 0.045 A | 0.045 A | 15 mA each (datasheet) |
| | **Rail total** | **0.49 A** | **1.21 A** | 24 % of the 5 A peak rating |
| **Battery** | Drive motor via TB6612 | ~0.3 A cruising (no-load 0.1 A) | 1.6 A stall @ 12 V (vendor) | TB6612: 1.2 A continuous, 3.2 A peak per channel — a stall is above the continuous rating but under the peak, so it is only acceptable for short events; the anti-stall check in §7.4 notices a stopped wheel within 250 ms |

**Battery current.** With ~90 % buck efficiency, both 5 V rails at design peak draw 5 V × (2.45 + 1.21) A / 0.9 ≈ 20 W, i.e. ≈ 1.8 A from the 11.1 V pack, plus the motor. The 2200 mAh pack lasts well over 30 minutes at that worst case against a 3-minute round, so **capacity is not the constraint; voltage is.** The speed loop (§5.2) removes most of the voltage dependence, and we still check the pack before each round (good window 11.7–12.2 V).

**Pi supply check.** `vcgencmd get_throttled` must read `0x0` under full load. On the battery the Pi rail measured **5.07 V at idle, throttled `0x0`**.

## 6.3 PCB RevB — two stacked boards

v1 had one board shaped around the Raspberry Pi 4. v2 adds four ToF sensors (each with its own XSHUT line), the BNO085, the encoder, a second regulator and a power switch, on a chassis 27 % shorter. All of that does not fit on one footprint, so v2 uses **two 2-layer boards stacked on 11-pin and 4-pin headers**, each 117.5 × 50.6 mm, designed in KiCad 10 and made by JLCPCB (5 pcs per design, ~40 USD total with shipping).

| Board | Contents | Why on this board |
|---|---|---|
| **RevB_2 — power** (bottom) | Battery input, MINI560 #1 and #2 with their capacitors, TB6612FNG carrier, encoder connector with 4k7 pull-ups, 4 ToF connectors (I²C + XSHUT), 3 HC-SR04 connectors (TRIG + level-shifted ECHO) for the ToF cross-check | Keeps the switching regulators and the motor currents, with their heat and noise, away from the logic; heaviest board, so it sits low |
| **RevB_1 — logic** (top) | ESP32 DevKitC, power switch + bulk capacitors, level shifter, BNO085 header, servo header, 3.3 V regulator header, `TO_PI` connector (UART, start button, READY LED, 5 V for the Pi), PWR and READY LEDs | All connectors the team touches between runs are on top and reachable |

| | Logic board RevB_1 | Power board RevB_2 |
|---|---|---|
| Top render | <img src="electrical/export/WRO_RevB_1_top.png" width="420" alt="RevB_1 top render"> | <img src="electrical/export/WRO_RevB_2_top.png" width="420" alt="RevB_2 top render"> |
| Schematic (PDF) | [WRO_RevB_1_schematic.pdf](electrical/export/WRO_RevB_1_schematic.pdf) | [WRO_RevB_2_schematic.pdf](electrical/export/WRO_RevB_2_schematic.pdf) |
| KiCad project | [`electrical/WRO_RevB_1/`](electrical/WRO_RevB_1/) | [`electrical/WRO_RevB_2/`](electrical/WRO_RevB_2/) |

> ⚠️ **Assembly warning learned the hard way.** The two boards are almost symmetric and the stacking headers are not keyed. On 2026-11-06 we mounted the logic board rotated 180°: the battery pin landed on the Pi's 5 V net and **killed a Raspberry Pi 5** (PMIC failure, ~11 V on the 5 V rail). We now mark the orientation on both boards and measure `TO_PI` pins 7/8 = 5.0–5.2 V **without the Pi** before mounting it. Next revision: keyed headers, a PTC fuse and a 5 V TVS diode (SMBJ5.0A) on +5V_PI. Details in [§11.3](#113-risks-failure-modes-and-mitigations).

### Wiring / pin map (ESP32)

| Signal | ESP32 GPIO | Connected to | Notes |
|---|---|---|---|
| Motor PWM | 23 | TB6612 PWMA | |
| Motor direction | 18 (A1), 19 (A2) | TB6612 AIN1/AIN2 | Direction confirmed on the car, not from the schematic labels |
| Steering servo | 13 | SG90 signal | 50 Hz; command range 1–156, center 75 |
| Encoder A / B | 34 / 36 | N20 Hall encoder | Input-only pins → external 4k7 pull-ups to 3.3 V; PCNT ×4 |
| I²C SDA / SCL | 21 / 22 | 4 × VL53L1X | 4k7 pull-ups, addresses 0x30–0x33 assigned at boot |
| ToF XSHUT front / left / right / rear | 0 / 15 / 4 / 5 | VL53L1X XSHUT | Held low at boot, released one by one |
| BNO085 TX → ESP RX | 25 | BNO085 (UART-RVC) | 115200 baud, 100 Hz packets |
| UART to the Pi | 17 (RX) / 16 (TX) | Pi GPIO 14 / 15 (`/dev/ttyAMA0`) | 460 800 baud, `Serial2` started from core 0 |
| Start button | — (Pi GPIO 17) | SW1 `EXEC_PRGM` on the logic board → `TO_PI` pin 6 | Read by the Pi |
| READY LED | — (Pi GPIO 27) | D2 on the logic board → `TO_PI` pin 5 | Driven by the Pi |
| HC-SR04 left / right / front TRIG | 27 / 26 / 14 | HC-SR04 TRIG | 10 µs pulse, one sensor at a time |
| HC-SR04 left / right / front ECHO | 32 / 35 / 33 | HC-SR04 ECHO via the level shifter | 5 V → 3.3 V; read with a CHANGE interrupt |

## 6.4 Sensors — selection, placement and calibration

### Sensor set and why

| Sensor | What it measures | Why this one | Placement (and the field geometry behind it) |
|---|---|---|---|
| **Camera** IMX219 + NoIR wide lens | Pillars (red, green), parking walls (magenta), orange and blue corner lines | Only sensor that sees color; the wide lens sees both pillar rows of a straight and the corner line in one frame | Front center, 95 mm high, 24.8° down: the floor from 0.2 m to ~1 m ahead of the rear axle — one straight section is 1 m deep |
| **VL53L1X ToF ×4** | Distance to walls and the parking-lot walls | Narrow optical cone with a software-set region of interest (no echoes like the ultrasonic, ERR-19); 4 sensors can range in parallel | ~50 mm high (the walls are 100 mm high), level. Front: wall ahead (corners, start fix). Left/right: walls of the straight (localization). Rear: parking and reversing |
| **BNO085** | Yaw (heading) | Sensor fusion on the chip with continuous bias calibration; no integration on our busy loop | Near the center of rotation, so centripetal acceleration stays out of the fused heading |
| **Encoder** 12 CPR | Distance and speed | Closed-loop speed and odometry | On the motor; read by hardware (PCNT) |
| **HC-SR04 ×3** (cross-check) | Distance to the walls (left, right, front) | Fails for different reasons than the ToF: sound does not care about the wall's color, light does not care about echoes. Used to check the ToF, not to drive (see below) | Same sides as the ToF; the side sensors 28.3 mm further out, the front one 6.2 mm further forward |

### VL53L1X — making a ToF see a black wall

The inner walls are black, which absorbs most of the 940 nm light. What we measured and changed:

| Finding (evidence) | Change |
|---|---|
| The default 16 × 16 region of interest sees the white **floor** at long range and reads short: static test at 60 / 80 cm → −50 / −190 mm. On the track the right sensor saturated at 460–500 mm beyond ~45 cm (runs 35–42) and dragged the pose toward that wall | Left/right/rear use **ROI 8 × 8** (static error −8 / −40 mm at 60 / 80 cm), commit `8af0b2a2` |
| On the black wall many valid distances come with **range status 2** (signal fail) or 7 (wrap target fail) | Short-mode sensors accept status 2 up to 900 mm (`889aa0ce`); the front (Long mode) accepts only status 0, because its status 2 reads ~+20 cm long |
| The front sensor reads ~+20 cm long beyond 60 cm and is only reliable ≤ 80 cm, and only if mounted level | The localizer uses the front sensor only within its reliable range; the start fix gives it no vote; the mount is rigid and level |
| A sensor can freeze (same value) or fail at boot | Firmware skips a sensor that failed init and reports it (`ltf=` field in the ACK); in the open-round firmware a reading stale for > 300 ms re-initializes that sensor through its XSHUT pin |

Timing: left/right/rear in **Short** mode with a 20 ms budget; front in **Long** mode with 33 ms. The ACK to the Pi reports the worst I²C time per sensor, so a slow sensor shows up in the log, not as a mystery in the driving.

### Ultrasonic cross-check (in progress)

The ToF and the ultrasonic fail in opposite situations. The ToF measures light, so a black wall far away returns little signal and the reading goes long or short (table above). The HC-SR04 measures sound: the wall color does not matter, but its wide cone picks up pillars and multipath echoes (ERR-19 in v1). When both agree, the reading is trustworthy; when they disagree, one of them is wrong and that reading should not move the pose.

| Step | What | Status |
|---|---|---|
| 1 | Non-blocking read in `PurePursuit.ino`: one sensor fired every 30 ms (so one sensor's echo is not heard by the next), echo timed with a CHANGE interrupt instead of `pulseIn()` (which would block the loop up to 25 ms). ~11 Hz per sensor, ~10 µs of loop time per ping | Written, not yet on the car |
| 2 | Both readings in every ACK (`tL/tR/tF` next to `uL/uR/uF`), recorded in the journal; [`scripts/compara_tof_us.py`](scripts/compara_tof_us.py) reports, per sensor and distance band, the median ToF − US difference and how often they disagree | Written, not yet on the car |
| 3 | Static test like the ToF one: black and white wall at 30 / 60 / 80 / 120 cm, both sensors in one table | Pending (needs the car) |
| 4 | If the data supports it: the EKF drops a ToF reading that disagrees with the ultrasonic by more than the measured threshold, and the front ultrasonic covers 80–200 cm, where the front ToF is unreliable | Pending |

### BNO085

UART-RVC mode: the sensor streams a 19-byte packet with yaw, pitch, roll and acceleration every 10 ms on one wire (BNO TX → GPIO 25), with a checksum. No driver library, and the IMU stays off the I²C bus the four ToF sensors share. Calibration and checks:
- **Sign.** The yaw sign is −1 for our mounting (`BNO_YAW_SIGN`). A wrong sign mirrors the whole pose: it cost us a day of track runs on 2026-11-09 (the car "turned before the wall").
- **Scale.** Hand-rotate the car 1080° in the waiting state and compare (`BNO_ESCALA`); the bench result was 1.0.
- **Acceptance.** ≥ 99 % of packets at ~100 Hz with the motor running; heading unchanged with the motor running at standstill (magnetic interference check).

### Camera and color

| Step | What and why | Where |
|---|---|---|
| **Capture** | Picamera2: full sensor mode 1640 × 1232, scaled to **640 × 480 by the Pi 5's image processor**. Our first Pi 5 pipeline scaled on the CPU (GStreamer) and limited the loop to 20–23 fps at 245 % CPU; the ISP path delivers ~40 fps at 5 % CPU with the same framing (`66ca91aa`) | `vision.open_camera` |
| **Lens shading** | The NoIR lens has no IR filter, and the Pi 5's default lens-shading table turned the frame edges magenta (R/G 1.31–1.40 at the corners) — close to the pink parking walls and red pillars. We calibrated our own table from 5 raw photos of a white sheet with the official tuning tool: R/G and B/G = 1.00 ± 0.01 at the center and all corners | [`camera_tuning/`](src/RASPI/cam/camera_tuning/README.md) |
| **White balance** | Fixed gains `<1.10, 1.37>` (auto white balance drifts with the colored walls in view) | `vision.py` |
| **Exposure** | **Fixed** exposure measured per venue. With auto exposure, inside the magenta parking lot the exposure rose 2.3×; leaving the lot the floor was burned for ~1 s, the orange line was seen late and the car hit the island (run 57) | `pure_pursuit/mide_exposicion.py`, `camera_tuning/exposicion.json`, `c6757575` |
| **Color classes** | Pillars are classified by hue **and** channel ratios, measured on our track (~950 lux): red H 170–177, S 144–158, G/R 0.38–0.44; pink wall H 139, G/R 0.67, B/R 1.19; orange tape G/R 0.76–0.78; green H 71, G/R 1.95. Ratios stay stable when the venue light changes; hue near red wraps around and shifts. Thresholds: red S ≥ 110, G/R ≤ 0.60, B/R ≤ 0.70 | `vision.py`, [`calibra_luz.py`](src/RASPI/cam/pure_pursuit/calibra_luz.py) |
| **Bird's-eye view (BEV)** | Homography from floor crosses at known positions (rear-axle frame) → 1 px = 2 mm on the floor. Fit error 17 mm mean, 36 mm max; the residual pattern is lens barrel distortion. A perspective-n-point fit of the same homography gives the camera pose used by the twin (§5.4) | `bev.py`, `calibrate.py`, `bev_calib.npz` |

## 6.5 Main computer — Raspberry Pi 5

**Why we upgraded.** Profiling the race code on the Pi 4: the main thread used 97 % of a core and the loop ran at 14–18 fps; vision, not capture, was the cost (capture ~5 ms; color 17–22 ms; BEV + centerline 45–48 ms). With the enclosure it reached 84 °C and throttled.

| | Raspberry Pi 4 (v1) | Raspberry Pi 5 (v2) |
|---|---|---|
| Vision loop | 14–18 fps | ~40 fps (camera-limited) with ISP scaling |
| Full v2 pipeline per frame (vision + map + localizer) | 60 ms | **23.5 ms mean, 32 ms p95** |
| Route re-plan (dynamic programming) | ~190 ms median | **59–70 ms** (peak 223 ms), in a background thread |
| First plan at start | 2.6 s | 1.95 s (done before the READY LED, so it does not count in the run time) |

**What the port required:** UART on `/dev/ttyAMA0` (on the Pi 5 `serial0` is the debug connector); `rpi-lgpio` instead of `RPi.GPIO`; a 22-to-15-pin camera cable; `camera_auto_detect=0` + `dtoverlay=imx219,cam0`; system OpenCV from apt. Details in [§12](#12-how-to-build-and-run).

**Open item — cooling.** The replacement Pi 5 (8 GB, after the one lost in the incident above) runs without a fan and sits at ~80 °C with soft-throttle flags during long sessions. **[TODO: mount the Active Cooler / a fan and record the temperature over a 3-round session.]**

---
# 7. Software Architecture

## 7.1 Two controllers, one link

| | Raspberry Pi 5 — Python 3, OpenCV, NumPy | ESP32 — Arduino C++ (dual core) |
|---|---|---|
| Role | Perception and planning: *where should the car go?* | Real-time control: *how do we drive there now?* |
| Rate | One cycle per camera frame, ~40 Hz | Main loop at sensor rate; BNO085 at 100 Hz; UART RX in its own task on core 0 |
| Owns | Camera, start button, READY LED, recording | Servo, motor, encoder, 4 ToF, BNO085 |
| Key outputs | The route (list of points in mm), the allowed speed, the pose correction | Odometry pose, sensor readings and state, sent back in every ACK |
| If the other fails | — | Keeps following the last complete route it received; stops if it never got one |

**Why split this way.** Vision is heavy and its time per frame varies (20–35 ms); steering and speed control need a steady rate. Running both on one processor would make the control loop jittery. The split also lets each side be tested alone: the ESP32 can follow a route sent from a PC with no camera ([route-following bench](docs/route-following-bench.md)), and the Pi code can run on recorded video.

**Why the route follower runs on the ESP32 (new in v2).** In v1 the Pi computed the steering from the image and sent it every frame. The pose reached the Pi 4–6 frames late (the ACK travels the UART after the frame was captured), so the Pi steered for where the car *had been*. In v2 the Pi sends the **whole route** once, and the ESP32 steers against its own odometry at loop rate with no communication delay. The Pi only corrects the pose and changes the route when the map changes.

## 7.2 Module map

**Raspberry Pi** — [`src/RASPI/cam/pure_pursuit/`](src/RASPI/cam/pure_pursuit/)

| Module | Responsibility | Talks to |
|---|---|---|
| [`runtime_nuevo.py`](src/RASPI/cam/pure_pursuit/runtime_nuevo.py) | Entry point. Main loop: frame → perception → map → localization → route; serial link, start button, READY LED, HUD and recording | All modules |
| [`config.py`](src/RASPI/cam/pure_pursuit/config.py) + [`carrera.json`](src/RASPI/cam/pure_pursuit/carrera.json) | Every tunable parameter with its unit and why; `carrera.json` is the exact race configuration exported from the simulator | — |
| `../vision.py`, [`color_corr.py`](src/RASPI/cam/pure_pursuit/color_corr.py) | Camera capture (Picamera2), color correction, red/green pillar and magenta detection (lookup-table masks + ratio tests) | `runtime_nuevo` |
| [`bev.py`](src/RASPI/cam/pure_pursuit/bev.py) | Bird's-eye-view homography, image ↔ floor (mm) | vision, corner lines, map |
| [`corner_lines.py`](src/RASPI/cam/pure_pursuit/corner_lines.py) | Orange/blue corner lines, direction of travel | map, start fix |
| [`digital_map.py`](src/RASPI/cam/pure_pursuit/digital_map.py) | Map of the field: votes each pillar detection into one of the 6 possible pillar **seats** of each straight | route mode |
| [`localizer.py`](src/RASPI/cam/pure_pursuit/localizer.py) | EKF on (x, y, heading): odometry from the ESP32 + ToF distances against the known walls | route mode |
| [`start_fix.py`](src/RASPI/cam/pure_pursuit/start_fix.py), [`salida_cajon.py`](src/RASPI/cam/pure_pursuit/salida_cajon.py) | Blind start: where along the straight the parking lot is; which side to leave the lot | route mode, ESP32 |
| [`route_mode.py`](src/RASPI/cam/pure_pursuit/route_mode.py) | Which seats are known, when to re-plan, speed allowed, uploading the route | ESP32 |
| [`ring_route.py`](src/RASPI/cam/pure_pursuit/ring_route.py) | Route planner: dynamic programming over the lap + B-spline smoothing | route mode |
| [`wro_field.py`](src/RASPI/cam/pure_pursuit/wro_field.py) | Field geometry and pillar-placement rules from the rulebook | map, planner, twin |
| [`cam_web.py`](src/RASPI/cam/pure_pursuit/cam_web.py), [`bev_recorder.py`](src/RASPI/cam/pure_pursuit/bev_recorder.py) | Live camera/HUD in a browser; per-run recording of the video and the clean BEV | tools |

**ESP32** — [`src/ESP32/PurePursuit/PurePursuit.ino`](src/ESP32/PurePursuit/PurePursuit.ino) (development) → [`src/ESP32/PurePursuitCarrera/PurePursuitCarrera.ino`](src/ESP32/PurePursuitCarrera/PurePursuitCarrera.ino) (race build, exported with the race parameters by [`exportar_carrera.py`](src/RASPI/cam/pure_pursuit/twin/tools/exportar_carrera.py) so the car and the simulator run the same source)

| Block | Functions | Notes |
|---|---|---|
| Serial link | `parseRuta()`, V2 parser, checksum check, ACK builder | `Serial2` started from a task pinned to core 0 with a 4 KB buffer (§11.3) |
| Odometry | `actualizarOdometria()` | Encoder distance (PCNT) + BNO085 yaw → pose of the rear-axle center |
| Route follower | `rutaSteer()`, `rutaServo()` | Pure pursuit with 200 mm lookahead and 0.10 s lag prediction; measured wheel→servo table |
| Speed | `setMotor()`, `setMotorTope()` | Feed-forward + PI on encoder speed, 700 mm/s cap |
| State machine | `INICIO`, `SIGUIENDO`, `ESTACIONANDO`, … | §7.4 |
| Sensors | `initUnToF()`, `leerToF()`, BNO RVC parser | ROI, timing budget and status rules of §6.4 |

**Open Challenge firmware** — [`src/ESP32/RondaAbierta/RondaAbierta.ino`](src/ESP32/RondaAbierta/RondaAbierta.ino), ESP32 only (no Pi needed): §7.6. This is the tested fallback; we are working on running the open round with the route-mode firmware too (§7.6).

## 7.3 Data flow and timing (one Pi cycle)

```mermaid
sequenceDiagram
  participant CAM as Camera (ISP)
  participant PI as Pi runtime
  participant ESP as ESP32
  CAM->>PI: frame 640×480 (~25 ms period)
  PI->>PI: color correction, red/green/magenta masks
  PI->>PI: BEV + corner lines
  ESP-->>PI: ACK: pose px,py,yaw · ToF L/R/F/B · state · turn count · diagnostics
  PI->>PI: localizer: predict with odometry, correct with ToF ray-casts
  PI->>PI: digital map: vote detections into seats (with the pose 5 frames old)
  PI->>PI: route mode: resolve seats → re-plan if needed (background thread)
  PI->>ESP: V2 line: rt=1, map transform rtx/rty/rth, allowed speed rv, flags  *XX
  PI->>ESP: RTB / RTP×n / RTE (only when the route changed)
  ESP->>ESP: every loop: odometry → pure pursuit → servo, speed loop → motor
```

Measured on the Pi 5: **23.5 ms mean per cycle (p95 32 ms)**; a re-plan takes 59–70 ms in a background thread, so a new route never blocks a frame. The camera image is older than the latest ACK, so map votes use the pose from 5 frames earlier (`ROUTE_FIX_LINE_LAG_FRAMES`).

## 7.4 ESP32 firmware and state machine

```mermaid
stateDiagram-v2
  [*] --> WAITING: power on, sensors init
  WAITING --> INICIO: READY from the Pi (button pressed)
  INICIO --> SIGUIENDO: out of the parking lot (S-maneuver done)
  SIGUIENDO --> SIGUIENDO: follow route, count corners
  SIGUIENDO --> ESTACIONANDO: 12 corners done, U-turn, lot alongside (Pi sees magenta) or timeout
  ESTACIONANDO --> TERMINANDO: parked and straightened
  SIGUIENDO --> TERMINANDO: 12 corners (no parking mode)
  TERMINANDO --> [*]
```

- **INICIO — leaving the lot.** The car starts parallel to the outer wall inside the lot. Before arming, the Pi looks at the first pillar ahead and tells the ESP32 which way to leave (`isal`: between the pillar and the outer wall needs only a short S; otherwise a wider S) and whether a pillar is close (`isen=1` → slower exit, 180 instead of 195 PWM). The S-maneuver is driven by encoder distance and yaw.
- **SIGUIENDO — racing.** Pure pursuit on the active route. Each loop: odometry → lookahead point 200 mm ahead along the route → wheel angle δ = atan(2·L·y / ℓ²) (L = 113 mm wheelbase, y = lateral offset of the lookahead point, ℓ = its distance) → measured servo table → servo. The pose is projected **0.10 s ahead** (the measured wheel lag) before choosing the lookahead point, which removed the overshoot at corner exits. A small "wall panic" term (`WALL_PANIC_GAIN`) pushes away from a wall that a side ToF sees too close, as a last-resort safety net. Speed = min(route profile from the Pi, base speed), through the speed loop. Corners are counted against the route geometry, so the stop happens inside the correct section.
- **ESTACIONANDO — parking.** After corner 12 the car continues to the next corner, does a **half turn** and comes back to the lot, so the lot is approached from the side that lets a reverse parallel park start in the open. The Pi confirms the lot is alongside (magenta seen); the ESP32 scans the lot walls with the side and rear ToF and the encoder, solves a reverse **arc–straight–arc** path for the measured lot position (`parkPlanT1Solve()`), drives it, then straightens with short forward/backward passes against the reference heading. An anti-stall rule raises the PWM in steps of 20 if the encoder sees less than 4 mm of progress in 250 ms (parking at low PWM on a fresh vs. tired battery).
- **Fallbacks.** If the Pi never saw the lot, a timeout triggers the parking scan anyway. If the Pi's lines stop arriving, the ESP32 keeps the last *complete* route (double buffer: a half-uploaded route is never used).

**Link protocol (UART 460 800 baud, one line per message):**

```
Pi → ESP   V2,<flags...>,rt=1,rtx=<mm>,rty=<mm>,rth=<deg>,rv=<pwm>*HH     every frame
           RTB,<n>,<step_mm>*HH      start of a new route of n points (closed loop)
           RTP,<i>,x,y,x,y,...*HH    10 points per line, mm on the mat
           RTE,<n>*HH                end: becomes active only if all n points arrived
ESP → Pi   ACK,px=..,py=..,yaw=..,est=..,tc=..,tL=..,tR=..,tF=..,tB=..,rn=..,ue=..,lc=..,ltf=..,...
```

`*HH` is an XOR checksum: the ESP32 drops a corrupted line and counts it (`lc=`); if the ACK shows that the ESP32 does not have the full route (`rn` ≠ n), the Pi re-sends it. The ACK also carries UART error counters (`ue=`, by type), loop time and per-sensor I²C time, which is how we found the electrical problems in §11.3.

## 7.5 Start procedure

1. Power switch on → Pi boots, `wro-runtime` service starts, ESP32 initializes the sensors.
2. The Pi opens the camera, computes the first plan (~2 s) and checks the image (brightness/blackout). Only then the **READY LED** turns on.
3. The single **start button** (Pi GPIO 17) → the Pi sends `READY` ×3; the ESP32 answers and the run starts. No other interaction is needed or allowed.

## 7.6 Open Challenge (ESP32 only)

The Open Challenge has no pillars, so the camera adds risk without adding information. `RondaAbierta.ino` runs on the ESP32 alone:

- **Heading hold with an absolute reference.** Straight *n* is held at exactly *n* × 90° from the start heading (BNO085), so heading errors do not accumulate from corner to corner. The PID runs on every BNO packet (100 Hz). **Kp 1.0, Kd 0**: the v1 value Kd 0.8 caused a saturating oscillation ("dance") after every corner on the new car, and Kp 1.5 was also too high (tested on the track).
- **Wall following as a cascade.** The distance to the wall only shifts the heading target (±15°, rate-limited), with hysteresis, so a bad ToF reading cannot turn the car directly.
- **Corners.** Triggered by the front ToF plus the inner side seeing an opening. The turn ends 5° before 90° and the servo unwinds smoothly over the last 25°.
- **Robust ToF handling** (§6.4): black-wall statuses accepted at their real distance, a jump filter (a reading > 40 cm closer needs a second confirming reading — this removed mid-straight false corners), and a stale-sensor watchdog.

Result: **12/12 corners in 29.6–29.9 s** in both directions on our track (runs 133246 and 141103, 2026-10-19). At the national final v1 needed 57.5 s.

**Next step — open round with route mode.** `RondaAbierta.ino` is reactive: it decides each corner from what the ToF sees at that moment, so it has to drive the whole round at a speed that is safe for a corner it has not seen yet. In the Open Challenge the inner walls can be placed in different positions, but they do not move during the round. We are adapting the obstacle-round firmware (`PurePursuit.ino`, §8) so that lap 1 maps the walls with the ToF and the encoder (the position of each straight's inner wall and where each corner starts), and laps 2–3 follow a planned route with pure pursuit and the speed profile, knowing in advance where to turn and how fast each section can be driven. One code base for both rounds also means one set of parameters to calibrate at the venue. Until this beats 29.6 s reliably on our track, `RondaAbierta.ino` stays as the open-round firmware.

---

# 8. Obstacle Management — route mode

## 8.1 Idea

The obstacle field is not random in an arbitrary way: by the rules, each straight has **6 possible pillar seats** (two rows × three columns), and the parking lot lies in the start straight. So the field can be described by a small table: *which seat of which straight holds which color*. Once that table is known, a full lap is a fixed curve on the mat that can be planned in advance and followed with odometry. The camera's job reduces to **filling the table** and correcting the pose.

```mermaid
flowchart LR
  A["Camera detections<br/>(red / green, BEV mm)"] --> B["Digital map<br/>votes per seat"]
  C["Rulebook constraints<br/>(seats, max per straight)"] --> B
  B --> D{"Seat state"}
  D -->|confirmed color| E["Planner: pass on the<br/>correct side"]
  D -->|unknown| F["Planner: treat as an<br/>obstacle with no side"]
  D -->|seen empty| G["Planner: free"]
  E & F & G --> H["DP route + B-spline"]
  H --> I["Speed profile"]
  I --> J["Upload to ESP32"]
  K["Localizer EKF"] --> B
  K --> J
```

## 8.2 Mapping pillars into seats

Each pillar detection is projected to the floor (BEV, mm), moved to the mat frame with the localized pose, and **votes** for the nearest seat. A seat is confirmed when the votes agree and a detection has landed within 160 mm of it. Edge cases we hit and handled:

| Edge case | Handling | Evidence |
|---|---|---|
| A pillar seen **while the car is still turning** out of a corner lands ~400 mm off (heading lag × off-axis angle) | Votes > 20° off-axis taken while the car is > 25° from the straight's heading still count, but cannot confirm a seat or set its distance | ERR-30: real runs 48/50/52 replayed; run 52 now maps E/X2 correctly |
| The image is older than the pose | Votes use the pose from 5 frames earlier | Twin: the lot-position estimate from the orange line had the wrong length by ≈ 0.22 s × speed before this |
| Pink parking walls and orange tape look red | Ratio tests (G/R, B/R) per §6.4 | Pi 4 replays of 6 runs; Pi 5 thresholds measured on our track |
| Seat never visible (behind the camera at the start) | Rulebook constraints resolve it: a lone pillar (X) has no partner, never two in the same column, max two per straight, forbidden seats next to the lot | Twin lap-1 tests |

## 8.3 Lap 1 — planning with an incomplete map

In lap 1 most seats are unknown. A seat is **resolved** when the map confirms it, when the rules exclude it, or when it was within 550 mm and ±45° of the camera for 16 frames with nothing detected ("seen empty"; our measured detection rate in that zone is ~98 %). Until then:
- An unresolved seat is an obstacle **with no pass side**: the route never goes where a pillar might be, but passes close to it so that, if it turns out to need the other side, the change is only ~300 mm.
- With ≥ 2 votes of one color, the route already takes that color's side before confirmation (**provisional color**).
- Speed is lower near unresolved seats and the car only stops if it is looking at an unresolved seat (max 2 s).
- Each re-plan keeps the next 2 route stations in front of the car unchanged, so the steering never jumps.

From lap 2 the map is complete and the route is the same every lap, at full speed.

## 8.4 Route planner — dynamic programming

[`ring_route.py`](src/RASPI/cam/pure_pursuit/ring_route.py) describes the lap by **stations** every ~75 mm along the corridor, and at each station a lateral position *d* = distance to the island (100–900 mm, step 20 mm). In corners the stations become concentric arcs around the island's corner. Dynamic programming finds the optimal path through this grid:

- **Hard constraints:** clearance of the whole car footprint (rear axle +147.6 / −21.8 mm along, ±66 mm across) ≥ 70 mm (`ROUTE_HARD_CLEAR_MM`) to pillars, walls, island and parking walls; radius ≥ 170 mm; each segment ≤ 55° from the corridor direction; pass side by color (red right, green left).
- **Cost:** length + curvature² + a penalty when clearance < 140 mm; with `ROUTE_TIME_COST`, the cost is the *time* under the speed profile instead of the length.
- **Smoothing:** closed cubic B-spline, resampled every 80 mm for the upload.

*Why DP:* the lap is a closed corridor with a natural order (stations), so DP over (station, lateral position) is exact on its grid, cannot get stuck in a local minimum between two close pillars, and its run time is fixed and predictable (59–70 ms on the Pi 5), which matters for a planner that re-runs during the race. Before it we used a line built from the map with Bézier knees (`_build_line`) and an A* racing line ([`racing_line_astar.py`](src/RASPI/cam/pure_pursuit/racing_line_astar.py)); the `_build_line` had steps of up to 180 mm and 90° kinks, and with it the twin finished **1 of 21** races against **69 of 69** with the DP route (known map, same seeds).

*Why a 20 mm lateral grid:* a 10 mm grid removed a small bump in the corners and gained 0.3 s median, but each re-plan became ~5× more expensive; with the real Pi latency modeled it dropped from 40/48 to 29/48 clean races, so we kept 20 mm.

## 8.5 Localization — EKF

The localizer keeps (x, y, heading) of the rear axle on the mat. **Prediction:** odometry increments from the ACK (encoder + BNO085). **Correction:** each ToF reading against a ray-cast of the known map. A reading is used **only if the expected ray hits a wall or the island**; if it would hit a pillar or a parking wall it is ignored, because those positions are less certain. Process noise grows with distance driven (2 mm² per mm). A reading that jumps > 100 mm from the previous one is ignored once (echoes from a pillar to the wall behind it). Each ToF has a maximum range for correction (laterals 450–480 mm) from the static tests in §6.4.

## 8.6 Speed profile

The route's speed is computed from its curvature: v ≤ √(a_lat · R) with a_lat ≈ 2.77 m/s² (just above the measured 2.5–2.8 m/s² grip, tuned in the twin), braking ≤ 3.0 m/s², lower speed near pillars (< 193 mm) and near unseen seats, and a 0.12 s latency compensation. The ESP32 receives it as the allowed PWM (`rv`), which the speed loop turns into a target speed.

## 8.7 Blind start — the parking lot can be anywhere along the straight

At the national final the judges moved the parking lot along the start straight. The car knows the straight, its heading and its distance to the outer wall, **not** how far along it is. [`start_fix.py`](src/RASPI/cam/pure_pursuit/start_fix.py):

1. Assume the lot is as far back as possible (the car then believes it is behind its real position, so it turns late, never into the island).
2. Build a histogram over the unknown shift δ (5 mm bins) from every ToF reading that pillars cannot explain, weighted by the distance driven (so a stopped car does not vote 9 times).
3. When the orange corner line is seen, project it to the mat and use it as the strongest vote (`ROUTE_FIX_LINE`).
4. Fix when 90 % of the probability is within ±20 mm and agrees with the line within 60 mm; at the end of the straight force the best estimate.
5. Re-vote the camera detections stored before the fix with the corrected pose, re-plan, and switch the ESP32 to the new frame only when the full new route has arrived (no jump mid-dodge).

Twin result with the lot at a random position: **41/42 clean, median lot error 13 mm** (the old code that assumed a fixed lot: 7/42).

## 8.8 2026 extra task — the surprise pillar

**The rule** ([WRO 2026 Future Engineers Extra Task](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Extra-Task.pdf), published 2026-11-09). The attempt is optional and declared before the round. One pillar, never the first one on the route, is replaced by a pillar of the **opposite color** that is **50 mm or 150 mm tall** instead of 100 mm (same 50 × 50 mm base). The car must pass it on the side of its **original** color, i.e. the opposite of what it sees. Each correct pass earns 5 points, up to 15; a pass on the wrong side ends the attempt.

**Idea.** Color alone cannot solve this: the special pillar is a perfectly normal red or green pillar. The only thing that gives it away is its **height**, so the Pi measures each pillar's height in the image and swaps the driving color when the height is not 100 mm. Everything after that is unchanged: the digital map, the planner (§8.4) and the ESP32 receive the *driving* color, so the route, the protocol and the firmware stay the same. The detection keeps both colors (seen and driving) in the log.

**How the height is measured** ([`surprise.py`](src/RASPI/cam/pure_pursuit/surprise.py)):

| Option | Why we did not use it / why we did |
|---|---|
| Height/width ratio of the blob | Rejected. In 735 rendered views (3 heights × 7 distances × 5 car angles × 7 viewing directions), the ratio of the three classes **overlapped**: the square base looks wider when seen from a corner, and perspective changes the top. |
| Height in pixels alone | Rejected. A 50 mm pillar close to the camera looks taller than a 150 mm one far away. |
| **Pixel height compared with a normal 100 mm pillar at the same floor distance** | **Used.** The pillar's foot sits on the floor, so the bird's-eye-view homography (§6.4) gives its distance; the expected pixel height of a *normal* pillar at that distance comes from a calibration table (photos of one 100 mm pillar at several distances, [`calibrar_sorpresa.py`](src/RASPI/cam/pure_pursuit/calibrar_sorpresa.py)) or, before calibrating, from the camera geometry. In the same 735 views the measured/expected ratio separated cleanly: **0.50–0.61** (50 mm), **0.985–1.05** (100 mm), **1.51–1.54** (150 mm). |

Only the pillar's **foot** goes through the floor homography; the height is measured in the camera image, so the body of the pillar is never warped as if it lay on the floor.

**Edge cases.**
- **Confirmation.** A special pillar needs **3 consistent frames** and visible floor under its base (so the top of a pillar seen over a wall is not taken as a short pillar). A well-measured normal pillar is accepted at once, so the car does not react later than in a normal round.
- **Cropped pillars.** A pillar cut by the image edge is never re-classified from the partial view; the car keeps the class already confirmed, and the map keeps its position.
- **Memory.** The class is stored per seat (§8.2), so laps 2 and 3 reuse it. GO clears it.
- **No extrapolation.** Outside the distances covered by the calibration table no new special class is created.
- **Small pillar.** The 50 mm pillar has a smaller blob than the normal minimum area, so a smaller area is accepted only if it is consistent with 50 mm.

**Switching it on.** Off by default (`SURPRISE_RULE = False`), so a normal round runs exactly the normal code. For a declared attempt: `runtime_nuevo --surprise-rule`, or `"SURPRISE_RULE": true` in `carrera.json`. The runtime refuses to start the attempt without a calibrated camera.

**Results in the digital twin** (2026-11-10; the twin renders the special pillar at its real height and a judge scores the passes on the original color):

| Test | Result |
|---|---|
| Normal round with the feature present but off (24 fields × 2 noise seeds) | **48/48 identical** to the reference |
| Surprise pillar, 24 fields × 2 heights | **35/48 scored the full 15 points**, 1/48 scored 10 |
| Passes on the wrong side of the special pillar | **0** |
| Full race with parking, surprise mode on | 6/48 (normal-mode controls on the same fields: 4/24) |
| Unit tests | 390 passed (both colors and directions, three heights, distance, cropping, memory across laps and GO, judge) |

**Open before declaring the attempt.**
1. **Calibrate with real photos** of a normal pillar at several distances on the car (no real table yet), then check with real 50 mm and 150 mm pillars.
2. **Three regressions in the twin:** on two fields the car hits the parking walls only when surprise mode is on, and on one field (150 mm) it hits a normal pillar after 10 corners. In all three the special pillar itself was classified correctly; the cause is in the route or the parking, not in the classifier.
3. Decide per round, with these results, whether the 15 points are worth the risk to the main score.

Design notes and the full twin study: [`docs/notes/SURPRISE_RULE_2026.md`](docs/notes/SURPRISE_RULE_2026.md) (Spanish).

---
# 9. Digital Twin — software-in-the-loop simulator

## 9.1 Why we built it

With v1, every idea had to be tested on the real track: one run, one video, one log. Between the national and the international final we had ~11 weeks, a car that did not exist yet for the first month (new PCB, new chassis) and a new strategy to develop. So we built a **digital twin** that runs **the real code**, not a re-implementation of it:

| Part | In the twin | Real or modeled |
|---|---|---|
| ESP32 firmware | `PurePursuit.ino` compiled for the PC (Arduino API mocks, `zig c++`) into a shared library | **Real code** |
| Raspberry Pi software | `runtime_nuevo.py` and every module of §7.2, unmodified | **Real code** |
| UART link | Bytes and time at 460 800 baud, ESP32 RX buffer and the 128-byte hardware FIFO (lines that do not fit are lost) | Modeled from our measurements |
| Camera | Rendered from the **measured** camera pose (tilt 24.8°, 95 mm, 152 mm), flat colors, with camera vibration | Modeled |
| Vehicle | Bicycle model with the **measured** servo table, ±1.5 servo backlash, 0.10 s wheel lag, motor speed per PWM, braking/coasting and grip limit from §5.2 | Modeled from measurements |
| Sensors | ToF with per-sensor valid-status probabilities and the front Long-mode bias, BNO085 noise, encoder counts | Modeled from measurements |
| Field | Random pillar layouts following the rulebook (checked against the official randomizer), random parking-lot position, both directions | From the rules |

Code: [`src/RASPI/cam/pure_pursuit/twin/`](src/RASPI/cam/pure_pursuit/twin/) — entry point `python -m pure_pursuit.chassis_twin`, plan and history in [`docs/notes/twin_plan.md`](docs/notes/twin_plan.md) and [`docs/notes/HANDOFF.md`](docs/notes/HANDOFF.md).

**[TODO: add a screenshot or GIF of the twin viewer (camera panel + map panel with the planned route) to `docs/img/twin.png`.]**

## 9.2 How we use it — the benchmark rule

A change is accepted only if it wins on the **standard bench**: the same 42 random fields × 5 noise seeds for the current version (base) and the candidate, compared **paired** (same field, same noise). The rule:

1. No new failure. A failure is any collision, a pillar passed on the wrong side, any touch of the outer wall, or a park outside the lot.
2. Then, lower median and mean time.
3. A second bench charges the real Pi 5 CPU time (frame ×4.28, re-plan ×5.8 vs. the PC) inside the simulated time, so changes that only work with a fast PC are rejected.

Tools: [`twin/tools/banco/`](src/RASPI/cam/pure_pursuit/twin/tools/banco/) (bench, paired summary, an evolution-strategy optimizer that tunes 8–10 parameters at a time). Every accepted configuration is exported to the car with [`exportar_carrera.py`](src/RASPI/cam/pure_pursuit/twin/tools/exportar_carrera.py), which writes `carrera.json` (Pi) and regenerates `PurePursuitCarrera.ino` (ESP32), so the car and the twin never drift apart.

## 9.3 What the twin gave us — and where it was wrong

| Result | Number |
|---|---|
| Old reactive control vs. route mode (known map) | 1/21 → **69/69** clean races |
| Lap 1 with an unknown map | **21/21** clean |
| Parking lot at a random position, blind start | **41/42** clean, median lot error 13 mm (fixed-lot code: 7/42) |
| Full race (start, 3 laps, U-turn, parallel park), final tuned configuration | **126/126** without wall contact; best 32.9 s, median 37.4 s |
| Same, charging the real Pi 5 latency | 40/48 (18/48 before the latency fixes: background planning, warm caches) |

The twin is a model, and the real car found what the model did not have. **Every hardware bug found on the car became a model in the twin**, so it cannot hide again:

| Found on the real car | Missing in the twin | Now modeled |
|---|---|---|
| Route never accepted (`rn=0`): UART FIFO overflow | Unlimited serial buffers | RX ring + 128-byte FIFO |
| Front ToF invalid 57–68 % of the time beyond 30 cm, +20 cm long | Perfect ToF | Per-sensor status probabilities, Long-mode bias |
| Pi loop at 22 fps instead of 40 | Assumed 40 fps | Measured frame period (then fixed on the car, §6.4) |
| Lateral ToF reading the floor | Perfect ToF | Range curve per sensor |
| Camera pose (tilt 24.8° vs the 45° first assumed) | Assumed camera | Measured pose from the BEV calibration |

---

# 10. Testing and Validation

## 10.1 Testing workflow

```mermaid
flowchart LR
  A["Idea / bug"] --> B["Twin: reproduce<br/>(seed + noise)"]
  B --> C["Change code"]
  C --> D["Twin bench<br/>42 × 5 paired"]
  D -->|new failure| C
  D -->|accepted| E["Export race config<br/>commit"]
  E --> F["Bench test on the car<br/>(sketch / no motor)"]
  F --> G["Track runs<br/>video + BEV + journal"]
  G --> H["Run report<br/>reporte_run.py"]
  H -->|car ≠ twin| I["Model the difference<br/>in the twin"]
  I --> B
  H -->|pass| J["Validation matrix<br/>+ release"]
```

1. **Component tests** with small sketches before any full run: [`EncoderTest`](src/ESP32/EncoderTest/), [`TofTest4`](src/ESP32/TofTest4/), [`BnoRvcTest`](src/ESP32/BnoRvcTest/), [`TofBnoTest`](src/ESP32/TofBnoTest/), [`CalibFox`](src/ESP32/CalibFox/) (servo radius, encoder, ToF), [`PinScan`](src/ESP32/PinScan/) (UART wiring).
2. **Motion bench without the Pi:** [`SeguirRuta`](src/ESP32/SeguirRuta/) + [`scripts/ruta_bt.py`](scripts/ruta_bt.py): the PC sends a route over Bluetooth and the ESP32 follows it with odometry ([results](docs/route-following-bench.md)).
3. **Track runs:** every run records the camera/HUD video (`orillasNN.avi`), the clean BEV (`_bev.bin`) and the Pi journal with every message to and from the ESP32. Recording stops 5 s after the car stops. [`scripts/reporte_run.py`](scripts/reporte_run.py) turns the journal into a per-corner report.
4. **Acceptance:** a run passes only if it completes autonomously with no contact, every pillar on the correct side, and (obstacle round) a park inside the lot. Failed runs are kept: each one names the next thing to change. The full method is in [`docs/testing.md`](docs/testing.md).

**Metrics we track per run:** corners completed, collisions/contacts, pillars on the wrong side, lap and total time, parking-lot position error, final park position, Pi frame rate, ESP32 loop time, UART errors (`ue`, `lc`), route points accepted (`rn`).

## 10.2 Component and bench results

| Test | Result | Date |
|---|---|---|
| Encoder: push 1 m along a tape (PCNT ×4, 5.45 counts/mm) | 1008–1043 mm | 2026-11-09 |
| ToF static, ROI 16 vs ROI 8, black wall 60 / 80 cm | −50 / −190 mm → **−8 / −40 mm** | 2026-10-21 |
| UART Pi ↔ ESP32 at 460 800, bench | 583/583 ACK, 0 errors, route accepted | 2026-11-07 |
| Turning radius on the mat (servo 1 / 156) | 123 / 134 mm | 2026-10-22 |
| Route following, ESP32 only, open lap ×3 at PWM 255 | 28.7–29.1 s, mean deviation **12 mm** with wall correction | 2026-10-22 |
| Route following, ESP32 only, obstacle layout ×3 | 26.2–26.6 s, lateral error rms **7 mm**, all pillars correct | 2026-10-23 |
| Pi 5 pipeline | 23.5 ms/frame mean, re-plan 59–70 ms | 2026-11-06 |

## 10.3 Open Challenge (v2)

| Run | Direction | Corners | Time | Interventions | Firmware | Status |
|---|---|---|---|---|---|---|
| 133246 | CW (right) | 12/12 | 29.9 s | 0 | `RondaAbierta.ino` | Validated |
| 141103 | CCW (left) | 12/12 | 29.6 s | 0 | `RondaAbierta.ino` | Validated |
| **[TODO]** | CW / CCW × 3 each, different inner-wall positions | | | | release commit | To run before the deadline |

## 10.4 Obstacle Challenge (v2, real car, route mode)

| Date | Runs | What happened | Cause → fix | Status |
|---|---|---|---|---|
| 11-07 | first test | Car did not move | Pi→ESP UART not arriving; then FIFO overflow (`rn=0`) → `Serial2` on core 0, 4 KB buffer (`1360188`) | Fixed |
| 11-09 | track tests | Car turned before the walls | BNO085 yaw sign lost when merging branches → pose mirrored; ISR encoder losing counts → PCNT | Fixed |
| 11-11 | 35–36 | Completed with weaving (8 % more distance than the twin) | Pi at 22 fps (CPU scaling) → Picamera2 ISP, 40 fps (`66ca91aa`) | Fixed |
| 11-11 | 37–38 | Lost V2 lines while the motor ran (`ue`≈400/s) | Bad UART cable → replaced, `ue`=0 | Fixed |
| 11-11 | **41** | **Clean race, 12 corners**, speed loop on | — | Pass |
| 11-11 | **48** | **Clean race, 12 corners**; parking sequence started but the car did not end fully inside the lot | Parking still being tuned (see §10.5) | Race: pass · Park: open |
| 11-11 | 50, 52 | Lone green (E/X2) mapped at the start of its straight | ERR-30: off-axis votes while turning (`a22bb3e0`) | Fixed offline |
| 11-11 | 51 | Hit 2nd green after the lot | ERR-31: lot fix without orange-line votes | Open |
| 11-12 | 57, 59, 60 | Hit the island after the lot | Auto-exposure burn + line votes rejected while still turning → fixed exposure (`c6757575`), turn limit 3°→10° (`1000d3cf`) | Fixed |
| 11-12 | 62, 64 | Completed; lot error +196/+201 mm vs ~+230 real; near-miss at E/X in lap 1 | E/X1 ↔ E/X2 flip in lap 1 | Open |

**Formal validation matrix for the release** (to be filled with the final firmware; each row = one full round with a random layout and a random lot position):

| Run | Direction | Lot position | Score | Time | Collisions | Wrong side | Park | Battery V | Commit | Status |
|---|---|---|---:|---:|---:|---:|---|---:|---|---|
| OBS-01 | CW | **[TODO]** | | | | | | | | |
| OBS-02 | CW | | | | | | | | | |
| OBS-03 | CW | | | | | | | | | |
| OBS-04 | CCW | | | | | | | | | |
| OBS-05 | CCW | | | | | | | | | |
| OBS-06 | CCW | | | | | | | | | |

## 10.5 Known limitations

- **Lap-1 mapping of the lone pillar right after a corner** (E/X1 vs E/X2) can flip until the car is straight; planned fix: keep both seats as obstacles until confirmed head-on.
- **Lot position without orange-line votes** falls back to ToF only, which can be ~0.3 m off (ERR-31).
- **Parking is not finished yet.** The car leaves the lot, completes the race and runs the parking sequence, but it has not yet ended **fully inside** the lot on the v2 car. We are tuning the arc–straight–arc maneuver (§7.4) to make it both complete and as fast as possible.
- **Consistency of the full round.** Clean 3-lap rounds with a correct map exist (runs 41, 48), but not every run yet completes all 3 laps with every pillar mapped correctly; the open mapping cases are the two above. The validation matrix above will be filled once this is consistent.
- **Front ToF** is reliable only to ~80 cm on the black wall; it is never used for the lot fix.
- **Surprise pillar (§8.8)** is validated only in the twin: it still needs a calibration table from real photos and three twin regressions (parking and one pillar) fixed before we declare the attempt.
- **Pi 5 temperature** without a fan (~80 °C) — cooling pending (§6.5).
- **Twin parking** is the least realistic part of the simulator; parking constants are confirmed on the car.

---

# 11. Systems Thinking and Engineering Decisions

## 11.1 How the subsystems constrain each other

| If we change… | …it affects | Example from this season |
|---|---|---|
| Gear ratio (mobility) | Top speed → speed profile → planner time cost → how close to pillars the route can pass at speed | 2 : 1 → 1.25 : 1 made the car grip-limited in corners, so the speed profile (a_lat) became the key tuning knob |
| Camera frame rate (compute) | Every "N frames" constant, votes per pillar, latency of the pose used for mapping | Moving from 14 to 40 fps required re-scaling frame counts into seconds; at 22 fps (bad pipeline) the real car weaved |
| ToF field of view (sensors) | Localizer accuracy → map position of pillars → route clearance | ROI 16 read the floor; the pose was pulled 100–250 mm toward one wall and the car grazed a pillar |
| PCB layout (power) | Pi supply → throttling → frame rate → mapping | The second regulator isolates the Pi from servo spikes |
| Firmware/UART (software) | Whether the route arrives at all | One FIFO overflow = route never accepted = car follows nothing |
| Chassis length (mechanics) | Turning radius, parking-lot length (1.5 × car), clearance the planner must keep | 240 → 176 mm and 208 → 123–134 mm radius |

## 11.2 Trade-offs — "we chose X instead of Y because…"

| Decision | Instead of | Because (data) |
|---|---|---|
| Plan a full **route** and follow it with odometry | Reactive steering from the image (v1) | Twin: 1/21 → 69/69 clean races; real car: clean 12-corner races (41, 48) without stopping at corners |
| Route follower on the **ESP32** | Steering computed on the Pi | The pose reached the Pi 4–6 frames late; on the ESP32 the follower runs at loop rate on current odometry |
| **ToF as the distance sensor**, HC-SR04 only as a cross-check | Ultrasonics as the main sensor (v1), or ToF alone | Ultrasonic phantoms in ~20 % of straight frames (ERR-19) rule them out as the main sensor; with ROI 8 and status rules the ToF read the black wall within −8/−40 mm at 60/80 cm. But the ToF's weak spot (dark walls, far away) is exactly where sound works, so we keep the ultrasonics to check it instead of trusting one physics |
| **BNO085** fused yaw over UART-RVC | MPU-6050 integration; BNO over I²C | MPU heading 6.5 % low, −7.5° per corner; RVC keeps the IMU off the busy ToF I²C bus and needs no library |
| **Encoder read by PCNT hardware** | Interrupts | Interrupts lost 25–65 % of counts under motor load on the floor; PCNT: 1 m push → 1008–1043 mm |
| **1.25 : 1** gearing | 2 : 1 (v1) or 1.5 : 1 | Corners are grip-limited (2.5–2.8 m/s²); speed is only useful on straights: 514 → ~800 mm/s |
| **Speed loop capped at 700 mm/s** | Full 800 mm/s | Braking from full speed locks the wheels and slides 24–51 mm; the cap keeps odometry trustworthy |
| **Dynamic programming** planner on a 20 mm grid | Bézier line from the map; A*; 10 mm grid | `_build_line` kinks (1/21); 10 mm grid 5× slower re-plan, 29/48 vs 40/48 with real latency |
| **Fixed camera exposure** per venue | Auto exposure | Auto exposure burned the floor for ~1 s after leaving the magenta lot → orange line missed → crash (run 57) |
| **Picamera2 with ISP scaling** | GStreamer scaling on the CPU | 245 % CPU and 22 fps → 5 % CPU and 40 fps, same image |
| **Raspberry Pi 5** | Optimize the Pi 4 | Capture was only ~5 ms of a 65 ms frame; vision itself was the cost; Pi 4 throttled at 84 °C |
| **Two stacked PCBs** | One bigger board | The 176 mm chassis has no room for one board; power and motor noise stay on the lower board |
| **Open round on the ESP32 alone** | Use the camera too | No pillars to see; heading hold + ToF gave 12/12 in ~30 s, with fewer failure points |
| **Simulator running the real code** | Testing only on the track | Hundreds of full races per change; every accepted change passed 210+ paired races before reaching the car |

## 11.3 Risks, failure modes and mitigations

| Failure mode | How we found it | Impact | Mitigation | Status |
|---|---|---|---|---|
| Logic board stacked rotated 180° | Pi 5 died (2026-11-06) | Lost a Pi 5 (replaced) | Orientation marks; measure the Pi rail **without the Pi** first; next rev: keyed headers + PTC + TVS | Mitigated |
| Pi does not boot on battery | 1000 µF on the Pi's 5 V output (design: 100 µF) tripped the MINI560 start-up protection | No start | Back to 100 µF; add 470–1000 µF ≥ 25 V at the regulator input next rev | Fixed |
| UART FIFO overflow | `rn=0`, `UART_FIFO_OVF` in the log | Route never accepted | `Serial2` on core 0, 4 KB buffer; checksum + re-send; error counters in every ACK | Fixed |
| Bad UART cable | `ue`≈400/s only with the motor on | Lost commands | Cable replaced; `ue` watched in every run | Fixed |
| Encoder counts lost | 1 m push read 326–411 mm | Pose wrong | PCNT hardware counter | Fixed |
| Yaw sign wrong after a merge | Car turned before the walls | Mirrored pose | Sign checked in the bench checklist; constants audited after merges | Fixed |
| ToF reads the floor / black wall | Static tests, lateral pose bias | Pose bias 100–250 mm | ROI 8, status rules, per-sensor max range in the EKF | Fixed |
| Camera exposure burn after the lot | Video of runs 57/59/60 | Late orange line → crash | Fixed exposure per venue | Fixed |
| Pi overheating | ~80 °C, throttle flags | Lower frame rate | Fan / Active Cooler | **Open** |
| Venue lighting changes colors | National final: pink read as red | Wrong pillar color | Ratio tests + `calibra_luz.py` at the venue | Mitigated |
| USB powering the car without the battery | Motor weak, 636 vs 2000 counts/s | Misleading bench results | Bench tests only with the battery connected | Procedure |

## 11.4 Iteration timeline

| Date | Milestone |
|---|---|
| 2026-09-19 | **1st place, Mexico national final** with v1 ([`v1.0-nacional`](docs/releases.md)) |
| 2026-09-21 → 10-04 | Profiling, component decisions, components ordered; v2 chassis designed; digital twin started |
| 2026-10-06 | Pi 5: same pipeline at 40 fps on recorded video |
| 2026-10-13 | Route mode in the twin: 69/69 known map, 21/21 lap 1 |
| 2026-10-14 → 10-19 | New car runs: open round 12/12 in ~30 s; encoder and ToF characterized |
| 2026-10-17 | PCB RevB ordered |
| 2026-10-22 → 10-24 | Route-following bench at full speed; car dynamics measured |
| 2026-11-04 → 11-07 | PCB assembled; first full-system tests; UART and power problems found and fixed |
| 2026-11-11 | First clean route-mode races on the real car (runs 41, 48) |
| 2026-11-12 | Fixed exposure, lot-fix robustness |
| 2026-11-16 | Documentation for the international final; v2 CAD published (parts, STEP, STL, renders); HC-SR04 cross-check of the ToF added to the firmware (not yet on the car) |
| next | Parking fully inside the lot; consistent 3-lap rounds; open round with route mode; validation matrices (§10.3–§10.4); tag `v2.0-internacional` |

The dated, phase-by-phase story is in the [Engineering Journal](ENGINEERING_JOURNAL.md); every decision (DEC-xx) and incident (ERR-xx) with its evidence is in the [engineering log](docs/engineering-log.md).

---
# 12. How to Build and Run

## 12.1 Build the car

1. **Print the parts** — STL in [`models/v2/stl/`](models/v2/stl/) (parts list and quantities in [`models/v2/README.md`](models/v2/README.md)); gear profiles in [`Mecanica/Engranes/`](Mecanica/Engranes/). PLA, 0.2 mm layers; gears and rack at 100 % infill.
2. **Order the PCBs** — Gerbers from the KiCad projects [`electrical/WRO_RevB_1/`](electrical/WRO_RevB_1/) (logic) and [`electrical/WRO_RevB_2/`](electrical/WRO_RevB_2/) (power): KiCad → Plot → Gerber + drill. Schematics in PDF: [`electrical/export/`](electrical/export/).
3. **Assemble** following [§5.5](#55-assembly-order-v2). **Before mounting the Pi:** with the boards stacked in the marked orientation and the battery connected, measure 5.0–5.2 V on `TO_PI` pins 7/8, 5 V on the servo header and 3.3 V on the sensor header.
4. **Components** — see the [cost report](#15-cost-report) for the full list.

## 12.2 ESP32 firmware

Arduino IDE 2 or `arduino-cli`, board **ESP32 Dev Module** (`esp32:esp32:esp32`, ESP32 Arduino core 3.x — needed for `driver/pulse_cnt.h`). Library: **VL53L1X by Pololu**. Upload with the car battery **off** (the motor start can drop the USB connection).

```bash
arduino-cli core install esp32:esp32
arduino-cli lib install "VL53L1X" "MPU6050_tockn"     # MPU6050_tockn: legacy include, not used on v2

# Obstacle Challenge (race build, generated from PurePursuit.ino + the race parameters)
arduino-cli compile --fqbn esp32:esp32:esp32 src/ESP32/PurePursuitCarrera
arduino-cli upload  --fqbn esp32:esp32:esp32 -p <PORT> src/ESP32/PurePursuitCarrera

# Open Challenge (ESP32 only)
arduino-cli compile --fqbn esp32:esp32:esp32 src/ESP32/RondaAbierta
arduino-cli upload  --fqbn esp32:esp32:esp32 -p <PORT> src/ESP32/RondaAbierta
```

To regenerate the race build after changing `PurePursuit.ino` or the tuned parameters:

```bash
cd src/RASPI/cam
PYTHONPATH=. python pure_pursuit/twin/tools/exportar_carrera.py ruta_perfil pure_pursuit/twin/tools/carrera_bench.json --pi-baud 460800
# writes pure_pursuit/carrera.json + src/ESP32/PurePursuitCarrera/PurePursuitCarrera.ino
```

## 12.3 Raspberry Pi 5 software

Raspberry Pi OS (Debian 13 "trixie", 64-bit).

```bash
git clone https://github.com/Nakashima26/WRO_FE_2026_FoxRobotics.git FoxRobotics
cd FoxRobotics
sudo apt install python3-opencv python3-numpy python3-serial python3-picamera2 \
                 python3-rpi-lgpio gstreamer1.0-libcamera gstreamer1.0-plugins-good
python3 -m venv --system-site-packages .venv && source .venv/bin/activate
pip install -r requirements.txt          # skips the apt packages on the Pi
```

`/boot/firmware/config.txt`: `enable_uart=1`, `camera_auto_detect=0`, `dtoverlay=imx219,cam0` (the port the ribbon is on). The ESP32 is on `/dev/ttyAMA0`. The camera tuning file (lens shading) is loaded from [`src/RASPI/cam/camera_tuning/`](src/RASPI/cam/camera_tuning/README.md).

**Autostart:** `sudo ./scripts/install_autostart_pi.sh` installs [`deploy/systemd/wro-runtime.service`](deploy/systemd/wro-runtime.service) plus the Pi 5 drop-in in `deploy/systemd/wro-runtime.service.d/`. Logs: `journalctl -u wro-runtime.service -b 0`. To restart the service use `stop`, wait 4 s, `start` (a direct `restart` can leave the camera busy). Always shut down with `sudo poweroff`: cutting power corrupted libraries on the SD card once.

## 12.4 Calibration at a new venue (≈ 20 minutes)

All from `src/RASPI/cam/`. Full guide: [`pure_pursuit/INSTRUCCIONES.md`](src/RASPI/cam/pure_pursuit/INSTRUCCIONES.md) and [`docs/notes/CALIBRACION.md`](docs/notes/CALIBRACION.md).

| Step | Command | What it sets |
|---|---|---|
| 1. Exposure | `python -m pure_pursuit.mide_exposicion --guardar` | Fixed exposure for the venue light → `camera_tuning/exposicion.json` |
| 2. Colors | `python -m pure_pursuit.calibra_luz` (or the browser view `python -m pure_pursuit.cam_web`) | Floor reference, red/green/magenta/orange ratios |
| 3. BEV (only if the camera mount moved) | `python -m pure_pursuit.calibrate` | Homography from floor crosses → `bev_calib.npz` |
| 4. Servo center | Drive straight at PWM 150 from both sides | `centroServo` (75 on our car) |
| 5. Sensor check | First ACKs after the button: all four ToF valid, `ue=0`, `rn` = route length | Wiring and UART |

## 12.5 Run

Power on → wait for the READY LED → press the button. Manual run with a debug window:

```bash
cd src/RASPI/cam
python -m pure_pursuit.runtime_nuevo --serial-port /dev/ttyAMA0 --record-orillas    # with debug window
python -m pure_pursuit.runtime_nuevo --serial-port /dev/ttyAMA0 --no-window --record-orillas --record-output ~/FoxRobotics/videos_orillas   # as the service runs it
```

`carrera.json` (the race configuration exported from the twin) is loaded automatically on top of `config.py`; `--sin-carrera` disables it. Each run is recorded to `videos_orillas/` (video + clean BEV + journal). Summary: `python scripts/reporte_run.py --pi`.

## 12.6 Run the digital twin (PC)

```bash
# once (Python 3.11+); on macOS/Linux use .venv-sim/bin/ instead of .venv-sim/Scripts/
python -m venv .venv-sim
.venv-sim/Scripts/pip install -r src/RASPI/cam/requirements-sim.txt ziglang

cd src/RASPI/cam
export PYTHONUTF8=1                       # Windows consoles: the logs print non-ASCII characters
export TWIN_PI_UART_US_PER_BYTE=21.7      # 460 800 baud, as on the car (the default models 115 200)
export FOX_CXX="python -m ziglang c++"    # Windows: use zig even if another c++ (e.g. MinGW) is on the PATH

# watch one race in the viewer (camera, BEV and map with the planned route)
python -m pure_pursuit.chassis_twin --preset ruta_perfil --park-pos random --seed 1

# headless: print the metrics only, or save the viewer as a GIF
python -m pure_pursuit.chassis_twin --preset ruta_perfil --park-pos random --seed 1 --no-show
python -m pure_pursuit.chassis_twin --preset ruta_perfil --park-pos random --seed 1 --save-gif race.gif
```

The first build of the firmware library takes ~3 minutes; it is cached per firmware hash. Each run ends with a `[TWIN] stop=… metrics={…}` line (finished, collisions, lap times, park result). The race configuration tuned on the bench passes extra `--pi KEY=VALUE` overrides; the full command lines of each bench are in [`docs/notes/HANDOFF_PERFIL.md`](docs/notes/HANDOFF_PERFIL.md).

---

# 13. Repository Structure

```
WRO_FE_2026_FoxRobotics/
├── README.md                    ← this document (v2, international final)
├── ENGINEERING_JOURNAL.md       ← dated build log, phase by phase
├── docs/
│   ├── v1-national/README.md    ← v1 documentation, as judged at the national final
│   ├── engineering-log.md       ← decisions (DEC-xx), incidents (ERR-xx), milestones
│   ├── testing.md               ← testing workflow and checklists
│   ├── releases.md              ← one release per event (v1.0-nacional, v2.0-internacional)
│   ├── route-following-bench.md ← ESP32 route-following bench and measured dynamics
│   └── notes/                   ← internal working notes in Spanish (hand-offs, plans, to-dos), kept as written
├── src/
│   ├── ESP32/
│   │   ├── PurePursuit/             ← main firmware (development)
│   │   ├── PurePursuitCarrera/      ← race build of the obstacle round (generated)
│   │   ├── RondaAbierta/            ← Open Challenge firmware (ESP32 only)
│   │   ├── SeguirRuta/              ← route-following bench firmware
│   │   ├── CalibFox/                ← calibration sketch (servo, encoder, ToF)
│   │   ├── *Test/, PinScan/, TofId/ ← component bring-up sketches
│   │   └── _archive/, Controller_PI/← v1 firmware, kept for history
│   └── RASPI/cam/
│       ├── vision.py                ← camera capture + pillar detection
│       ├── camera_tuning/           ← Pi 5 lens-shading table, exposure
│       └── pure_pursuit/            ← Pi race software (§7.2)
│           ├── runtime_nuevo.py, config.py, carrera.json
│           ├── digital_map.py, localizer.py, route_mode.py, ring_route.py, start_fix.py, …
│           └── twin/                ← digital twin (simulator, benches, tests)
├── electrical/
│   ├── WRO_RevB_1/, WRO_RevB_2/     ← v2 KiCad projects (logic, power)
│   ├── export/                      ← schematics PDF/SVG + 3D renders
│   └── WRO_RevA/                    ← v1 board
├── Mecanica/Engranes/               ← involute gear generator + DXF profiles
├── models/                          ← v1 CAD/STL/renders; v2 in models/v2/ (cad, step, stl, renders)
├── schemes/                         ← index of the schematics (v2 → electrical/export/, v1 images)
├── logs_tof/                        ← raw ESP32 logs behind the measured tables (encoder, servo arcs, braking, grip, ToF, calibration laps)
├── scripts/                         ← run report, ToF vs ultrasonic comparison, Bluetooth route bench, lap analysis
├── remote/                          ← live camera view from the Pi in a browser (bench only, never during a run)
├── deploy/systemd/                  ← Pi autostart service (wro-runtime)
├── .github/workflows/deploy-pi.yml  ← every push to main is pulled on the Pi and the service restarted (stop + 4 s + start)
├── v-photos/  (v2/ = current car)   t-photos/   video/
└── requirements.txt
```

**Versioning.** Each event is a git tag + release notes in [`docs/releases.md`](docs/releases.md): `v1.0-nacional` is the code that won the national final; `v2.0-internacional` will be the code of the international final. Commit messages describe the change *and the evidence* (run number, twin result), so the history doubles as an engineering record.

---

# 14. Videos

| Challenge | Link |
|---|---|
| Open Challenge — v2, 3 laps | **[TODO: YouTube link]** |
| Obstacle Challenge — v2, 3 laps + parking | **[TODO: YouTube link]** |
| Digital twin — a full simulated race | **[TODO: optional]** |
| v1 (national final) — Open / Obstacle | [Open](https://youtu.be/orP-BNSG-6s) · [Obstacle](https://youtu.be/VI6V6hTf_HE) |

Each driving demonstration is at least 30 s long. Index: [`video/video.md`](video/video.md).

---

# 15. Cost Report

Prices in USD, as paid (including shipping where it applied).

| Item | Qty | Unit | Total | Notes |
|---|---:|---:|---:|---|
| Raspberry Pi 5 (8 GB) | 1 | 140 | 140 | Replacement for the 16 GB board lost on 2026-11-06 (16 GB bundle with cooler: 410) |
| ESP32 DevKitC | 1 | 8 | 8 | |
| Raspberry Pi Camera v2 NoIR + wide lens, 22-to-15-pin cable | 1 | 10 | 10 | Cable included with the camera |
| VL53L1X ToF | 4 | 6.75 | 27 | 500 MXN total |
| BNO085 | 1 | 27 | 27 | 500 MXN |
| N20 50:1 460 rpm with 12 CPR encoder | 1 | 20 | 20 | RobotShop |
| SG90 servo | 1 | 6 | 6 | |
| TB6612FNG carrier (SparkFun ROB-14450) | 1 | 5 | 5 | |
| MINI560 5 V buck | 2 | 6 | 12 | |
| 3.3 V regulator, level shifter, LEDs, switch, capacitors, headers | — | — | 13 | Regulator 1, level shifter 3, LEDs 1, switch 2, capacitors 1, headers 5 |
| PCB RevB (2 designs × 5 pcs, JLCPCB) | 1 | 40 | 40 | Including shipping |
| 3S LiPo 2200 mAh | 1 | 13 | 13 | |
| LEGO wheels, tires, axles, bushings | — | — | 12 | |
| PLA filament (~200 g) | — | — | 5 | |
| Screws, nuts, standoffs | — | — | 2 | |
| **Total** | | | **340** | Parts in the car; spare boards, test prints and the lost Pi 5 not included |

---

# 16. License

Released under the [MIT License](LICENSE) — code and documentation. This repository is public as required by the WRO Future Engineers rules and will remain public for at least 12 months after the competition.

*WRO Future Engineers 2026 — FoxRobotics — México*
