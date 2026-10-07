# FoxRobotics — Engineering Journal

> **Team:** FoxRobotics · **Event:** WRO 2026 Future Engineers — Self-Driving Car
> **Region:** México — Baja California

## Team & roles

| Member | Responsibility |
|---|---|
| **Erick Blanco** | Mechanical design; vehicle programming and control — both the Open and the Obstacle rounds |
| **Jesse Banda** | Vehicle programming, primarily computer vision; built the code structure the rest was developed on |
| **César Emiliano Ahumada** | Electrical bring-up and testing; schematic and PCB design |
| **Daniel Millán** | Coach |

Dated, chronological record of the build. It is the **timeline** companion to the two
thematic views in the [README](README.md):

- [§5.3 Iteration log](README.md#53-iteration-log) — structural changes by topic, with status.
- [§6.5 Milestones](README.md#65-milestones) — M1–M11 with acceptance criteria.
- [§5.5 Failure & Incident log](README.md#55-failure--incident-log) — post-mortems `ERR-01…ERR-09`.

Where an entry maps to one of those, it is cross-referenced rather than repeated.

**Sources & conventions**

- The **pre-repo period** (late March – early May 2026) is reconstructed from the team's
  design notes and is marked *(reconstructed)*.
- From 2026-04-13 onward, entries are anchored to **git history on `main`** (short hashes
  link into the log) and to the numbered on-track test sessions (`orillasNNN`) — the
  session counter has incremented on every integrated run since June 2026 and is now
  around **700**, so most tuning happened between commits, not in them.
- **branch** marks work implemented on a feature branch (`Workin`, `SectionTurning`,
  `ParedCenterline`) and not yet merged to `main`.

---

## Phase 0 — Vehicle design & per-peripheral bring-up *(reconstructed; pre-repo)*

**Focus:** one reliable vehicle, architecture fixed before any code — Raspberry Pi 4 for
vision, ESP32 for all real-time I/O ([README §1.1](README.md#11-design-goals)).

### 2026-03-30 → 04-05 — Semana Santa design week
A full week on the vehicle before anything else: SolidWorks CAD of the whole car, fit
checks and tolerance checks on every mating part, packaging the chassis around the
electronics (Raspberry Pi 4 first). Decisions locked here and never revisited: two-controller
split (Pi 4 + ESP32), rack-and-pinion Ackermann steering, rear-wheel drive with a printed
open differential, ~20 × 10 cm footprint for obstacle-field clearance.
→ [README §2](README.md#2-mechanical-design--mobility), [§2.4](README.md#24-steering--rack-and-pinion-with-ackermann-geometry).

### 2026-04 — Chassis, base wiring, PCB layout started
Chassis printed in PLA on a Bambu Lab A1 and assembled with LEGO axles/wheels
(**M1**). Base circuit designed and validated on a breadboard (**M3**). Custom PCB
laid out to fit the finished mechanical model — designed *after* the CAD, not before
(**M4**). → [README §3.3](README.md#33-pcb--wiring).

### 2026-04-13 → 04-27 — First commits: CAD + PCB
Repository created (git started well after the project did). CAD model, schematic, and
PCB shape committed; routing begun.
`283078e` · `ade099b` · `c9ce1e2` · `7e2a3d3` · `51ea239` (final PCB shape) · `f015e0d` (routing).

### ~2026-04-25 → 05-09 — Per-peripheral ESP32 bring-up
Each sensor and actuator bench-tested in isolation with a dedicated sketch, ESP32 only —
each HC-SR04, the servo, the DC motor, gyro read, serial echo
([`src/ESP32/TestCodes/`](src/ESP32/TestCodes/)) (**M2**). First combined controller
sketch alongside them. By 2026-05-09 this produced a **slightly functional Open-round
car running on the ESP32 alone** (no Raspberry Pi yet). `6474f65` (TestCodes +
FirstController).

---

## Phase 1 — First closed-loop Open Challenge run (May 2026)

**Focus:** get the car around the track on wall following alone.

### 2026-05-09 — First working Open-round controller
Single-loop PID on `distL − distR`, two VL53L0X ToF sensors, 3-state FSM
(`Controller_PI.ino`). First lap capability. `61544cb`.
→ [README iteration log "First firmware"](README.md#53-iteration-log).

### 2026-05 — `ERR-01`: ToF lost the black wall
Against the matte-black outer wall the VL53L0X range collapsed past ~70 cm; white walls
read fine. Reproduced on the bench with a single sensor, so not a wiring fault. **Both
channels switched to HC-SR04 ultrasonic** — traded ±3 mm for ±15 mm and closed the gap in
software later. → [README ERR-01](README.md#err-01--vl53l0x-lost-the-black-wall-past-70-cm).

### 2026-05 — `ERR-02`: wall error goes blind at high yaw
On corner entry the car drifted into a wall while the PID output stayed small: the
HC-SR04 beam cone hits the wall obliquely once yawed, both sides over-report, `distL − distR`
sits near zero. Motivated the **cascade PID** (outer wall→heading, inner heading→servo via
IMU) built over the next weeks. → [README ERR-02](README.md#err-02--wall-error-goes-blind-at-high-yaw),
[§4.6](README.md#46-cascade-pid-always-running-underneath).

### 2026-05-23 — Camera bring-up
Pi camera wired; track-edge (`detect_orillas.py`) detection and the BEV calibration tool
scaffolded; threaded recorder added so runs are reviewable. Many same-day iterations.
`21fe8df` · `5797487` · `345b430` (`record_orillas.py`).

### 2026-05-25 → 05-26 — Repo reorganized, first README
`src/` and `models/` restructured; first README written.
`6a448ca` · `ea70bfc` · `dde0543` · `a3b2fe6`.

---

## Phase 2 — Two controllers, one link (June 2026)

**Focus:** split vision and real-time control across the two processors and make the link
survive latency.

### 2026-06-04 → 06-05 — Integration pass + UART diagnostics
Vision, `wro_runtime.py`, and `Controller_PI.ino` wired end-to-end. UART framing tools
added (`diagnose_uart.py`, `test_serial_debug.py`, `test_uart_simple.py`); dead code
parked; `wro-runtime.service` drafted. `0a9251f` · `9169693` · `d6f677a` · `a8345dd`.

### 2026-06-08 → 06-09 — Two-controller split formalized + deploy infra
VNC for live view on the Pi; a self-hosted GitHub Actions runner and push-to-deploy CI;
an 800 ms timeout so a late packet makes the ESP32 reuse the last command instead of
stalling. `69f82a9` · `a065d78`.
→ [README §4.1](README.md#41-system-overview--two-controllers-one-link),
[§5.2 trade-off 1](README.md#52-key-engineering-trade-offs).

### 2026-06-11 → 06-13 — Pure Pursuit workspace
`FixedController` for the Open round (`711c6ec`); `pure_pursuit/` scaffold with
`INSTRUCCIONES.md` and `calibrate.py` (`95f3744`).

### 2026-06-22 — Centerline extraction + geometric Pure Pursuit
Floor-colour centerline in the BEV, first `PurePursuit.ino`, first `centerline.py`.
Replaces the reactive pillar-offset PID, which over-reacted near and under-reacted far.
`79706be` (camera-duplicate fix) · `e494d8c` (centre algorithm) · `556266b`.
→ [README iteration log v2.0](README.md#53-iteration-log),
[§4.3](README.md#43-vision-pipeline-raspberry-pi--pure_pursuit).

### 2026-06-25 — Last commit before the July break
`2b0f82d`. Repo quiet until 2026-08-07.

---

## Phase 3 — Wide lens & white balance (July – 2026-08-07)

### 2026-07 (first week) — Lens swap 63° → 120°
Wide-angle NoIR lens arrived in the first week of July, so the camera sees far enough
ahead to read a pillar and the corner on the same frame. The rest of that period went to
bench tests aimed at removing the reddish cast it introduced (see `ERR-03` below).

### 2026-08-07 — `ERR-03`: red cast on every frame
The NoIR lens has no IR-cut filter; every frame came out heavily red-tinted and all HSV
masks broke. Fixed by pinning libcamera white balance
(`awb-enable=false colour-gains=<1.2,1.5>`) and re-tuning the red/green ranges on the
corrected image. Also a first pass at a false-turn bug.
`9ada7a0` (FalseTurn bug) · `140051a` · `945c2f1` (RGB range cleanup).
→ [README ERR-03](README.md#err-03--wide-angle-noir-lens-put-a-red-cast-on-every-frame).

---

## Phase 4 — Sim, calibration, corner lines (2026-08-11 → 08-19)

### 2026-08-11 — Offline kinematic sim
`pure_pursuit_sim.py` (bicycle model of centerline + Pure Pursuit + obstacle memory) and
`runtime_nuevo.py`, so controller logic can be tuned without track time. `0a2c5d2`.
→ [README iteration log v2.1](README.md#53-iteration-log).

### 2026-08-12 — Obstacle detection hardening
`vision.py` shape filtering — area, solidity, aspect ratio — so the mat's coloured lines
are not read as pillars. Several same-day passes.

### 2026-08-13 — BEV calibration 4 → 9 points
RANSAC fit over a 3×3 grid of physical floor markers; a 4-point fit is exact and cannot
reject a mis-click or check reprojection error. `daf1e03` (a batch of `test`/`Revert`
commits precede it — the failed attempts). → [README iteration log v2.2](README.md#53-iteration-log).

### 2026-08-15 — "line memory" tried and reverted
`89f9d51` → `270b783` same day. Kept in the log as a rejected approach.

### 2026-08-17 → 08-19 — Orange / blue ground-line tracking
Row-by-row tracking of the mat's corner lines in the BEV — a turn trigger and a
"my straight vs. the next one" boundary for obstacles. `d3a611e` · `8edc1b8`.
**2026-08-18 was a heavy revert day** — a batch of centerline and `.ino` experiments all
backed out (`663d45b` … `42b397a`); the net keeper was `770c9a3` (RemoverLines).
`d592798` (WallDistance) starts feeding ultrasonic wall distance toward steering.
→ [README iteration log v2.3](README.md#53-iteration-log).

---

## Phase 5 — Wall-aware steering; Open Challenge validated (2026-08-20 → 08-28)

### 2026-08-20 → 08-21 — `ParedCenterline` merged
Centerline biased by ultrasonic wall distance (it drifted toward the outer wall on wide
corners without it); V2 protocol fixes; BetterSteering / BetterDodge passes; ghost-object
handling started. `f5066e1` · `24b2ec7`.
→ [README iteration log v2.4](README.md#53-iteration-log).

### 2026-08-24 — Ghost obstacle after a turn
Cans left in memory after a turn kept a phantom keep-out. First fix reverted, v2 shipped.
`a41ac0e` → `3c96abf`. `333b0e1` ("FreshStart: GirandoStep1") begins a rebuild of the
turn logic.

### 2026-08-25 → 08-27 — Line-classification & centerline tuning stretch
Long run of small commits and reverts on orange/blue line classification and centerline
stability; segmented-obstacle and interior-turn prototypes (`510a89a`, `b55e317`);
turning-detection v2. `fb81fe6` ("back2Basics", 08-27) resets the branch after the
centerline experiments diverged.

### 2026-08-27 — Obstacle-pass pulse fixed; earlier dodge
`ff9c83b` — the `pasado` "obstacle physically cleared" pulse was never firing.
`68aab07` — earlier/stronger dodge and earlier RECUPERANDO. `ab574ab` — rolling-memory
`ds_px` brake on hard turns. `57d3218` — lateral rebase: enter RECUPERANDO when the can
crosses the axis, not when it passes behind.
→ [README iteration log v2.5](README.md#53-iteration-log).

### 2026-08-28 — `ERR-08`: Pi undervoltage → frame-rate rescale
~0.3 V drop on undersized power leads browned out the Pi under load. Re-fed in 22 AWG;
loop went **~7 → ~14 fps**. Half the per-frame knobs were then calibrated at the old rate
and firing twice as fast — every one re-scaled. `e4ca4f9` · `1a19cdb`.
→ [README ERR-08](README.md#err-08--raspberry-pi-undervoltage-from-undersized-power-wiring),
[iteration log v2.7](README.md#53-iteration-log).

### 2026-08-28 — Hot start + run recording
Pipeline runs disarmed during the button delay; the ESP32 is gated on the first real V2
line so the car never rolls on the fallback PID; MJPG `.avi` HUD capture from the moment
the button is pressed. `3245c1c` · `2355caf` · `f257371`.
→ [README iteration log v2.6](README.md#53-iteration-log).

### 2026-08-28 — `ERR-07`: RECUPERANDO anchor drifts (experiments)
The geometric dead-reckoning anchor for the recovery trigger drifted 200–400 mm in
1–2 s and fired early or late; `SPEED_SCALE` at 1.0 / 0.35 / 0.60 all wrong somewhere.
`f24c547` · `25d0565` and their reverts. Resolved the next day.
→ [README ERR-07](README.md#err-07--recuperando-trigger-tied-to-assumed-linear-speed).

### 2026-08-28 — **M8: Open Challenge validated**
10 complete autonomous runs from varied start positions and field configurations, track
direction latched by the car, no wall contact, correct finish, ~12 s per run at 60 %
motor. Focus moves to the Obstacle Challenge.
→ [README MIL-01](README.md#mil-01--open-challenge-full-autonomous-run).
**Evidence still to log:** per-run video links, commit SHA per run, CW/CCW split,
battery voltage before/after.

---

## Phase 6 — Obstacle-round root causes (2026-08-29)

### 2026-08-29 — `ERR-07` fixed: measured-state RECUPERANDO trigger
Anchor retired. New trigger reads state that is already measured — centerline avoidance
weight (ARM), the memory's own "is a can still in the way" test (CLEAR), and a heading
snapshot (SKEW ≥ 25°). A gentle dodge that straightens itself never stops for a recovery.
`52549c8`, then `ab637aa` · `ef40152` · `5eee517` · `0306cfd` · `98da50a` tuning.
→ [README iteration log v2.9](README.md#53-iteration-log).

### 2026-08-29 — `ERR-06` fixed: pivot-trap
Near a can the steer term hit ±0.9 (≈ full lock) so the car pivoted in place instead of
arcing; the can's `y` never advanced, so RECUPERANDO never armed. Fixes: `LOOKAHEAD_MIN_PX`
60 → 78; adaptive look-ahead and steer-gain re-keyed on the **longitudinal** gap;
`forget_color_obstacles()` on a confirmed pass so the centerline un-bends. `1daf563`.
**Confirmed clean at session orillas 417** — both cans cleared, all 12 turns.
→ [README ERR-06](README.md#err-06--the-car-pivoted-in-place-instead-of-arcing),
[iteration log v2.8](README.md#53-iteration-log).

### 2026-08-29 — Orange-line direction, fast verdict
Slope evaluated from the first frame; armed-debounce for a green can seen just after a
turn. `a9acdde` · `13b163b`.

---

## Phase 7 — Hardening, segmented turns, documentation (2026-08-31 → 09-06)

### 2026-08-31 — Exterior cone at the corner mouth
A can right at the corner: drive straight past it, *then* turn, so the blind arc can't
sweep into it. `4e5ca40`. → [README iteration log v2.11](README.md#53-iteration-log).

### 2026-08-31 — Turn direction inferred during the run
`TurnDirectionTracker` — direction latched from the orange corner-line slope while
running, not pre-set at check-in. `3a6735e` · `a72a751` · `e641817`
(`LINE_FIT_MAX_SLOPE_DEG` 45 → 72, one turn sense wasn't fitting).
→ [README iteration log v2.10](README.md#53-iteration-log).

### 2026-08-31 — Camera-safe service restart
CI and ops use `stop → sleep 4 → start`; `systemctl restart` doesn't release the CSI
device and the camera wedges. `9aff6c9`.
→ [README iteration log v2.12](README.md#53-iteration-log).

### 2026-08 → 09 — **branch** work (`Workin`, `SectionTurning`) — not on `main`
- **Segmented obstacle turns** `CRUCERO → MANIOBRA`: cruise to a set front-sensor
  distance, then a forward arc or a reverse pivot chosen from the outer-wall distance;
  a mandatory motor-coast phase before every direction reversal after **`ERR-04`** (a
  reversal with no coast destroyed a TB6612 and an N20). → README iteration log v3.0,
  [ERR-04](README.md#err-04--tb6612-and-motor-destroyed-by-a-reversal-with-no-coast-delay).
- **Mid-turn cone detector** (`mid_turn.py`) — Phase 1, logging only (`[MTURN]`).
  → README iteration log v3.1.
- **Open round on pure wall + gyro PID** — Pi steer ignored; per-round `AngGiro` /
  `MOTOR_MAX`; `giroArmado` corridor-arm; front-wall approach slow-down; `TERMINANDO`
  return-to-start finish. → README iteration log v3.2, `ERR-05`.

### 2026-09-01 → 09-06 — Documentation pass
README expanded to the current structure (architecture, V2 spec, ERR log, iteration log,
risk table, milestones); v-photos; wiring diagrams + PCB render; 3D models; Open Challenge
video and YouTube link; milestone journey.
`3955cf3` · `5eee9a2` · `a4379be` · `0bb3fd1` · `baaa2da` · `eca476b`.

---

## Phase 8 — National final (2026-09-07 → 09-19)

### 2026-09-07 → 09-17 — Pre-final hardening (summary)
The work between the documentation pass and the final is listed row by row in the
[README iteration log](README.md#53-iteration-log) (v3.3 → v3.9):
- Gyro heading-hold by default on clean straights; run 700 was the first clean 12-corner Obstacle run.
- Start from the parking lot (`INICIO`).
- Floor-referenced color correction.
- Camera check before arming.
- Obstacle-memory merge guard.
- Red hue range for the new venue.
- Parallel parking: perfect parks in runs 1025, 1052 and 1064. Run 1064 was a full clean Obstacle round including the park.

### 2026-09-18 → 09-19 — **M14: 1st place, Mexico national final**
Last changes before the rounds were a color calibration for the venue lighting and
firmware tuning (`fc2bc3e`, `7c3db40`, `e18ff45`). The code that ran was committed as
`d5cc66d` ("WRO National") and becomes release **`v1.0-nacional`**
([releases](docs/releases.md)). Documentation score: 26/30.

---

## Phase 9 — After the final: check, diagnose, choose the v2 components (2026-09-21 → 09-27)

### 2026-09-21 → 09-22 — Is the car still the same?
First week back: test runs to confirm the car behaves as at the final, before changing anything. They turned up two problems the final had hidden:
- **Strong light, run 1190:** the orange tape was read as red and a pale patch of floor as green. Fixed with two lighting-independent tests per blob (`RED_GR_MAX`, `GREEN_S_MIN_MED`). Replayed over 6 recorded runs: red detections in 1190 went 34 → 19 and green 164 → 65; every rejected detection was checked by eye and was false. → [ERR-17](docs/engineering-log.md#err-17--strong-light-tape-read-as-red-pale-floor-read-as-green).
- **Run 1188, corner 4:** the forward/reverse corner maneuver flipped at the last moment, because the outer-wall distance sat right on its 28 cm threshold, and the arc jammed. Probably the same failure we saw at the final. Fixed with hysteresis and one shared decision. → [ERR-22](docs/engineering-log.md#err-22--the-corner-maneuver-changed-its-mind-at-the-last-moment).

Both fixes are in `96145ba`.

- **Bird's-eye calibration re-checked** with 9 measured floor marks. The calibration we raced with compresses depth by ~2×. A new calibration (9 px error) is saved aside; the old one stays for racing until all the pixel-based settings are moved to millimeters. A refactor of the perspective transform was tried and reverted the same day (`f30328f` → `b2d91d6`).

### 2026-09-21 — Where does the time go? (Raspberry Pi 4 profiling)
Measured on the car with the real race code:
- The main Python thread used **97% of one core** and the loop ran at **14–18 fps**, depending on the scene.
- Camera capture was only ~5 ms; color detection ~20 ms; bird's-eye view + centerline ~45 ms.
- Building the debug overlay only on the frames that get recorded gained **+8.3%** (A/B test on the car: 16.7 → 18.1 fps).
- With the remote-view client connected, the Pi reached **84 °C** and throttled ([ERR-27](docs/engineering-log.md#err-27--the-remote-desktop-client-overheated-the-pi-4)).

Conclusion: a faster capture method would gain at most 5–10%. The real limit is the CPU and its temperature. → decision to move to a **Raspberry Pi 5** with an Active Cooler.

### 2026-09-21 → 09-24 — What to improve, and with which parts
We reviewed the logged failures by root cause. Most traced back to the car not knowing its own state well enough:
- Speed was open-loop, so the behavior changed with battery charge ([ERR-11](docs/engineering-log.md#err-11--a-fresh-battery-over-rotates-the-dodges)).
- The heading came from gyro-rate integration only ([ERR-23](docs/engineering-log.md#err-23--systematic-heading-offset-after-every-corner)).
- The front ultrasonic produced phantom echoes ([ERR-19](docs/engineering-log.md#err-19--phantom-echoes-from-the-front-ultrasonic)).

We also read the public repositories of other teams: the 2026 Mexican 30/30 documentation and the top teams of the 2025 World final. That gave us real data on distance sensors against the black wall:
- Multi-zone ToF sensors and the VL53L1X both reach ~80–100 cm.
- A low-cost LiDAR refreshed too slowly for our speed.
- ToF sensors mounted low hit the floor.

**Decisions:**

| Need | Chosen | Instead of | Why |
|---|---|---|---|
| Compute and heat | Raspberry Pi 5 16 GB + Active Cooler + M.2 HAT+ | More optimization on the Pi 4 | The profiling above |
| Near-field distance, rear sensing | 4 × VL53L1X, keeping the ultrasonics | LiDAR, multi-zone ToF | Simple driver, enough range for a near-field sensor, and teams that used it documented it well. The ultrasonics stay because the two types fail differently against the black wall |
| Heading | BNO085 (on-chip fusion) | MPU-6050 integration | 6.5% heading loss per run, −7.5° bias per corner |
| Speed and distance | Pololu 50:1 motor with encoder | Open-loop N20 | Battery-voltage dependence; the rules' reference build suggests an encoder |
| Clean power for the Pi | A second Mini560 only for the Pi | One shared regulator | Servo current spikes must not reach the Pi |

**All components were ordered this week.**

### 2026-09-26 — Firmware cleanup
One parking-mode switch (`PARK_MODO`: none / nose-in / parallel) replaces three booleans that had to be changed together; unused settings removed. `cdf6251`.

---

## Phase 10 — Components arrive; new mechanics; digital twin (2026-09-28 → 10-04)

### 2026-09-28 → 10-04 — Bench tests of the new components
The components arrived, and each one was tested alone before integration. One sketch per question:

| Sketch | Question it answers |
|---|---|
| [`TofId`](src/ESP32/TofId/TofId.ino) | Is this sensor an L0X or an L1X? It reads the model register directly, with no library. |
| `TofTest1`, `TofTest1_L1X` | Does one sensor range on the bus? |
| [`TofTest4`](src/ESP32/TofTest4/TofTest4.ino) | Can all four share one I²C bus? Each one is woken up with its XSHUT line and given its own address (0x30–0x33). |
| `BnoTest`, [`BnoRvcTest`](src/ESP32/BnoRvcTest/BnoRvcTest.ino) | BNO085 over I²C vs UART-RVC. RVC chosen: one wire, 100 Hz, checksum, no library, and it stays off the ToF bus. |
| [`TofBnoTest`](src/ESP32/TofBnoTest/TofBnoTest.ino) | All four ToF plus the BNO085 at the same time. It reports rates (ToF ~20 Hz each, BNO ~100 Hz), lost packets and ToF error codes. |
| `ServoTest` | The servo's real usable range for the new steering |

### 2026-09-28 → 10-04 — v2 mechanical design
Full redesign of the chassis to make the car shorter, around the new parts ([README §2.6](README.md#26-v2-chassis--shorter-car-pi-on-top)):
- The motor turns parallel to the rear axle, driving the differential with spur gears (module 1, 15T/30T, same 2:1 as the old bevel pair). Involute profiles generated as DXF by our own script.
- The battery moves to the center, crosswise, in a side-loading tunnel where the Raspberry Pi used to be.
- The Raspberry Pi 5 goes on top, open to the air for its fan.
- Overall length goes from ≈24 cm (including the extension added to leave the parking lot) to ≈17 cm (168.7 mm in CAD). Wheelbase ≈147 → ≈113 mm. Camera tilt ≈20° down.

We also started reworking the steering for the shorter car (finished in Phase 11) and worked out how to fit everything.

### 2026-10-02 → 10-06 — Digital twin (Jesse) — **branch** `digital-twin`, not on `main`
A **software-in-the-loop simulator** of the whole car. It runs the real Pi code (`runtime_nuevo.py`) and the real ESP32 firmware, compiled for the PC, against a simulated field with randomized pillar layouts (one "seed" per layout). Each run produces the same logs as the real car. Highlights, with the full record in the branch's `docs/twin_plan.md`:
- **First batch of 7 layouts:** 4 hit a green pillar 1–3 s after a right turn. One was traced frame by frame to a rule that released the "wait before turning" block too early when a green sat in the first column of the next straight.
- **v2 hardware preset (`hw_nuevo`):** encoder, 4 ToF, BNO085 model, the new dimensions, and the Raspberry Pi 5. Frame-based settings were made independent of the frame rate (`df84b28`, `25de149`).
- **Parallel parking planned with the map of the field:** 91/100 successful parks over a harness of start positions, and 98/100 when grazing the outer wall is allowed (`58a1338`, `87532bb`).
- **Determinism checks** (the same seed gives the same run) and a sensor-noise ensemble, so a change can be compared across many layouts instead of one track run.

---

## Phase 11 — Raspberry Pi 5 bring-up, steering finalized, PCB started (2026-10-05 → 10-07)

### 2026-10-06 — Raspberry Pi 5 set up for headless work
- SSH with key-only login over Tailscale.
- Logs that survive reboots.
- The repository and a Python environment that reuses the system OpenCV, which has GStreamer.
- `rpi-lgpio` in place of `RPi.GPIO`, which does not support the Pi 5.
- The ESP32 link moved to `/dev/ttyAMA0` (on the Pi 5, `/dev/serial0` is the debug connector).
- Boot time 53 s → 13 s after the first boot.

### 2026-10-06 — Raspberry Pi 5 benchmark
The unmodified race runtime, replaying recorded run 1190 as the camera input at 30 fps, ran at **40–41.5 fps** with the Pi at 46 → 52 °C and no throttling. The Pi 4 managed ~15.5 fps on the same run.

Found on the way: the main loop doesn't wait for a new camera frame, so at 40 fps it re-processes about one frame in four. The settings counted in frames would then run ~3× faster in real time. Open item: cap the loop rate, or move those settings to seconds (the twin branch already did the latter).

### 2026-10-06 → 10-07 — Camera bring-up on the Pi 5
- The camera wasn't detected automatically → driver loaded explicitly for port CAM1.
- The race pipeline failed with `not-negotiated` → fixed with `format=BGRx` (30 fps, ~2% CPU) — [ERR-28](docs/engineering-log.md#err-28--raspberry-pi-5-the-race-camera-pipeline-would-not-start).
- **Magenta ring at the frame edges** (corners R/G 1.31–1.40 on a white sheet). Root cause: the factory lens-shading table, made for the stock lens. We calibrated a new table for our lens from 5 photos of a white sheet with the official Raspberry Pi tuning tool, then symmetrized its brightness part to remove the uneven lighting of the sheet. Result: R/G and B/G = 1.00 at the center and all corners. The table is shipped in the repository with the procedure, to repeat it at the venue — [ERR-29](docs/engineering-log.md#err-29--raspberry-pi-5-magenta-ring-at-the-frame-edges), [`camera_tuning/`](src/RASPI/cam/camera_tuning/README.md).
- A browser-based live view and color-calibration tool (`cam_web.py`), because the Pi 4's remote-desktop client doesn't work on the Pi 5's desktop.

### 2026-10-06 — Steering geometry finalized
- **Diagnosis:** we simulated the linkage in Python with the CAD dimensions (model within ~1° of the CAD). The old geometry was **anti-Ackermann**: 39.5° inner wheel against 42.8° outer.
- **Fix, reprinting only the rack, the two tie rods and the pinion:** rack joints 43 → 47 mm, tie rods 18.13 mm, pinion 14 → 19 teeth (servo moved 2.5 mm).
- **Result in CAD:** **53.1° inner / 37.0° outer, ~106% Ackermann**. Turning radius ~208 → ~115 mm together with the shorter wheelbase ([README §2.4](README.md#24-steering--rack-and-pinion-with-ackermann-geometry)).

### 2026-10-05 → 10-07 — Mechanical design finished; tests with the new parts; PCB started
- Mechanical v2 design completed, around the final steering.
- Tests with the new components on the car.
- Start of the **two-floor PCB**: a power floor with both regulators, the motor driver, switch and fuse; and a logic floor with the ESP32 and the connectors for 3 ultrasonics, 4 ToF, BNO085 and servo ([README §3.3](README.md#v2-pcb--two-floor-board-in-design)).

### 2026-10-06 → 10-07 — Documentation for the international final
- The `parking` branch merged into `main` (`844195e`, `2dd11cf`). The README judged at the final is kept as the base, with v2 added on top.
- New: [releases](docs/releases.md), [test record](docs/testing.md), [engineering log](docs/engineering-log.md) (ERR-10 onward), §5.0 design constraints, the v2 power budget, sensor placement against the field geometry, and the BNO085 + ToF sections.

---

## Open items (as of 2026-10-07)

| Item | Milestone | State |
|---|---|---|
| Frame rate on the Pi 5: cap the loop or move per-frame settings to seconds | M15 | Open; done on the twin branch, not yet on `main` |
| Re-check the red/green/orange/magenta thresholds under the new camera calibration | M15 | Open |
| Encoder, BNO085 and ToF in the race firmware (ToF and BNO log-only first) | M16 | Bench-tested; twin model on branch |
| BNO085 heading with the motor running at standstill (magnetometer check) | M16 | Pending |
| Two-floor PCB designed, fabricated, wired; v2 wiring diagram | M16 | In design |
| v2 CAD/STL and renders into `models/`; measure the printed car (size, weight, turning radius) | M16 | Pending |
| Merge the `digital-twin` work into `main` | M16 | On branch |
| Validation matrix for v2: both directions, 3 consecutive clean runs | M16 | Not started |
| Tag `v1.0-nacional`; tag `v2.0-internacional` before the event | — | Pending |
| Documentation deadline | — | 2026-11-24 |

---

## How this journal is maintained

- New entry at the bottom of the current phase, dated, with the **why** and the **measured
  result** — not just what changed.
- When a change lands on `main`, add the commit short-hash. Branch work is labelled
  **branch** until merged.
- [README §5.3](README.md#53-iteration-log) stays the thematic summary; this file is the
  timeline. Keep them consistent when either changes.
