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

## Open items (as of 2026-09-06)

| Item | Milestone | State |
|---|---|---|
| *Mine vs. beyond the corner* classification — a can seen over the corner is sometimes assigned to the wrong straight | M10 | **Current focus** — `ERR-09`, open |
| Parking maneuver inside the bay | M11 | Not started — after M10 |
| Merge `SectionTurning` / `Workin` to `main` once track-tuned | M9 | On branch |
| Evidence backlog: per-run video + SHA for the 10 Open runs; RECUPERANDO true/false-fire tally; obstacle clean-run ratio | M8, M9 | Pending |

---

## How this journal is maintained

- New entry at the bottom of the current phase, dated, with the **why** and the **measured
  result** — not just what changed.
- When a change lands on `main`, add the commit short-hash. Branch work is labelled
  **branch** until merged.
- [README §5.3](README.md#53-iteration-log) stays the thematic summary; this file is the
  timeline. Keep them consistent when either changes.
