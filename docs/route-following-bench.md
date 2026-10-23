# Route-following bench (ESP32 only) — results of 2026-10-22/23

This bench tests the motion layer of the car without the Raspberry Pi or the camera. The PC sends a route (a list of points in mm), and the ESP32 follows it with pure pursuit on its own odometry (encoder + BNO085). The lateral ToF sensors check the car's real position against the walls. We used it to tune the car at full speed (PWM 255) and to run the Obstacle Challenge layout with routes from the digital twin's planner. All numbers below come from the logs in `logs_tof/` (one `.txt` + `.png` per run, named by date and time).

## 1. Setup

| Part | File | What it does |
|---|---|---|
| Firmware | [`src/ESP32/SeguirRuta/SeguirRuta.ino`](../src/ESP32/SeguirRuta/SeguirRuta.ino) | Pure pursuit on the rear axle (same algorithm as `rutaSteer()` in `PurePursuit.ino`), yaw-rate damping, wall correction with the outer wall, soft finish. Route protocol `RTB/RTP/RTE` (same as the race firmware), plus `RTS` (start pose), `RTM` (wall map) and `SET`/`PAR` (any parameter by name, no re-flash). Logs pose `P`, raw ToF `T`, corrections `C` over Bluetooth. |
| PC tool | [`scripts/ruta_bt.py`](../scripts/ruta_bt.py) | Builds the route (straight, lane change, S, lap, CSV, obstacle course), simulates it, uploads it over Bluetooth, starts the car, records the log and plots planned vs simulated vs driven vs wall-measured. Prints a `RESUMEN` line per run. |
| Obstacle routes | `ring_route.py` from the twin (worktree `WRO2026-isla`) | Dynamic-programming planner: red signs passed on the right, green on the left, footprint clearance ≥ 55 mm, radius ≥ 170 mm. `ruta_bt.py obstaculos` converts its frame (mat center, heading 0 = +y) to the bench frame (outer corner, heading 0 = +x). |
| PC check of the firmware | scratchpad harness (`sil2.cpp`) | The real `.ino` compiled with Arduino mocks, bicycle physics, ToF that ray-cast the true walls, and injected errors (BNO scale, encoder over-count, placement). Used to check every firmware change before flashing. |

Frame: mm, origin at the outer corner behind the start (bottom-left), heading in degrees, counter-clockwise from +x. The tracked point is the center of the rear axle.

## 2. Final configuration

Script defaults since commit `c3b05b3d` (laps: add `--sentido izq|der`; obstacles: `ruta_bt.py obstaculos --senales ...`).

| Parameter | Value | Why |
|---|---|---|
| PWM straight / curve | 255 / 230 | 255 in curves gave the same time (28.7 s) and less consistent position (§5) |
| Lookahead | 275 mm | 150 made the servo oscillate, 200 still oscillated, 250–300 smooth (§5) |
| Yaw-rate damping `kd` | 0.04 °wheel per °/s | 0.08 started a fast (~3 Hz) oscillation after corner 1 |
| Servo center | 75 | Mean servo on straights 74.4 / 75.2 / 74.7; 76 tested, no better |
| Wheel → servo | 1.75 °/° left, 1.78 °/° right; max wheel 40° / 42.5° | Turning radii measured on the mat at PWM 150 (123 mm right, 134 mm left) |
| Encoder | 5.45 counts/mm, no curve correction (`encCurva` 0) | See rejected hypothesis R-1 |
| BNO scale | 1.0 | See rejected hypothesis R-2 |
| Wall correction | position only (`pared 1`), gain 0.3 per reading, max 5 mm per reading, outer walls only | Best lateral result without oscillation (§5) |
| Start pose | `salidaPared 1`: lateral position from the outer wall, averaged while standing | Removes the placement error when the start is within ToF range |
| Finish | after the last turn, PWM steps down to 150, hard brake at the start line | Hard braking at 255 skidded 25–29 mm and the encoder missed it; now 7–12 mm |
| Lateral ToF faces | L 32.8 mm, R 36.8 mm from the centerline, 113 mm ahead of the rear axle | Measured with the car centered in the 991.7 mm corridor (L 463, R 459 mm) |

## 3. Results

### 3.1 Open lap, 3 laps at 255/230, 3000 mm route on the 2975 mm mat

Real position = rear-axle distance to the outer wall, measured by the ToF on the outer-wall side. Corridor center = 496 mm.

| Run | Direction | Walls | Time | Bottom / start | Right | Top | Left | Mean deviation | Servo std on straights |
|---|---|---|---|---|---|---|---|---|---|
| 142917 | CCW | off | 28.8 s | 499 | 515 | 560 | 522 | 28 mm | 5.3 |
| **150923** | CCW | on | 29.1 s | 498 | 512 | 517 | 486 | **12 mm** | 4.9 |
| 143643 | CW | off | 28.7 s | 500 | 562 | 481 | 553 | 35 mm | 4.3 |
| **144109** | CW | on | 29.1 s | 498 | 529 | 486 | 500 | **12 mm** | 5.5 |

User observation on the final runs: the car ended at the start line, about 1 cm towards the inside, with no visible oscillation. Run 150923 used a fresh battery (12.2 V). The same settings on a low battery (145520) gave 28 mm.

### 3.2 Obstacle course, 3 laps at 255/230

Layout: red T1 bottom, green X2 right, red T3 top, green X1 left (CCW). Planner: 6.8 m per lap, minimum radius 198 mm, footprint clearance to signs 101 mm.

| Run | Time | Lateral error (odometry) rms / max | Servo range | Final position (odometry) |
|---|---|---|---|---|
| 152412 | 26.2 s | 7 / 27 mm | 60–120 | 2.2 cm from start |
| 152546 | 26.3 s | 7 / 25 mm | 60–121 | 1.8 cm |
| 152651 | 26.6 s | 7 / 26 mm | 60–121 | 2.2 cm |

User observation: all three runs passed every sign on the correct side with no contact, and ended at the start. The car was not repositioned between runs. On this layout the car stays more than 650 mm from the outer wall most of the time, so the wall correction used only 4–13 readings per run: these runs are essentially odometry only.

## 4. Real vs simulation

The simulation in `ruta_bt.py` uses the same pure pursuit and damping as the firmware, a kinematic bicycle on the rear axle, a first-order servo lag and the measured wheel→servo map. The firmware itself, compiled on the PC, matches this simulation within 1–5 mm on every route.

The table below is with the first speed model (`v = 4.39 · (PWM − 57)` mm/s, speed changes instantly). Section 4.1 has the model after adding the measured dynamics.

| Run | Route | Time real / sim | Lateral rms real / sim | Lateral max real / sim | Servo std straights real / sim |
|---|---|---|---|---|---|
| 111616 | Straight 2 m, PWM 180 | 3.9 / 4.2 s | 5.0 / 0.0 mm | 10 / 0 mm | 2.9 / 0.0 |
| 112029 | S, PWM 180/150 | 4.6 / 5.0 s | 4.8 / 5.5 mm | 11 / 12 mm | 9.7 / 3.8 |
| 113619 | Lap, 180/150, lookahead 200 | 14.3 / 14.9 s | 5.6 / 3.3 mm | 10 / 9 mm | 5.9 / 5.4 |
| 121313 | 3 laps, 255/230, lookahead 200 | 28.1 / 27.4 s | 2.6 / 4.2 mm | 6 / 9 mm | 6.7 / 6.0 |
| 142217 | 3 laps, lookahead 150 | 30.7 / 27.2 s | 14.2 / 3.5 mm | 55 / 9 mm | 11.4 / 6.4 |
| 142749 | 3 laps, lookahead 275 | 28.5 / 27.0 s | 6.7 / 8.5 mm | 16 / 21 mm | 5.1 / 6.0 |
| 143643 | 3 laps CW, lookahead 275 | 28.7 / 27.2 s | 5.5 / 8.5 mm | 11 / 21 mm | 4.3 / 6.0 |
| 152546 | Obstacles, 3 laps | 26.3 / 24.6 s | 7.2 / 10.0 mm | 25 / 32 mm | 12.3 / 14.1 |

What the comparison shows:

* **Tracking error relative to the route** (what the car believes) is within a few mm of the simulation at lookahead 200–300, often better. The real car is not worse at following its own estimate.
* **The real car is 5–6 % slower at 255** (28.5 vs 27.0 s). The speed model came from PWM 90–180; in curves the car loses ~4 % more (805 → 776 mm/s at 255 in curves, tyre scrub).
* **Servo lag.** With the first guess (80 ms) the simulation did not oscillate after corners and the car did. A first-order lag of **0.12 s** reproduces the real servo range and error at PWM 210, 240 and 255. With that value the simulation predicted the damping gain range (0.04–0.15) correctly, but not the fast oscillation of `kd` 0.08 (servo backlash and speed limit are not modeled).
* **Lookahead 150 is the clearest failure of the model:** the simulation stays calm (std 6.4) and the car oscillates (11.4, servo hitting 18–145). Short lookahead amplifies the unmodeled servo backlash.
* **The S-curve** shows more servo activity on the car (9.7 vs 3.8) with the same tracking error: the real servo corrects its own play.
* **The simulation cannot see odometry errors.** The car's belief matched the simulation, but the walls showed 1–6 cm of real lateral error per straight (§3.1). That gap is what the wall correction closes.

### 4.1 Speed model fitted to the logs

From the pose log (25 Hz) of the runs above:

| Quantity | Measured | How it is in the model now |
|---|---|---|
| Straight speed per PWM | 90 → 145, 150 → ~400, 180 → 540, 255 → 830–850 mm/s | Piecewise linear through those points (the old line gave 869 at 255) |
| Speed response | Start: 10 → 90 % of top speed in 0.63–0.73 s (0.8–1.0 m/s² mean, ~2 m/s² peak) with the firmware's 0.5 s PWM ramp. Step 255 → 150: 90 % of the drop in 0.52–0.60 s (0.6–0.8 m/s²) over 30–35 cm | First-order lag, τ = 0.24 s |
| Speed loss in curves | 805 → 776 mm/s with ~25° of wheel at PWM 255 | −0.16 % per degree of wheel |
| Hard brake (TB6612 short brake) | From 400–520 mm/s: 9–15 mm (~8.5–9 m/s²). From ~830 mm/s the encoder says 24 mm (~14.5 m/s², above the ~2.5 m/s² grip): the wheels lock and the car slides, which the encoder does not see | 8.5 m/s² applied at the end of the route |

Result with the new model (same runs, both models use the same pure pursuit):

| Run | Time real / sim | Straight speed p90 real / sim | Brake real / sim |
|---|---|---|---|
| 131742 (180/150) | 45.3 / 43.8 s | 533 / 540 mm/s | 15 / 13 mm |
| 142317 (lookahead 250) | 28.4 / 28.5 s | 849 / 829 | 8 / 13 |
| 142749 (lookahead 275) | 28.5 / 28.4 s | 842 / 829 | 10 / 13 |
| 143643 (CW) | 28.7 / 28.7 s | 836 / 829 | 8 / 13 |
| 150923 (walls, fresh battery) | 29.1 / 28.7 s | 834 / 829 | 10 / 13 |
| 152546 (obstacles) | 26.3 / 26.0 s | 834 / 826 | 10 / 13 |

Lap times now agree within 0.1–0.4 s at 255 (they were 1.4–1.7 s fast). Still not modeled: servo backlash (why lookahead 150 oscillates on the car and not in the simulation), the skid when braking from full speed, and the battery's effect on speed.

## 5. Parameter sweeps

All 3 laps CCW at 255/230 unless noted.

| Sweep | Values → result |
|---|---|
| Lookahead (no walls) | 150: servo std 11.4, 30.7 s, ToF lateral at corner exit +27 mm · 200: std 6–8 · **250: 5.1, +18 mm · 275: 5.0–5.1, +7/+14 mm** · 300: 5.6, +13 mm |
| Damping `kd` | 0: rms 8 / max 20 mm, servo 56–92 continuously oscillating · **0.04: rms 2.4 / max 6** · 0.08: rms 3.8–6 but a fast oscillation after corner 1, −31 mm at corner 2, in both runs |
| Servo anticipation (predict pose 0.04 s ahead) | Sim: std 4.9 → 4.0. Car: std 12.4, lateral 53 mm, "the servo dances". Rejected. |
| Wall correction at lookahead 200 | gain 0.15 / max 10 mm: std 9.1 · gain 0.3: std 10.1. The jumps of the believed pose made pure pursuit jerk. |
| Wall correction at lookahead 275 (CW) | off: 35 mm, std 4.3 · 0.15 / 3 mm: 25 mm, 5.0 · **0.3 / 5 mm: 12 mm, 5.5** · 0.4 / 7 mm: 16 mm, 5.4 |
| Heading correction from the walls (`pared 3`) | Noisy (±0.5° per straight) and, against the interior box (whose real position is uncertain), it drifted −8° in 3 laps. Not used. |
| PWM in curves | 230: 28.7 s · 255: 28.7 s, right/left straights 3 cm out / 6 cm in. No gain. |
| Servo center | 77: +5 mm left bias · **75** · 76: no improvement |
| Finish | Hard brake at 255: 25–29 mm skid · PWM ramp over 300 mm: still ~810 mm/s at the line · **step to 150 after the last turn: 7–12 mm** |

## 6. Rejected hypotheses

| ID | Hypothesis | Test | Result |
|---|---|---|---|
| R-1 | The encoder over-counts in curves (rigid rear axle, no differential), so the car cuts each corner by 5–9 cm | Fitted on the 180 run: 15 % at full lock fit the wall data (41 → 10 mm). With `encCurva` 0.15 / 0.08 on the car. | The encoder counted 22.6–22.8 m in 3 laps instead of 22.24–22.31 m: the car drove 30–50 cm too far and ended ahead of the start. Removed. |
| R-2 | The BNO over-reads 0.4 % (`bnoEscala` 0.996) | Same fit; then 3-lap runs with 0.996 and 0.994 | The inward error grew lap after lap (~0.36° per corner). The hand-rotation test (CalibFox) had measured 0.9998. Back to 1.0. |
| R-3 | The corner cut is speed-dependent (skid, BNO latency) | Same lap at 180 and 255, both ToF | Corner-exit offset +63 vs +67 mm: not speed-dependent. |
| R-4 | The interior box is rotated | L (interior wall) and R (outer wall) sloped the same way on every straight | It was the car (confirmed by the user). The interior box is still off by up to ~5 cm on one side, so only the outer walls are used. |

## 7. Measured calibration values

| Quantity | Value | How |
|---|---|---|
| Encoder | 5.45 counts/mm | 4031 counts in 740 mm (2026-10-19); over 22.3 m it agreed with the tape within ~5 cm |
| Turning radius at the rear axle (mat, PWM 150) | right 123 mm (servo 1), left 134 mm (servo 152) | CalibFox `steer --auto`, 3 repetitions |
| Servo lag | ~0.12 s (first order) | Fitted to runs at 210/240/255 |
| Speed | 805 mm/s straight, 776 mm/s in curves at 255 | Odometry, run 143417 |
| Braking | 7–12 mm from PWM 150; 23–29 mm from 255 | `# fin ... frenando` |
| Lateral ToF (Short, ROI 8, 50 ms) | ±5 mm up to 60 cm on the black wall | Static test 2026-10-21 |
| Mat | 2975 mm (one direction measured), corridor 991.7 mm | Tape |

## 8. Open issues

* **Right ToF drops out intermittently** (dead in 4 of 5 runs on one afternoon, then fine). **BNO connector** came loose twice when the battery was changed (`BNO:NO` at boot). Both wait for the RevB PCB.
* **Battery.** The same settings gave 12 mm on a fresh battery and 28 mm on a low one, and the lap time grew 27.3 → 29.2 s during the day. Compare settings only on a charged battery.
* **Obstacle routes start beyond ToF range.** The start chosen on the longest straight part of the route was 80 cm from the outer wall, so `salidaPared` could not correct it and the start pose relied on the previous run's stop.
* **Remaining 1–2 cm inward bias** with walls on. It is at the level of run-to-run variation.
* Things learned here still have to go into the race firmware (`PurePursuit.ino` route mode).

## 9. Log index

| Logs | What |
|---|---|
| `ruta_recta_*`, `ruta_s_*` 111616–112029 | First straight and S tests |
| `ruta_vuelta_*` 113619–121728 | First laps, center 77 → 75, `kd` sweep |
| `ruta_vuelta_*` 124325–131742 | First runs with ToF logging; corner-cut diagnosis (180 vs 255) |
| `ruta_vuelta_*` 134603–141910 | Encoder / BNO corrections (rejected), wall-correction tuning at lookahead 200 |
| `ruta_vuelta_*` 142022–142917 | Anticipation (rejected), lookahead sweep |
| `ruta_vuelta_*` 143324–150923 | Center 76, curves at 255, CW, wall correction at lookahead 275, fresh battery |
| `ruta_obstaculos_*` 152412–152651 | Obstacle course, 3 runs |
