# Testing and Validation

This document describes how we test the robot, what we measure, and the results we have so far. Results are tied to a code version (commit) so they can be reproduced. A failed run is kept in the record, because it tells us what to change next. Decisions and failures found through these tests are in the README failure log ([§5.5](../README.md#55-failure--incident-log), ERR-01 to ERR-09) and in the [engineering log](engineering-log.md) (ERR-10 onward).

## 1. What every run produces

Every run is recorded automatically. Nobody has to remember to start a recording.

| Evidence | Where | What it is used for |
|---|---|---|
| Video `orillasNNN.avi` | `~/FoxRobotics/videos_orillas/` on the Pi | Camera and bird's-eye view side by side, with the debug overlay: detections, path, steering, state, stage timings. MJPG format, so it still plays if the run is cut off. |
| Pi journal | `journalctl -u wro-runtime.service` | Every message sent to the ESP32 and every ACK received: heading, state, the three ultrasonic distances, maneuver choice, parking phase. It also logs the decision traces `[RECUP]`, `[LINEA]`, `[MEMREJ]` and `[COLOR]`. Persistent across reboots on the Pi 5. |
| Run report | `python scripts/reporte_run.py --pi` | Turns the journal of the last run into a per-corner summary: states, heading at each corner, forward/reverse maneuver, distances, parking phases. Optional HTML output. |

Run numbers (`NNN`) are global and increase with every run, so a run number identifies one video plus one journal section.

## 2. Test workflow

1. **Record the configuration.** Commit hash on the Pi (`git log -1`), challenge type and race length in the firmware (`rondaObstaculos`, `TURNS_PER_RACE`), battery voltage, direction, and sign layout.
2. **Pre-run checks** (section 4). A run that fails a check doesn't count.
3. **Run without touching anything.** We never change parameters during a run.
4. **Measure.** Score by the official table, time, interventions, contacts with signs or walls, signs moved, corners completed, parking result, and the voltage after the run.
5. **Diagnose every failure from evidence.** Find the frame in the video, then the matching log line, and name the cause before changing code. A fix without an identified cause is not merged.
6. **Change one thing at a time.** Then repeat the same layout in both directions. When a change touches perception or memory, we first **replay** recorded runs through the old and the new code (section 5) and compare frame by frame.
7. **New behaviors start in log-only mode.** A new detector or trigger first only logs what it *would* do, next to the existing one. It gets control after the logs show it is right. We adopted this rule after ERR-25.

## 3. Metrics

| Metric | Why |
|---|---|
| Score and time per round | What the competition ranks |
| Corners completed out of 12, and laps | Main reliability measure |
| Signs touched or moved | Moving a sign out of its area costs points; we count every contact |
| Heading error at the end of each maneuver (`ang`) | Measures drift and maneuver accuracy (−7.5° ± 0.9° bias found this way, ERR-23) |
| Heading swing during a dodge | Detects over-rotation (battery ERR-11, camera tilt ERR-18) |
| Closest wall distance per straight (`dL`, `dR`) | Margin before contact |
| Detections per run, real vs false | Checked by eye on cropped detections when tuning vision |
| Loop rate (fps) and `get_throttled` | Compute and power health |

## 4. Pre-run checklist

- [ ] Battery between **11.70 and 12.15 V** (ERR-11). Discharge a fresh charge first.
- [ ] `vcgencmd get_throttled` = `0x0` (ERR-08).
- [ ] Firmware flashed with `TURNS_PER_RACE = 12` and the right `rondaObstaculos` value for the round. Test builds use 4 turns.
- [ ] Status LED on: the camera passed its brightness check (ERR-12).
- [ ] Camera level and lens seated (ERR-18; run 840 had a loose, blurry lens).
- [ ] At a new venue: run `calibra_luz.py` during practice and check red, green, orange and magenta on real frames (ERR-14).
- [ ] Remote desktop, Wi-Fi, Tailscale and Claude Remote Control **off** for official rounds (rule 11.10).
- [ ] After any `git pull` on the Pi: `sync` before unplugging (ERR-10).

## 5. Replay testing

Most vision and memory changes are validated before touching the track, by feeding recorded runs through the new code:

- **Detection replay:** run `vision.detect_on` over every frame of a recorded `.avi` (the camera half of the overlay is exactly what the detector saw). Count detections before and after, and inspect the rejected crops. Used for ERR-15 and ERR-17 (6 runs).
- **Memory replay:** feed the real `ObstacleMemory` with the logged detections and ACKs. It reproduces the logged memory state at 93–100% of frames. Used for ERR-21 (16 runs, 37,774 frames).
- **Performance replay:** run the real `runtime_nuevo.py` with a recorded video as the camera and no ESP32. Used for the Raspberry Pi 5 port (40–41.5 fps on run 1190).

Replay can't test anything that depends on the car moving differently, such as a new steering decision. Those changes need track runs.

## 6. Results

### 6.1 Reference runs (v1, before the national final)

| Run | Date | Round | Dir. | Result | Notes |
|---|---|---|---|---|---|
| 700 | 2026-09-05 | Obstacle | — | 12/12 corners, clean | First clean run after the gyro-default change (DEC-04) |
| 834 | 2026-09-10 | Obstacle | CCW | Clean dodges | Reference for the battery window (ERR-11) |
| 842 | 2026-09-11 | Obstacle | CCW | Over-rotated dodges | Fresh battery; same code as 834 |
| 845 | 2026-09-11 | Obstacle | — | Red sign grazed (lap 2) and moved (lap 3) | ERR-24 |
| 860 | 2026-09-12 | Obstacle | — | Crash, no turn | Camera blind for 19 s (ERR-12) |
| 1013 | 2026-09-16 | Obstacle | — | Red sign not dodged | New venue lighting (ERR-14) |
| 1025 | 2026-09-16 | Obstacle | — | Perfect parallel park | Reference parking constants |
| 1047 | 2026-09-16 | Open | — | 12/12 corners | Phantom front echoes in ~20% of straight frames (ERR-19) |
| 1064 | 2026-09 | Obstacle | CCW | **Full clean round + parallel park** | Start from the lot, 12 corners, parked with 0.9° heading error |
| National final | 2026-09-19 | Both | — | **1st place** | Release `v1.0-nacional` |

### 6.2 Validation matrix for v2 (international final)

For each configuration we run both directions and three start sections. A configuration passes only with **3 consecutive clean runs**.

| Run | Round | Dir. | Start section | Layout | Score | Time | Contacts | Interventions | Battery (V) | Commit | Status |
|---|---|---|---|---|---:|---:|---:|---:|---:|---|---|
| OPEN-01 | Open | CW | | | | | | | | | To run |
| OPEN-02 | Open | CCW | | | | | | | | | To run |
| OBS-01 | Obstacle | CW | | | | | | | | | To run |
| OBS-02 | Obstacle | CCW | | | | | | | | | To run |

### 6.3 Sensor-fault behavior

| Fault | What the robot does | Evidence | Status |
|---|---|---|---|
| Pi silent for > 800 ms | The ESP32 falls back to wall-following + gyro and keeps driving | Firmware `piTimeoutMs` | In code, not fault-injected |
| Camera black or frozen before the start | The camera is reopened; the LED stays off and the start is blocked until real frames arrive | Run 860 analysis, `486cc4c` | Validated |
| Camera black **during** a run | Not detected; the car drives on stale vision | Run 860 | Known limitation |
| `RECUPERANDO` never reaches its heading | It times out, then continues | Firmware `timedOut` | In code |
| Front ultrasonic phantom echo | Corner trigger needs consecutive readings, and a minimum time between maneuvers | ERR-19 | Mitigated |
| Car too close to a side wall | Proportional push away from it (`wallPanic`, below 18 cm) | ERR-20 | Validated |

## 7. Known limitations

- Speed is open-loop: the motor has no encoder, so maneuvers depend on battery voltage (ERR-11). An encoder motor is planned for v2.
- Heading drifts within straights through the dodges, and is only corrected against a wall during parking (ERR-23).
- The obstacle memory rotates its map with the wrong sign (ERR-21). Other parts were tuned around it, so we left it until it can be re-validated.
- The bird's-eye calibration assumes a fixed camera angle. Any impact on the mount requires a check (ERR-18).
- On the Raspberry Pi 5 the main loop runs faster than the camera (40 vs 30 fps) and reprocesses repeated frames. Before racing it, the loop rate must be capped, or the per-frame constants rescaled (ERR-26).
