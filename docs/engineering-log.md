# Engineering Log — FoxRobotics, WRO 2026 Future Engineers

This log records the decisions, failures and milestones that changed the robot. Every entry points to evidence: a recorded run (`orillasNNN` = run number NNN, a video plus the Pi's journal), a commit, or a measurement. We add entries; we do not rewrite them. When a later change replaces an entry, we mark it *Superseded* and keep it.

**Status legend**

| Status | Meaning |
|---|---|
| Validated | The fix was tested on the track and the acceptance check passed |
| Validated (replay) | Validated by replaying recorded runs through the new code, not yet on the track |
| Mitigated | The symptom is controlled, but the root cause is still there |
| Under investigation | The cause is known or suspected; the fix is not done |
| Rejected | Tried and removed, with the reason |

**How we find problems.** Every run records the camera and bird's-eye view to `videos_orillas/orillasNNN.avi`, and the Pi logs every message sent to the ESP32 and every ACK it gets back (`journalctl`). [`scripts/reporte_run.py`](../scripts/reporte_run.py) turns the journal into a per-corner report: state changes, heading, distances and maneuver choices. Most entries below were diagnosed by matching the video frame to the log line. See [testing.md](testing.md).

---

## 1. Decision log — "we chose X instead of Y because…"

| ID | Decision | Alternatives considered | Why (data) | Status |
|---|---|---|---|---|
| DEC-01 | Split the work: vision on a Raspberry Pi, real-time control on an ESP32 | Everything on one board | On the Pi, one 640×480 HSV pass takes 8–15 ms. Running it next to servo PWM and ultrasonic timing makes the control loop jittery. The ESP32 keeps driving on the gyro if the Pi stalls for > 800 ms. | Validated |
| DEC-02 | HC-SR04 ultrasonics for walls (v1) | VL53L0X ToF | The VL53L0X returned out-of-range on the black walls at 300 mm in most readings. Revisited in DEC-12. | Superseded by DEC-12 |
| DEC-03 | Pure Pursuit on a bird's-eye view | PID on the sign's x-offset in the camera image | In the raw image, pixel offset ≠ distance (a near and a far sign need different gains). In the bird's-eye view 1 px ≈ 2 mm, and Pure Pursuit uses the car's real geometry, so one controller handles straights, dodges and corner exits. | Validated |
| DEC-04 | On a clean straight, steer with the **gyro**. Vision steers only while dodging. | Vision steers all the time | With vision always steering, each small dodge left the chassis 20–30° crooked and the car drifted into a wall (runs 679–690). After the change, run 700 was the first clean 12-corner run. | Validated (run 700) |
| DEC-05 | Obstacle Challenge corners: **stop, then forward arc or reverse pivot**, chosen by the distance to the outer wall | Continuous turn (as in the Open Challenge) | The continuous turn kept hitting or skipping the sign at the mouth of the next straight. Two weeks before the national final we chose points over time: the score ranks first, time only breaks ties. | Validated (national final) |
| DEC-06 | "Sign passed" trigger from **measured state**: real dodge happened + nothing ahead in memory + chassis turned ≥ 15° | Dead-reckoned sign position from where it was first seen | We tried the dead-reckoned version four times (angle → geometric → geometric v2 → speed-scale tuning). Each was overfit to one recorded run. The geometric one crashed on the track (`d906e97`, reverted in `2a92e0d`). | Validated |
| DEC-07 | `RECUPERANDO` cannot be interrupted by a newly seen sign | Let a new sign take over immediately | While the chassis is still turned, the "new" sign is usually the one just passed, or one from the next straight. Interrupting sent the car towards it. | Validated |
| DEC-08 | Venue robustness through **floor-referenced color correction plus ratio tests** (blue/red, green/red, saturation) | Widen the HSV ranges, or raise camera gain | Hue near red wraps around and shifts tens of units with lighting, while the white floor barely changes. Ratios between channels stay stable under different lighting. Widening the hue range made the pink lot walls and the orange tape read as red (ERR-VIS-02, -04, -06). | Validated (replay over 6 runs) |
| DEC-09 | Keep the Pi 4 at ~14 fps and rescale every per-frame constant | Process every 2nd frame at ~7 fps | Lower latency gave smoother dodges. Constants written as "N frames" doubled their effect at 2× fps; we rescaled 8 of them so they keep the same meaning in seconds (`03df4ae`). | Validated |
| DEC-10 | Upgrade to a **Raspberry Pi 5 (16 GB) + Active Cooler** | Optimize the Pi 4 further, or move to Picamera2 capture | Profiling showed capture was only ~5 ms of a ~65–77 ms frame; vision and path planning were the cost, and the Pi 4 throttled at 84 °C. A faster capture method would gain ≤ 5–10%. The Pi 5 runs the same pipeline at 40–41.5 fps at 52 °C (replayed run 1190). | Under test (port in progress) |
| DEC-11 | Fix the steering: rack joints 43 → 47 mm, tie rods 18.13 mm, pinion 14 → 19 teeth | New knuckles with a shorter arm; move the rack back 6 mm | Linkage simulation matched the CAD within ~1°. Moving the rack meant reprinting the chassis, and a shorter arm risked the linkage locking up. The bigger pinion + new spacing gives 53.1° / 37.0° (~106% Ackermann) by reprinting only the rack, two linkages and the pinion. | Validated in CAD; track test pending |
| DEC-12 | Add 4 × VL53L1X ToF as **near-field** sensors, keep the HC-SR04 | LiDAR; multi-zone VL53L8CX; ultrasonics only | The front ultrasonic produced phantom echoes (ERR-SEN-01). Teams that tested ToF on the WRO black wall report ~80–100 cm usable range, with L1X ≈ L8CX. The L8CX had initialization problems for another team. The low-cost LiDAR another team tested refreshed only ~5 scans/s, too slow at our speed. | Under test (bench) |
| DEC-13 | **Not** adding a magnetometer or optical-flow sensor | — | Magnetometer: the motor and battery fields next to it distort it. Optical flow: the mat is mostly white with little texture to track. | Rejected |

---

## 2. Failure and incident log

### Power

#### ERR-PWR-01 — Pi 4 undervoltage halved the frame rate
- **Observed:** August 2026. `vcgencmd get_throttled` = `0x50005` (undervoltage + throttling), CPU at ~600 MHz, vision at ~7 fps.
- **Cause:** ~0.3 V drop on the 5 V rail from feeding the Pi through Dupont jumper wires.
- **Fix:** Replaced them with 18–22 AWG wire. `get_throttled` = `0x0`, CPU at 1.8 GHz, ~14 fps (2026-08-28).
- **Lesson:** "7 fps is normal" was never true; the Pi was underpowered. `get_throttled` is now the first check of every session. The v2 power plan gives the Pi 5 its own regulator.
- **Status:** Validated.

#### ERR-PWR-02 — Power cut right after `git pull` left files empty
- **Observed:** 2026-09-12 and 2026-09-15. After an update, the runtime service restarted every 2 s without running anything.
- **Cause:** The battery was unplugged seconds after `git pull`. The files had not yet been written to the SD card, so they were left at 0 bytes. On 09-15 some of git's internal object files were empty too.
- **Fix:** Restored the empty files from the repository. Added a rule to the update procedure: `git pull && sync` before unplugging.
- **Status:** Mitigated (procedure).

#### ERR-PWR-03 — A fresh battery over-rotates the dodges
- **Observed:** runs 842 (bad) vs 834 (good), 2026-09-10/11, same code.
- **Data:** Heading swing in red dodges was −58…−82° in 842 vs −33…−54° in 834. The corner pivot, which runs at a fixed PWM with no Pi input, rotated 2.62°/frame vs 2.23–2.28: ~17% more motor power. The car left corners ~5 cm closer to the outer wall, and every following dodge started from the wrong place.
- **Good window:** 11.70–12.15 V on the 3S pack.
- **Mitigation:** Run inside the window (discharge a fresh pack first). **Proposed fix:** measure battery voltage on an ESP32 analog pin and scale the motor PWM by V_ref / V_batt.
- **Status:** Mitigated (procedure).

### Vision

#### ERR-VIS-01 — Camera blackout mid-run
- **Observed:** run 860. The car never turned and crashed. Run 859 had 8% black frames, run 861 had 2%.
- **Diagnosis:** We decoded every recorded bird's-eye frame and measured the 99th-percentile brightness. In run 860, 48% of frames were blind, including **19 s in a row** of full-frame color noise from the camera interface. The Pi kept processing frames; the camera was what failed. File size can't detect it: the noise frames weigh the same as good ones.
- **Fix:** Before arming, the runtime requires real frames (99th-percentile brightness ≥ 60). It reopens the camera if it sees 15 black frames in a row or no new frame (`CAM_CHECK_*`, "Feat: Camera check up", 2026-09-14). The status LED stays off until the camera passes, so a bad camera is caught before the start, not during the run. Commit `486cc4c`.
- **Status:** Mitigated. A blackout *during* a run is still not handled.

#### ERR-VIS-02 — Pink parking-lot wall detected as a red sign
- **Observed:** run 808, 2026-09-09, on the straights next to the parking lot. The car dodged a sign that wasn't there.
- **Diagnosis:** The dark part of the lot wall reads hue ≈ 3, inside the red range. The fixed camera white balance (blue gain 1.5×) pushes magenta towards red. A real sign is a blob that grows as the car approaches; the wall was "passed" straight ahead, with almost no turn.
- **Fix:** Added a blue/red ratio test on each blob (`RED_BR_MAX = 0.42`): real red signs measure 0.11–0.35, the pink wall 0.45–0.60. A ratio doesn't shift with lighting color the way hue does. Commit `e3784a8` (2026-09-16).
- **Status:** Validated.

#### ERR-VIS-03 — At a new venue, red signs were not detected at all
- **Observed:** run 1013, 2026-09-16. The car did not dodge the red sign.
- **Diagnosis:** Under that lighting the red sign measured hue 169–178. Our range covered ≤ 5 and ≥ 177, so a large sign 20 cm away produced zero detections. Old runs showed the sign at 169–171 too: we had always caught only its edge, which is why red was always detected late.
- **Fix:** Red range widened to 168–179, brightness limit raised. The pink-wall protection moved from the hue limit to the ratio test of ERR-VIS-02 (`e3784a8`).
- **Status:** Validated. This led to `calibra_luz.py`, a venue calibration tool for the practice time.

#### ERR-VIS-04 — In a darker room the orange line was read as red
- **Observed:** run 900. Orange line not detected; red signs only detected when close.
- **Diagnosis:** Floor brightness 212 → 184. The tape's hue fell from 9 to 4, into the red range (orange starts at 7). Only 17% of tape pixels still passed as orange.
- **Fix:** `color_corr.py` rescales the image channels so the white floor matches a reference color. On the same recorded run: tape detected as orange 17% → 44%, tape detected as red 3% → 0%. Cost ~1.9 ms/frame. The brightness gain is capped, so real signs don't get pushed out of their range. Commit `7bd353f`.
- **Status:** Validated (replay), then raced.

#### ERR-VIS-05 — False orange line in the middle of a straight
- **Observed:** runs 818 and 820. The car stopped dodging a green sign halfway, because the "line" made the sign look like it belonged to the next straight.
- **Causes:** (1) A stray pixel in the same column as the real line made it look 170 px closer. (2) Brown marks drawn on the test floor have the same hue as the tape but much lower saturation: floor marks have S 86–112, real tape S 140–198.
- **Fix:** The line must be a run of ≥ 3 neighboring columns, and must contain ≥ 8 strongly saturated "core" pixels (`LINE_CORE_HSV`). Replay of run 818: the false window disappears, and all 13 real corners still detect their line before the turn. Commit `283addd`.
- **Status:** Validated.

#### ERR-VIS-06 — Strong light: tape read as red, pale floor read as green
- **Observed:** run 1190, 2026-09-22 (after the national final).
- **Data:** Orange tape matched red by hue but had green/red = 0.38–0.46; a real red sign has 0.05–0.13. Washed-out floor matched green by hue but had saturation 37–44; a real green sign has 120–160.
- **Fix:** Two more lighting-independent tests per blob: `RED_GR_MAX = 0.28` and `GREEN_S_MIN_MED = 70`. Replay: run 1190 red detections 34 → 19 and green 164 → 65. Every rejected crop was checked by eye and was false. Runs 1005, 1064 and 1150 were unchanged. Commit `96145ba`.
- **Status:** Validated (replay over 6 runs).

#### ERR-VIS-07 — Camera tilted up caused over-steering
- **Observed:** 2026-09-09, 4 runs. The car turned too hard and too early on one red sign, sometimes pushing itself into the inner wall.
- **Cause:** The camera mount had tilted ~8° up. That inflated lateral distances in the bird's-eye view, so Pure Pursuit asked for more steering than needed and saturated (+41°).
- **Fix:** Re-leveled the camera. Peak steering command 0.68 → 0.35, no saturation, and the wall-protection override never fired again.
- **Lesson:** The bird's-eye calibration is only valid for one camera angle. The v2 camera mount must be rigid, and the calibration is re-checked after any impact.
- **Status:** Validated.

### Sensors

#### ERR-SEN-01 — Phantom echoes from the front ultrasonic
- **Observed:** run 1047 (2026-09-16) and 2026-09-17, Open Challenge. The car slowed down in the middle of straights.
- **Data:** The front distance fell under 90 cm in ~20% of straight-line frames, with impossible jumps such as 199 → 65 → 179 cm. On 09-17 the phantoms appeared almost only while the car ran 24–28 cm from the side wall.
- **Tried:** Firing one sonar per loop and shutting off the outer sonar. The run got worse, and we reverted it completely.
- **Hypothesis:** The wide ultrasonic cone grazes the side wall.
- **Next:** Tilt or raise the front sonar, and add the VL53L1X (DEC-12).
- **Status:** Under investigation.

### Navigation

#### ERR-NAV-01 — Drift towards a wall after small dodges
- **Observed:** runs 679–690 (2026-09-05).
- **Cause:** Small dodges never turned the car enough to trigger the "passed" event, so the chassis stayed 20–30° crooked and the following wall control could not recover.
- **Fix:** DEC-04 (gyro heading-hold by default). Also added a proportional wall-protection override (`wallPanic`, below 18 cm), and limited the corner re-referencing of the heading target so it cannot chase a runaway angle.
- **Status:** Validated (run 700 = first clean 12-corner run).

#### ERR-NAV-02 — Two red signs on both sides of a corner merged into one
- **Observed:** run 954, 2026-09-15.
- **Cause:** While dodging a red sign, the red of the next straight appeared 69 px from where the memory had moved the old one. They were merged, and the new sign inherited "ours". The car dodged it at full lock and hit the wall.
- **Fix:** A remembered sign cannot "reappear" more than 40 px to the side **and** more than 15 px further ahead than it could physically be. This check can only prevent merges, never create them. Commit `8bfe4d0`.
- **Validation:** Replayed 16 runs (37,774 frames) through the memory with and without the fix. Only 8 frames differ: the fix itself in run 954, and one 1-frame shift in run 933.
- **Status:** Validated (replay).
- **Found on the way:** The memory rotates the map with the wrong sign: 10.9 px/frame error vs 3.9 with the right sign. It is not fixed yet, because three other components were tuned around it.

#### ERR-NAV-03 — The corner maneuver changed its mind at the last moment
- **Observed:** run 1188, corner 4 (2026-09-22). Probably the same failure seen at the national final.
- **Cause:** The distance to the outer wall sat at 27–29 cm, exactly on the 28 cm threshold between "forward arc" and "reverse pivot". The approach decided *reverse*, the final check read 27 and chose *forward* at 15 cm from the wall, and the arc jammed.
- **Fix:** Threshold 20 cm with ±6 cm hysteresis. One shared decision for the approach and the maneuver. A forward arc with less than 35 cm of room ahead is forced to reverse. The ACK now reports `hug=`. Commit `96145ba`.
- **Status:** In repository, not yet validated on the track.

#### ERR-NAV-04 — Systematic heading offset after every corner
- **Observed:** runs 853–856, 37 straights.
- **Data:** Right after each maneuver the heading was −7.5° ± 0.9°: a constant bias of 5° we add on purpose plus ~2.5° of under-rotation. That part can be calibrated. The drift that varies is built **inside** the straights, by the dodges.
- **Mitigation:** The parking sequence re-measures the heading against the wall (`PARK_TRIM_*`).
- **Proper fix:** An absolute heading reference from the walls, which is one of the goals of the side ToF sensors.
- **Status:** Mitigated.

#### ERR-NAV-05 — Green-then-red slalom: the red is grazed "sometimes"
- **Observed:** runs 843–845 (2026-09-11). Run 845 grazed the red in lap 2 and pushed it out of its area in lap 3. Runs 843 and 844 passed cleanly.
- **Data:** The difference is millimeters. When the green ends up on the "wrong" side of the car's line after the corner, the green dodge grows from 6–18° to 19–56°. Then the red is seen only 20 cm ahead and needs a 58–74° dodge.
- **Lever:** The red is already visible from the corner exit as a 500–850 px blob, but the detector ignores blobs under 1000 px.
- **Status:** Under investigation.

### Software process

#### ERR-SW-01 — The geometric "sign passed" trigger crashed on the track
- **What happened:** On 2026-08-28 we computed the moment the sign is passed from where it was first seen plus dead reckoning. It worked in a synthetic simulation, then crashed on the track. We reverted it the same day (`d906e97` → `2a92e0d`).
- **Lesson:** Never replace a trusted behavior with one validated only in simulation. New behaviors are now first added in log-only mode next to the old one (`[RECUP]`, `[MTURN]` log lines), compared on recorded runs, and only then given control.
- **Status:** Rejected → replaced by DEC-06.

#### ERR-SW-02 — Doubling the frame rate made the car "more aggressive"
- **Observed:** 2026-08-28, after ERR-PWR-01 doubled the fps.
- **Cause:** Constants counted in frames now acted twice as fast: steering slew, memory decay, line persistence, post-turn recovery, and the duration of the "passed" pulse.
- **Fix:** All rescaled to keep the same value in seconds (`03df4ae`).
- **Lesson:** This applies again on the Pi 5 (40 fps). See [testing.md](testing.md).
- **Status:** Validated, and open again for v2.

#### ERR-SW-03 — The remote desktop client overheated the Pi 4
- **Observed:** 2026-09-21. The remote screen client used 55–60% of a CPU core, and the Pi reached 84 °C with its soft temperature limit active (`throttled=0xe0008`).
- **Effect:** 16.1–16.8 fps with the client connected vs 18.1 without.
- **Mitigation:** Disconnect it while tuning or measuring fps. Wireless must be off during official rounds anyway (rule 11.10).
- **Fix:** The Pi 5 with the Active Cooler.
- **Status:** Mitigated.

---

## 3. Milestones

| ID | Date | Run | Result |
|---|---|---|---|
| MIL-01 | 2026-09-05 | 700 | First clean 12-corner Obstacle Challenge run with stop-and-turn corners |
| MIL-02 | 2026-09-16 | 1025 | First perfect parallel park; its constants became the reference set |
| MIL-03 | 2026-09-16 | 1047 | Open Challenge 12/12 corners (with the phantom slow-downs of ERR-SEN-01) |
| MIL-04 | September 2026 | 1064 | Full clean Obstacle Challenge: start from the lot, 12 corners, parallel park (heading error 0.9° at the end) |
| MIL-05 | 2026-09-19 | — | **1st place, Mexico national final**, with release [`v1.0-nacional`](releases.md) |
| MIL-06 | 2026-10-06 | 1190 (replay) | Unmodified pipeline at 40–41.5 fps on the Raspberry Pi 5 |
