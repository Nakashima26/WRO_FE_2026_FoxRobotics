# FoxRobotics simulation and MathWorks modeling

## The engineering decisions behind the models

The WRO obstacle round requires a car that can pass pillars on the correct side, complete three laps and park. The Raspberry Pi builds a map from camera observations and plans a route. The ESP32 follows the route using odometry, a Hall encoder, an IMU and ToF measurements. The original Python/C++ twin runs the Pi software and compiled ESP32 firmware together, with simulated sensors and randomized fields. [Vehicle architecture](../../README.md#31-two-controllers-one-link) · [Twin and validation method](../../README.md#39-digital-twin--software-in-the-loop-simulator)

| Decision | Evidence from the original twin and the car |
| --- | --- |
| Select the N20 transmission | We evaluated 1.5 : 1 and 1.25 : 1 configurations in the twin. The selected 1.25 : 1 gearing gave the N20 more speed on straights while the route profile limited speed in curves. On the mat the car reached about 0.8 m/s at full PWM. |
| Upgrade to the N30 | The N20's measured straight-line speed capped further gains. We kept the 1.25 : 1 gears and tested the N30, measuring about 1.5 m/s on the mat and recalibrating the motor and encoder. High-speed short braking revealed a new odometry error from wheel lock. |
| Adopt and refine dynamic programming (DP) | Early known-map runs finished 1 of 21 with the earlier route builder and 69 of 69 with the DP route. These are historical test totals. For subsequent changes we used paired comparisons across 42 layouts × 5 noise seeds, or 210 matched scenarios per variant. We screened for new failures before comparing run times and exported accepted configurations to the Pi and ESP32. |

[Drivetrain measurements](../../README.md#12-drivetrain--gear-ratio-chosen-from-measured-speed) · [DP planner](../../README.md#374-route-planner--dynamic-programming) · [Paired benchmark and deployment](../../README.md#392-how-we-use-it--the-benchmark-rule)

We subsequently used Simulink to model braking distances independently and check whether the twin's vehicle dynamics, encoder and range geometry were reproduced step by step. The block model makes it possible to change braking and delay inputs and inspect their effect on curve-entry distance. The drivetrain selection and DP development above took place in the original twin and on the car.

## Models and results

- [N30 braking model](evidence/fox_n30_braking.slx) · [MATLAB generator](run_braking_study.m) · [bench comparison](evidence/bench_comparison.csv) · [curve-entry sweep](evidence/curve_entry_sweep.csv).
- [Twin plant replay model](evidence/fox_twin_replay.slx) · [MATLAB generator](run_twin_replay.m) · [sample-by-sample reference exporter](export_twin_case.py). Build the MEX with the generator before running this model on a new platform.

![Simulink curve-entry distance for coasting and commanded deceleration](evidence/curve_entry.png)

## Design question

How far before a corner should FoxRobotics begin reducing speed, and what happens to encoder odometry if the N30 wheels lock? The Raspberry Pi generates a speed profile from the mapped route. The ESP32 tracks that profile using the encoder. A short brake can stop the wheel before the chassis stops, so the controller can underestimate distance traveled.

We selected Simulink for a focused longitudinal model that exposes speed and displacement as separate states. It is a companion to the full-car software-in-the-loop twin. This study uses a constant deceleration input, a speed integrator limited at the target speed, and a distance integrator. The runner uses the model for both the bench comparison and a braking-distance sweep. It also checks every simulation against the constant-deceleration equation. The delay term assumes speed remains constant during the specified delay.

## Data and reference calculations

[Bench notes](../../src/RASPI/cam/pure_pursuit/twin/calib/medidas_20261117/NOTAS.md) record tests on the N30 at 11.88 V. [Input CSV](n30_measurements.csv) transcribes the five forward stop observations. The N30 observations are summaries. Encoder stop distance is used only where the test supports it. The tape measurement at high-speed short braking is approximate.

| Bench case | Role | Recorded distance | Analytical model |
| --- | --- | ---: | ---: |
| Coast from 1409 mm/s | Fit constant deceleration of 1051.5 mm/s² | 944 mm encoder | 944 mm by construction |
| Coast from 803 mm/s | Separate observation reserved from that fit | 329 mm encoder | 306.6 mm (22.4 mm low, 6.8%) |
| Short brake from 500 mm/s | Check against the existing low-speed twin parameter of 5000 mm/s² | 26 mm encoder | 25 mm |
| Short brake from 1066 mm/s | Wheel lock observed | 64 mm encoder | Outside the model |
| Short brake from 1502 mm/s | Approximate encoder undercount during wheel lock | 88 mm encoder, about 240 mm with tape | About 152 mm missed |

At an approach speed of 1500 mm/s, a target of 500 mm/s and an assumed delay of 100 ms, the analytical distances are approximately **1101 mm coasting** and **483 mm at the profile's nominal 3000 mm/s²**. This recaps why the existing [race profile](../../src/RASPI/cam/pure_pursuit/carrera.json) commands braking ahead of the curve and the [ESP32 controller](../../src/ESP32/PurePursuitCarrera/PurePursuitCarrera.ino) tracks the requested speed. The observed lockup supports avoiding short braking at high speed where the route follower relies on odometry. The 3000 mm/s² rate is a planning input. The ability to track it on a real approach remains to be measured.

The model addresses straight-line travel at one bench voltage. A single held-out coast observation cannot establish repeatability. Tire slip, battery variation, steering and controller dynamics remain outside the scenario model. The next physical test should log applied PWM, target and measured speed, and independent chassis displacement on repeated straight approaches, then compare distance at the target speed against the simulated prediction. Record any resulting change to the race profile alongside the test.

With repeated N30 runs at two measured battery voltages, we can use MATLAB Optimization Toolbox (`lsqnonlin`) to fit physically bounded parameters to a subset of runs and validate speed and displacement on the held-out runs. This is a proposed follow-up. The current study uses constant deceleration and the bench comparison above.

## Reproduce

From the repository root in MATLAB R2025b or a compatible release with Simulink:

```matlab
addpath('models/mathworks')
run_braking_study
```

The [runner](run_braking_study.m) writes `output/fox_n30_braking.slx`, `bench_comparison.csv`, `curve_entry_sweep.csv` and `curve_entry.png`. Open the generated model to inspect the integrators and run the default 1500-to-500 mm/s scenario. For a public GitHub repository, the [GitHub Actions workflow](../../.github/workflows/mathworks-braking.yml) uses MathWorks batch licensing and uploads the same files as an artifact. The models and focused braking outputs above are copies from the verified run.

The figures above are analytical reference calculations. The GitHub Actions run reproduced the three bench comparison rows in Simulink. MATLAB Online ran the braking model without a compiler. The plant replay requires a platform-specific MEX built by CI. A reproducible way to build and bundle native dependencies directly from MATLAB Online would make the plant easier to share with judges.

## Digital twin plant replay in Simulink

The replay runs the twin's [C99 vehicle kernel](../../src/RASPI/cam/pure_pursuit/native/src/twin_vehicle.c) at 1 ms in a discrete Simulink plant block. This brings over the forward and reverse motor maps, limits on acceleration and lateral grip, coasting, short braking, wheel lock and reverse wheelspin, servo speed and deadband, steering backlash and lag, and the rear-axle bicycle model. The plant also implements the twin's encoder fractional accumulator, quantization and slip correction. It exports ground-truth yaw rate from the simulated pose.

A separate range block uses the exported WRO wall, pillar and parking-board geometry and all four ToF mounts. It computes deterministic center-ray distances. ToF cone, material and incidence effects, calibration curves, update intervals, status errors and noise still belong to the full Python twin. The camera renderer, collision adjudication, UART timing and the real Raspberry Pi/ESP32 control loop also remain in that twin. The Simulink scenario drives the plant with scripted actuator commands in open loop.

From the repo root, with Python and NumPy installed:

```sh
python models/mathworks/export_twin_case.py models/mathworks/output/twin_case.json --seed 1
```

Then use MATLAB with Simulink and a configured supported C compiler (`mex -setup C`):

```matlab
addpath('models/mathworks')
run_twin_replay('models/mathworks/output/twin_case.json')
```

The runner builds the shared C source as a platform-specific MEX and generates `output/fox_twin_replay.slx`, plus a trajectory and speed plot and per-step CSV. To reopen the generated model in a new session, add both `models/mathworks` and `models/mathworks/output` to the MATLAB path. Re-run the generator if moving to a different platform. For the replay, start with the public repo's GitHub Actions workflow so the Linux runner can compile the C source.

The Python exporter uses independent Python dynamics and encoder code for the reference. The Simulink runner asserts matching timestamps, exact integer encoder counts, and per-channel continuous state and range errors below 1e-6 in each channel's units. This measures replay parity under the same commands and initial conditions. The bench study above addresses agreement with physical measurements. The [verified GitHub Actions run](https://github.com/Nakashima26/WRO_FE_2026_FoxRobotics/actions/runs/45001314032) passed with 1,961 samples on seed 1, exact integer encoder counts, a maximum continuous state error of 6.82e-13 in the logged channel units, and a maximum range error of 6.76e-12 mm. Its artifact contains both models, graphs and CSVs.

## Animated viewer in MATLAB Online

Extract `fox_twin_viewer.zip` from the Actions artifact into a MATLAB Drive folder. In MATLAB Online, open that folder and run `view_twin_replay`. The viewer plays the Simulink poses with the original twin's track artwork, car silhouette, reconstructed camera and BEV. It has play/pause, frame steps and a timeline. It reads images and a manifest, so it does not load the vehicle MEX or rerun the simulation.

To regenerate the viewer after a Simulink run, run `python models/mathworks/render_twin_replay.py <run_dir>` with NumPy, OpenCV and Matplotlib installed. The camera and BEV are rendered with the Python twin from the Simulink poses. The viewer shows the scripted plant replay and its camera geometry. Live autonomous control and the controller's estimated map belong to the original twin.

## Autonomous race playback

`render_twin_race.py` packages a run of the existing Pi and ESP32 software-in-the-loop twin for the same MATLAB viewer. It records the rendered camera, the processed BEV and the chassis pose while the controllers run. The manifest includes the finish reason, lap times and collision data.

```sh
TWIN_NOISE_SEED=1 python models/mathworks/render_twin_race.py models/mathworks/output/fox_twin_race.zip --seed 1 --setup src/RASPI/cam/pure_pursuit/twin/tools/carrera_bench.json
```

The demonstrated finish used the sources and benchmark setup from revision `08f6ccb`, with field seed 1 and noise seed 1. It completed 12 corners with no contacts. Extract the race ZIP into its own MATLAB Drive folder and run `view_twin_replay` there. This is playback of a closed-loop Python/C++ simulation. The controller does not run inside Simulink.

## Award preparation

| Official rubric | Max | Evidence and work before judging |
| --- | ---: | --- |
| Problem context and strategy | 5 | Show how speed planning, ESP32 control and encoder odometry interact in the approach to a curve. |
| Innovation and modeling | 5 | Linked models show the independent braking sweep and the shared twin physics replay. |
| Test and verification | 5 | Linked plots and metrics cover bench agreement and replay parity. Repeated physical curve-entry tests are still needed to validate the planning rate. |
| Code quality and readability | 3 | The models, generators and inputs are linked from the root README. |
| Report quality and product feedback | 2 | The braking study records the MATLAB Online experience and suggests an improvement for native dependency packaging. |

The award is for eligible teams at the International Final. Future Engineers submits the public repository link through the organizer's process. Ask the national organizer whether there is a separate award opt-in, where the submission link goes and what deadline and time zone apply. The general rules require the repository link at least three weeks before competition and the main scoring commit at least two weeks before competition. The organizer announces the exact link deadline. The award documents do not give a separate public deadline.

Access options for this study are GitHub Actions on the public repository, free MATLAB Online Basic with a MathWorks account, and the complimentary WRO competition license requested by the team leader or faculty advisor. An existing school license with Simulink can also run the model. We chose this scope to make a substantive modeling contribution using the actual N30 and preserve the race-tested full-car twin. A MATLAB-only study could meet the announcement's tool wording, but Simulink delivers the model file specifically requested by the rubric. Porting the entire twin would require recreating the hardware and firmware coupling before the port could support equivalent claims.

[Official award announcement](https://wro-association.org/mathworks-modeling-award-returns-to-wro-2026/) · [2026 rubric](https://wro-association.org/wp-content/uploads/MathWorks-Modeling-Award-Scoring-Rubric_20260312.pdf) · [Future Engineers rules §7](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf#page=9) · [WRO software request](https://www.mathworks.com/academia/student-competitions/world-robot-olympiad.html) · [MATLAB Online options](https://www.mathworks.com/products/matlab-online/matlab-online-versions.html)
