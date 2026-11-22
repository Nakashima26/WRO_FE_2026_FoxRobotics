# FoxRobotics N30 braking study

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

## Reproduce

From the repository root in MATLAB R2025b or a compatible release with Simulink:

```matlab
addpath('models/mathworks')
run_braking_study
```

The [runner](run_braking_study.m) writes `output/fox_n30_braking.slx`, `bench_comparison.csv`, `curve_entry_sweep.csv` and `curve_entry.png`. Open the generated model to inspect the integrators and run the default 1500-to-500 mm/s scenario. For a public GitHub repository, the [GitHub Actions workflow](../../.github/workflows/mathworks-braking.yml) uses MathWorks batch licensing and uploads the same files as an artifact. Review a successful run and publish the generated model, graph and metrics at stable repository links for judging.

The figures above are analytical reference calculations. The GitHub Actions run reproduced the three bench comparison rows in Simulink. We will add product feedback after the team has used the model and reviewed the simulation results.

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

## Award preparation

| Official rubric | Max | Evidence and work before judging |
| --- | ---: | --- |
| Problem context and strategy | 5 | Show how speed planning, ESP32 control and encoder odometry interact in the approach to a curve. |
| Innovation and modeling | 5 | Publish both study models and explain how the replay reuses the race twin physics and how the simplified braking study informs curve-entry planning. |
| Test and verification | 5 | Publish the generated plot and metrics. Add repeated independent chassis-distance tests and report the design decision they support. |
| Code quality and readability | 3 | Keep the generator, inputs and reproduction command together. Link the generated model from the root README. |
| Report quality and product feedback | 2 | Add actual Simulink experience and product feedback after execution. |

The award is for eligible teams at the International Final. Future Engineers submits the public repository link through the organizer's process. Ask the national organizer whether there is a separate award opt-in, where the submission link goes and what deadline and time zone apply. The general rules require the repository link at least three weeks before competition and the main scoring commit at least two weeks before competition. The organizer announces the exact link deadline. The award documents do not give a separate public deadline.

Access options for this study are GitHub Actions on the public repository, free MATLAB Online Basic with a MathWorks account, and the complimentary WRO competition license requested by the team leader or faculty advisor. An existing school license with Simulink can also run the model. We chose this scope to make a substantive modeling contribution using the actual N30 and preserve the race-tested full-car twin. A MATLAB-only study could meet the announcement's tool wording, but Simulink delivers the model file specifically requested by the rubric. Porting the entire twin would require recreating the hardware and firmware coupling before the port could support equivalent claims.

[Official award announcement](https://wro-association.org/mathworks-modeling-award-returns-to-wro-2026/) · [2026 rubric](https://wro-association.org/wp-content/uploads/MathWorks-Modeling-Award-Scoring-Rubric_20260312.pdf) · [Future Engineers rules §7](https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf#page=9) · [WRO software request](https://www.mathworks.com/academia/student-competitions/world-robot-olympiad.html) · [MATLAB Online options](https://www.mathworks.com/products/matlab-online/matlab-online-versions.html)
