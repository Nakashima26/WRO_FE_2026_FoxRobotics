# Graph Report - WRO_FE_2026_FoxRobotics  (2026-10-09)

## Corpus Check
- 114 files · ~210,262 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 19 file(s) not represented in the graph (top: .ino 9, (none) 6, .service 2)

## Summary
- 2027 nodes · 4700 edges · 117 communities (89 shown, 28 thin omitted)
- Extraction: 89% EXTRACTED · 11% INFERRED · 0% AMBIGUOUS · INFERRED: 494 edges (avg confidence: 0.88)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `b3f27128`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- .run
- sil_core.cpp
- CameraModel
- math
- centerline.py
- numpy
- IntegratedRuntime
- params.py
- runtime_nuevo.py
- test_render_equiv.py
- SignMap
- run_gui
- simulacion_v2.py
- sim.py
- ObstacleMemory
- cv2
- PPRuntime
- String
- test_track_map.py
- .process_frame
- reporte_run.py
- Hallazgos
- wro_field.py
- _make_runtime
- Localizer
- client.py
- test_native_build.py
- test_sensors.py
- 5.5 Failure & Incident log
- T13 — hw_nuevo: 3 vueltas + estacionar (agente 2026-10-03)
- Vision
- csv
- chassis_twin.py
- sil_advance_internal
- DigitalMap
- f
- TrackGeometry
- paridad.py
- FoxRobotics — Engineering Journal
- OrangeLineTracker
- salida.py
- native/build.py
- Sim
- digital_map.py
- TwinParams
- .hold_pasado
- _run_once
- HardwareSerial
- sensors.py
- test_paridad.py
- calibra_luz.py
- .__init__
- test_digital_dir.py
- 4. Software Architecture
- BEVTransformer
- MidTurnObstacleDetector
- INSTRUCCIONES.md
- VisionCorpusRecorder
- README.md
- firmware/build.py
- Arduino.h
- _WallWorld
- os
- CarController
- _map_safe
- SalidaCajon
- .render
- SerialLink
- time
- twin/prepark.py
- Handoff — digital twin Fox (rama `digital-twin`)
- BevRecorder
- FarHintManager
- map_view.py
- planner.py
- track_map.py
- MPU6050
- .__init__
- IntegratedRuntime
- diagnose_uart.py
- 1.2 Bill of Materials
- 6. Testing & Validation
- _paint
- run
- TrackFrame
- PPRuntime
- drive.py
- Phase 5 — Wall-aware steering; Open Challenge validated (2026-08-20 → 08-28)
- ESP32Model
- DistGyro.py
- 3.2 Sensor selection and placement
- subprocess
- FakeSerialLink
- ThreadedFrameGrabber
- abi.c
- 2. Mechanical Design & Mobility
- Vision
- AsyncVideoWriter
- Phase 1 — First closed-loop Open Challenge run (May 2026)
- Phase 2 — Two controllers, one link (June 2026)
- Phase 4 — Sim, calibration, corner lines (2026-08-11 → 08-19)
- sil_set_callbacks
- Encoder
- test_localizer_with_twin_sensors
- AsyncVideoWriter
- Plan digital twin / mapeo (rama `digital-twin`)
- all.sh
- SonarDef
- _canonize_for_hash
- calibration.py
- PinState
- setup-github-runner.sh
- PROMPT_SIGUIENTE.md
- install_autostart_pi.sh
- tools/README.md

## God Nodes (most connected - your core abstractions)
1. `PPRuntime` - 72 edges
2. `DigitalMap` - 56 edges
3. `FirmwareSIL` - 43 edges
4. `TwinParams` - 42 edges
5. `TrackGeometry` - 40 edges
6. `CameraModel` - 39 edges
7. `BEVTransformer` - 37 edges
8. `SignMap` - 37 edges
9. `randomize()` - 37 edges
10. `Vision` - 37 edges

## Surprising Connections (you probably didn't know these)
- `2026-08-31 — Turn direction inferred during the run` --references--> `TurnDirectionTracker`  [INFERRED]
  ENGINEERING_JOURNAL.md → src/RASPI/cam/pure_pursuit/corner_lines.py
- `Siguiente (en orden)` --references--> `DigitalMap`  [INFERRED]
  docs/HANDOFF.md → src/RASPI/cam/pure_pursuit/digital_map.py
- `Lista de tareas` --references--> `DigitalMap`  [INFERRED]
  docs/twin_plan.md → src/RASPI/cam/pure_pursuit/digital_map.py
- `T13 — hw_nuevo: 3 vueltas + estacionar (agente 2026-10-03)` --references--> `OrangeLineTracker`  [INFERRED]
  docs/twin_plan.md → src/RASPI/cam/pure_pursuit/corner_lines.py
- `T7 decisiones (plan de cambios, en orden)` --references--> `DigitalMap`  [INFERRED]
  docs/twin_plan.md → src/RASPI/cam/pure_pursuit/digital_map.py

## Import Cycles
- None detected.

## Communities (117 total, 28 thin omitted)

### Community 0 - ".run"
Cohesion: 0.07
Nodes (36): FirmwareSetupError, FirmwareSIL, _advance(), _encoder(), _gyro(), _tof(), SimSerialLink, Gyro (+28 more)

### Community 1 - "sil_core.cpp"
Cohesion: 0.05
Nodes (11): digitalRead(), digitalWrite(), HardwareSerial::available(), on_digital_write(), register_default_sonars_once(), serial2_visible_count(), set_error(), sil_digital() (+3 more)

### Community 2 - "CameraModel"
Cohesion: 0.07
Nodes (8): Twin SIL: perfilado y 3 optimizaciones clase A (deque de history, warp_wide a demanda) (2026-10-04, rama t15b2), CameraModel, warp_wide(), _pose_at(), test_render_performance(), test_render_sign_bev_projection(), cam(), field()

### Community 3 - "math"
Cohesion: 0.08
Nodes (12): obstacles_to_bev(), offset_polyline(), parse_args(), ray_segment_intersect(), render_bev(), run(), _square_corners(), ultrasonic_distances_cm() (+4 more)

### Community 4 - "centerline.py"
Cohesion: 0.08
Nodes (18): bend_line_above(), _clamp_to_pass_side(), detect_centerline(), draw_bev_debug(), _enforce_free_mask(), _extract_thin_blue_line(), _far_pass_cx(), _limit_lateral_step() (+10 more)

### Community 5 - "numpy"
Cohesion: 0.11
Nodes (18): _band_main_label(), _band_px_count(), _band_slice(), _components(), _core_px_count(), _curve_from(), curve_side(), detect_lines() (+10 more)

### Community 6 - "IntegratedRuntime"
Cohesion: 0.06
Nodes (8): AsyncVideoWriter, IntegratedRuntime, next_output_path(), PID, resolve_output_path(), SerialLink, ThreadedFrameGrabber, TurnDetector

### Community 7 - "params.py"
Cohesion: 0.09
Nodes (19): CameraParams, GyroParams, max_wheel_deg(), MotorParams, _prov(), steady_speed_mm_s(), SteeringParams, TimingParams (+11 more)

### Community 8 - "runtime_nuevo.py"
Cohesion: 0.10
Nodes (9): next_output_path(), resolve_output_path(), main(), parse_args(), parse_args(), Config, main(), parse_args() (+1 more)

### Community 9 - "test_render_equiv.py"
Cohesion: 0.08
Nodes (16): _clip_near(), _fill_face(), _project(), reference_render(), _ribbon(), _ribbon_to_cam(), _to_cam(), _poses_runner() (+8 more)

### Community 10 - "SignMap"
Cohesion: 0.13
Nodes (11): FrameResult, _color_norm(), FieldPose, SeatDef, _SeatState, SignMap, _field_center_to_bev_foot(), test_query_bev_roundtrip() (+3 more)

### Community 11 - "run_gui"
Cohesion: 0.08
Nodes (16): Acumulador, analizar(), _channels(), _fmt_ranges(), _linea_piso(), get_raw(), _mask_hsv(), _pasa() (+8 more)

### Community 12 - "simulacion_v2.py"
Cohesion: 0.07
Nodes (13): calcular_distancia_pared(), camara_wro(), PID, build_wall_grid(), cell_free(), clamp(), color_bias(), lerp_color() (+5 more)

### Community 13 - "sim.py"
Cohesion: 0.11
Nodes (17): compute_metrics(), _lateral_side(), _min_body_clearance(), _parse_ack_field(), _parse_est(), _section_along_mm(), main(), car() (+9 more)

### Community 15 - "cv2"
Cohesion: 0.11
Nodes (17): build_segment_bounds(), detect_turn_direction(), draw_debug(), FrameGrabber, get_window_image_size(), open_camera(), run(), run() (+9 more)

### Community 16 - "PPRuntime"
Cohesion: 0.12
Nodes (5): main(), on_camera_lost(), should_record(), should_start(), PPRuntime

### Community 17 - "String"
Cohesion: 0.08
Nodes (18): String, data_, LedcState, attached, duty, freq, res, operator+() (+10 more)

### Community 18 - "test_track_map.py"
Cohesion: 0.14
Nodes (15): _f(), fields_from_ack_line(), _i(), _LegacyState, _norm_deg(), Odometry, OdomPose, parse_ack() (+7 more)

### Community 19 - ".process_frame"
Cohesion: 0.07
Nodes (11): T16 — resultados (2026-10-04, rama t16-perf), is_interior_pass(), _parse_direccion(), _parse_estado(), _parse_heading(), _parse_inicio(), _parse_pb(), _parse_tc() (+3 more)

### Community 20 - "reporte_run.py"
Cohesion: 0.15
Nodes (20): a_html(), a_texto(), cargar_lineas(), construir(), cortar_run(), esquinas(), estacionamiento(), linea() (+12 more)

### Community 21 - "Hallazgos"
Cohesion: 0.08
Nodes (24): Estado tras merge (commit 0698d87, 2026-10-03), Estado tras T13 (commit 25de149, agente detenido por el usuario), Hallazgos, Lecciones (para cualquier agente nuevo), Plan T15 (2026-10-03) — atacar las maniobras con escenarios cortos, Plan T16 — rendimiento del twin (2026-10-04), Reparto de trabajo (2026-10-03), Revisión seeds / randomizer (verificado contra app.py oficial, 2026-10-03) (+16 more)

### Community 22 - "wro_field.py"
Cohesion: 0.13
Nodes (13): T15b2 — el cambio de sentido ahora dispara DENTRO de la esquina 13 (reglamento 9.23) (2026-10-04, rama t15b2), T15b2 — patrón 1 (CCW dispara la U encima del cajón): umbral de recorrido por sentido (2026-10-04, rama t15b2), _barrier_corners(), _card_seats(), Field, randomize(), _rear_axle_at_center(), section_to_world() (+5 more)

### Community 23 - "_make_runtime"
Cohesion: 0.18
Nodes (20): _orange(), test_hint_cleared_when_turn_starts(), test_hint_holds_after_line_passes_under_car(), test_v2_carries_hint_when_enabled(), test_v2_without_hint_by_default(), _run_one_v2(), _shadow_only(), test_gr_gf_from_mapped_entry_sign() (+12 more)

### Community 24 - "Localizer"
Cohesion: 0.19
Nodes (7): Localizer, _norm_h(), SensorReading, _side_readings(), test_localizer_us_tof_inconsistent_prefers_tof(), test_snap_lateral_ignores_sign_in_cone(), test_snap_lateral_recovers_start_offset()

### Community 25 - "client.py"
Cohesion: 0.11
Nodes (9): execute_command(), get_cam_overlay(), handle_commands(), main(), scale(), stream_screen(), index(), pi_endpoint() (+1 more)

### Community 26 - "test_native_build.py"
Cohesion: 0.12
Nodes (9): disable(), enabled(), get_kernel(), get_lib(), _reset(), _warn(), test_native_build_caching(), test_native_disable_env() (+1 more)

### Community 27 - "test_sensors.py"
Cohesion: 0.18
Nodes (12): _body_dir_world(), Pose, Ultrasonic, advance(), rng(), test_tof_black_wall_range(), test_tof_red_sign(), test_ultrasonic_perpendicular_wall() (+4 more)

### Community 28 - "5.5 Failure & Incident log"
Cohesion: 0.09
Nodes (19): 2026-08-29 — `ERR-06` fixed: pivot-trap, 2026-08-29 — `ERR-07` fixed: measured-state RECUPERANDO trigger, 2026-08-29 — Orange-line direction, fast verdict, Phase 6 — Obstacle-round root causes (2026-08-29), 5.1 Subsystem interaction map, 5.2 Key engineering trade-offs, 5.3 Iteration log, 5.4 Risk analysis (+11 more)

### Community 29 - "T13 — hw_nuevo: 3 vueltas + estacionar (agente 2026-10-03)"
Cohesion: 0.12
Nodes (5): T13 — hw_nuevo: 3 vueltas + estacionar (agente 2026-10-03), ema_per_frame(), fr(), per_frame(), _ack_num()

### Community 30 - "Vision"
Cohesion: 0.12
Nodes (3): run_car(), CarController, Vision

### Community 31 - "csv"
Cohesion: 0.14
Nodes (6): main(), _parse_seeds(), _run_one(), main(), run_one(), park()

### Community 32 - "chassis_twin.py"
Cohesion: 0.15
Nodes (9): _body_polygon(), _closed(), _draw_static(), _draw_world(), _seat_dots(), SimFrameView, blue_segments(), orange_segments() (+1 more)

### Community 33 - "sil_advance_internal"
Cohesion: 0.14
Nodes (20): Reglas duras (aprendidas, 2026-10-04), check_setup_timeout(), delay(), delayMicroseconds(), echo_pin_to_id(), HardwareSerial::readStringUntil(), HardwareSerial::writeChar(), millis() (+12 more)

### Community 34 - "DigitalMap"
Cohesion: 0.15
Nodes (3): T16infra — sentido del mapa desde el ESP, verdad del twin apagable, `paridad.py` (2026-10-05, rama t16infra), T3/T4 auditoría (verificado en código), DigitalMap

### Community 35 - "f"
Cohesion: 0.14
Nodes (11): f(), _apply_config_overrides(), _call_original_and_hash(), compare_corpus_fingerprints(), fingerprint_corpus(), main(), _reconstruct_arrays_from_npz(), _reconstruct_from_json() (+3 more)

### Community 37 - "paridad.py"
Cohesion: 0.25
Nodes (15): compare(), _config_line(), _fmt(), InoInfo, _line_of(), main(), _norm(), parse_ino() (+7 more)

### Community 38 - "FoxRobotics — Engineering Journal"
Cohesion: 0.11
Nodes (18): 2026-03-30 → 04-05 — Semana Santa design week, 2026-04-13 → 04-27 — First commits: CAD + PCB, ~2026-04-25 → 05-09 — Per-peripheral ESP32 bring-up, 2026-04 — Chassis, base wiring, PCB layout started, 2026-07 (first week) — Lens swap 63° → 120°, 2026-08-07 — `ERR-03`: red cast on every frame, 2026-08 → 09 — **branch** work (`Workin`, `SectionTurning`) — not on `main`, 2026-08-31 — Camera-safe service restart (+10 more)

### Community 40 - "salida.py"
Cohesion: 0.17
Nodes (9): build_field(), default_scenarios(), main(), poly_dist(), run_one(), coll(), _seg_pt(), summarize() (+1 more)

### Community 41 - "native/build.py"
Cohesion: 0.19
Nodes (6): build(), build_hash(), compiler(), _ext(), flags(), _replace_or_keep()

### Community 42 - "Sim"
Cohesion: 0.22
Nodes (9): Sim, test_baseline_short_run(), compare_with_golden(), compute_fingerprint(), extract_metrics(), main(), _norm(), save_golden() (+1 more)

### Community 43 - "digital_map.py"
Cohesion: 0.15
Nodes (4): onto(), smooth(), _lane_pose(), _heading()

### Community 44 - "TwinParams"
Cohesion: 0.18
Nodes (10): TwinParams, test_gyro_rate(), bench_collision(), bench_detect_on(), bench_firmware(), bench_render(), bench_sensors(), bench_widest() (+2 more)

### Community 45 - ".hold_pasado"
Cohesion: 0.17
Nodes (4): T2 — esquina SE (abajo-derecha, giro 3): ve el verde y gira igual, T7 decisiones (plan de cambios, en orden), T8 — bitácora de implementación (en curso), test_map_undecided_is_neutral()

### Community 46 - "_run_once"
Cohesion: 0.18
Nodes (13): _field_payload(), main(), _offer(), _parse_kv(), _run_once(), _hud(), on_frame(), _pace() (+5 more)

### Community 47 - "HardwareSerial"
Cohesion: 0.15
Nodes (9): HardwareSerial, kind_, Kind, UART2, USB, HardwareSerial::print(), HardwareSerial::println(), HardwareSerial::writeRaw() (+1 more)

### Community 48 - "sensors.py"
Cohesion: 0.15
Nodes (5): BnoParams, ToFParams, UltrasonicParams, Bno085Heading, ToF

### Community 49 - "test_paridad.py"
Cohesion: 0.17
Nodes (7): map_truth_keys(), resolve_preset(), test_map_truth_keys(), _env(), test_diferencia_sin_motivo_rompe(), test_hw_nuevo_todo_aceptado(), test_preset_contra_ino_sintetico()

### Community 50 - "calibra_luz.py"
Cohesion: 0.17
Nodes (7): frames_avi(), frames_bin(), frames_camara(), main(), _mask_rojo(), _mask_rosa(), open_camera()

### Community 51 - ".__init__"
Cohesion: 0.15
Nodes (4): FloorColorCorrector, TurnDirectionTracker, next_output_path(), resolve_output_path()

### Community 52 - "test_digital_dir.py"
Cohesion: 0.18
Nodes (11): _acks(), _feet(), map_cfg(), _orange(), test_config_garbage_is_undecided(), test_set_direction_equals_forced(), test_sim_inject_option(), test_update_digital_ignores_vision_tracker() (+3 more)

### Community 53 - "4. Software Architecture"
Cohesion: 0.13
Nodes (13): 4.1 System overview — two controllers, one link, 4.2 Inter-controller protocol ("V2"), 4.3 Vision pipeline (Raspberry Pi — `pure_pursuit/`), 4.4 Obstacle handling (Obstacle Challenge), 4.5.1 Open-round corner hardening, 4.5 ESP32 finite state machine (`src/ESP32/PurePursuit/PurePursuit.ino`), 4.6 Cascade PID (always running underneath), 4.7 Startup handshake (+5 more)

### Community 54 - "BEVTransformer"
Cohesion: 0.20
Nodes (3): BEVTransformer, map_obstacle_to_bev(), seen_above_obstacles()

### Community 56 - "INSTRUCCIONES.md"
Cohesion: 0.13
Nodes (13): 1. Calibrar la cámara (una sola vez), 2. Calibrar colores (antes de cada competencia / si cambia la luz), 3. Probar solo la visión (sin robot, sin ESP32), 4. Correr en el robot real, Comando, Flags útiles, Requisitos previos, Resumen de archivos (+5 more)

### Community 58 - "README.md"
Cohesion: 0.14
Nodes (13): 10. Photos, 7.1 Hardware requirements, 7.2 ESP32 firmware, 7.3 Raspberry Pi software, 7.4 Calibration (before each venue), 7.5 Run, 7.6 Autostart & deployment, 7. How to Build & Run (+5 more)

### Community 59 - "firmware/build.py"
Cohesion: 0.21
Nodes (7): _apply_overrides(), build(), _build_hash(), _cxx(), load_ino_text(), _replace_or_keep(), test_build_default_and_override()

### Community 60 - "Arduino.h"
Cohesion: 0.19
Nodes (3): arduino_max(), arduino_min(), constrain()

### Community 61 - "_WallWorld"
Cohesion: 0.23
Nodes (5): _EastWallWorld, _WallWorld, _poly_edges(), Segment, _square_edges()

### Community 62 - "os"
Cohesion: 0.27
Nodes (5): check_gpio_availability(), main(), read_response(), send_test_data(), setup_serial_port()

### Community 63 - "CarController"
Cohesion: 0.21
Nodes (3): CarController, run_car(), start_ssr()

### Community 65 - "SalidaCajon"
Cohesion: 0.29
Nodes (6): _num(), SalidaCajon, _foot(), test_lado(), test_lata_lejos_no_decide(), test_necesita_votos_y_solo_en_inicio()

### Community 66 - ".render"
Cohesion: 0.20
Nodes (3): poly(), px(), _seat_world()

### Community 67 - "SerialLink"
Cohesion: 0.17
Nodes (3): main(), PPConfig, SerialLink

### Community 68 - "time"
Cohesion: 0.24
Nodes (6): PPConfig, test_ack_turn_g_debounce_and_recovery(), test_arm_sends_ready_three_times(), test_chassis_twin_frame_optional(), test_process_frame_armed_sends_v2(), test_process_frame_disarmed_sends_nothing()

### Community 69 - "twin/prepark.py"
Cohesion: 0.23
Nodes (6): all_scenarios(), build(), park_metrics(), Scenario, stall_box(), _units()

### Community 70 - "Handoff — digital twin Fox (rama `digital-twin`)"
Cohesion: 0.18
Nodes (10): Cómo trabajar (preferencia del usuario), Entorno (Windows), Estado (2026-10-05, rama `t15b2` HEAD e2914de, NO fusionada a `digital-twin`, NO pusheada), Handoff — digital twin Fox (rama `digital-twin`), Mac (corridas pesadas — Windows está al tope de RAM), Pendiente de medir en el carro nuevo, Presets, Ramas viejas (sesiones anteriores; revisar antes de borrar) (+2 more)

### Community 73 - "map_view.py"
Cohesion: 0.40
Nodes (7): _bev_corners_field(), _build_static(), draw_poly(), _draw_robot(), render_map(), _world_to_img(), field_heading_to_twin()

### Community 74 - "planner.py"
Cohesion: 0.25
Nodes (5): color_bias(), is_free(), plan_trajectory(), rollout(), score_traj()

### Community 75 - "track_map.py"
Cohesion: 0.20
Nodes (5): _range_weight(), _ray_seg(), SensorMount, _shift_toward_island(), _WallCaster

### Community 76 - "MPU6050"
Cohesion: 0.20
Nodes (3): MPU6050, gyro_z_, TwoWire

### Community 77 - ".__init__"
Cohesion: 0.18
Nodes (4): Config, main(), PID, TurnDetector

### Community 79 - "diagnose_uart.py"
Cohesion: 0.25
Nodes (5): check_gpio_pins(), check_uart_status(), list_serial_devices(), main(), show_solutions()

### Community 80 - "1.2 Bill of Materials"
Cohesion: 0.20
Nodes (10): 1.1 Design goals, 1.2 Bill of Materials, 1. Vehicle Overview, Actuators, Custom Manufactured Parts, Electronics, Fasteners, LEGO Components (+2 more)

### Community 81 - "6. Testing & Validation"
Cohesion: 0.20
Nodes (10): 6.1 How we test, 6.2 Results to date, 6.3 Validation matrix, 6.4 Pre-run checklist (each venue), 6.5 Milestones, 6. Testing & Validation, MIL-01 — Open Challenge, full autonomous run, MIL-02 — Obstacle avoidance, standard layouts (+2 more)

### Community 82 - "_paint"
Cohesion: 0.27
Nodes (8): _fit_panel(), _key_name(), _paint(), poly(), _paint_static(), poly(), _to_px(), _view_window()

### Community 83 - "run"
Cohesion: 0.31
Nodes (3): get_frame(), Picker, run()

### Community 87 - "Phase 5 — Wall-aware steering; Open Challenge validated (2026-08-20 → 08-28)"
Cohesion: 0.22
Nodes (9): 2026-08-20 → 08-21 — `ParedCenterline` merged, 2026-08-24 — Ghost obstacle after a turn, 2026-08-25 → 08-27 — Line-classification & centerline tuning stretch, 2026-08-27 — Obstacle-pass pulse fixed; earlier dodge, 2026-08-28 — `ERR-07`: RECUPERANDO anchor drifts (experiments), 2026-08-28 — `ERR-08`: Pi undervoltage → frame-rate rescale, 2026-08-28 — Hot start + run recording, 2026-08-28 — **M8: Open Challenge validated** (+1 more)

### Community 90 - "3.2 Sensor selection and placement"
Cohesion: 0.25
Nodes (7): 3.1 Power budget, 3.2 Sensor selection and placement, 3.3 PCB & wiring, 3. Power Architecture & Sensors, Camera system, HC-SR04 ultrasonic sensors, MPU-6050 IMU

### Community 91 - "subprocess"
Cohesion: 0.32
Nodes (3): test_native_concurrent_build(), main(), run_config()

### Community 94 - "abi.c"
Cohesion: 0.43
Nodes (4): fox_abi_version(), fox_flt_eval_method(), fox_probe_contract_d(), fox_probe_contract_f()

### Community 95 - "2. Mechanical Design & Mobility"
Cohesion: 0.29
Nodes (7): 2.1 Chassis selection, 2.2 Wheels, 2.3 Drive system (torque vs. speed analysis), 2.4 Steering — rack-and-pinion with Ackermann geometry, 2.5 Custom parts & assembly, 2. Mechanical Design & Mobility, Assembly order

### Community 98 - "Phase 1 — First closed-loop Open Challenge run (May 2026)"
Cohesion: 0.33
Nodes (6): 2026-05-09 — First working Open-round controller, 2026-05-23 — Camera bring-up, 2026-05-25 → 05-26 — Repo reorganized, first README, 2026-05 — `ERR-01`: ToF lost the black wall, 2026-05 — `ERR-02`: wall error goes blind at high yaw, Phase 1 — First closed-loop Open Challenge run (May 2026)

### Community 99 - "Phase 2 — Two controllers, one link (June 2026)"
Cohesion: 0.33
Nodes (6): 2026-06-04 → 06-05 — Integration pass + UART diagnostics, 2026-06-08 → 06-09 — Two-controller split formalized + deploy infra, 2026-06-11 → 06-13 — Pure Pursuit workspace, 2026-06-22 — Centerline extraction + geometric Pure Pursuit, 2026-06-25 — Last commit before the July break, Phase 2 — Two controllers, one link (June 2026)

### Community 100 - "Phase 4 — Sim, calibration, corner lines (2026-08-11 → 08-19)"
Cohesion: 0.33
Nodes (6): 2026-08-11 — Offline kinematic sim, 2026-08-12 — Obstacle detection hardening, 2026-08-13 — BEV calibration 4 → 9 points, 2026-08-15 — "line memory" tried and reverted, 2026-08-17 → 08-19 — Orange / blue ground-line tracking, Phase 4 — Sim, calibration, corner lines (2026-08-11 → 08-19)

### Community 103 - "Encoder"
Cohesion: 0.47
Nodes (3): EncoderParams, Encoder, test_encoder_slip()

### Community 104 - "test_localizer_with_twin_sensors"
Cohesion: 0.40
Nodes (4): tp(), test_localizer_with_twin_sensors(), tof_mm(), us_mm()

### Community 106 - "Plan digital twin / mapeo (rama `digital-twin`)"
Cohesion: 0.40
Nodes (4): Entorno (Windows), Lista de tareas, Meta, Plan digital twin / mapeo (rama `digital-twin`)

### Community 107 - "all.sh"
Cohesion: 0.40
Nodes (4): MKL_NUM_THREADS, OMP_NUM_THREADS, OPENBLAS_NUM_THREADS, all.sh script

### Community 108 - "SonarDef"
Cohesion: 0.50
Nodes (4): SonarDef, echo, id, trig

### Community 111 - "PinState"
Cohesion: 0.67
Nodes (3): PinState, level, mode

## Knowledge Gaps
- **22 isolated node(s):** `data_`, `USB`, `UART2`, `kind_`, `gyro_z_` (+17 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 789 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **28 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `DigitalMap` connect `DigitalMap` to `.render`, `._seat_ok`, `Handoff — digital twin Fox (rama `digital-twin`)`, `runtime_nuevo.py`, `Plan digital twin / mapeo (rama `digital-twin`)`, `digital_map.py`, `.hold_pasado`, `PPRuntime`, `.__init__`, `test_digital_dir.py`, `MidTurnObstacleDetector`, `T13 — hw_nuevo: 3 vueltas + estacionar (agente 2026-10-03)`?**
  _High betweenness centrality (0.127) - this node is a cross-community bridge._
- **Are the 23 inferred relationships involving `PPRuntime` (e.g. with `BEVTransformer` and `BevRecorder`) actually correct?**
  _`PPRuntime` has 23 INFERRED edges - model-reasoned connections that need verification._
- **What connects `data_`, `USB`, `UART2` to the rest of the system?**
  _22 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `.run` be split into smaller, more focused modules?**
  _Cohesion score 0.06876285630326183 - nodes in this community are weakly interconnected._
- **Why does `Handoff — digital twin Fox (rama `digital-twin`)` connect `Handoff — digital twin Fox (rama `digital-twin`)` to `sil_advance_internal`?**
  _High betweenness centrality (0.117) - this node is a cross-community bridge._
- **Are the 8 inferred relationships involving `DigitalMap` (e.g. with `Siguiente (en orden)` and `Lista de tareas`) actually correct?**
  _`DigitalMap` has 8 INFERRED edges - model-reasoned connections that need verification._
- **Should `sil_core.cpp` be split into smaller, more focused modules?**
  _Cohesion score 0.050241545893719805 - nodes in this community are weakly interconnected._