# w_park_u_base

Tipo de prueba: carrera completa con estacionamiento geométrico (latencia/uno.sh: salida del cajón, 3 vueltas y estacionamiento) en el banco NUEVO con latencia. TWIN_FW_OVER PARK_GEOM_EXT=1 + PARK_GEOM_ROJO=1, TWIN_PI_EXTRA ROUTE_PARK_GEOM_EXT=1, TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, TWIN_CAMERA_FROM_CALIB=1, TWIN_LENS_CALIB=pure_pursuit/lens_calib.npz, TWIN_COST_FIRST=1.95, FF 2.8 / FD 3.8, ruido 1-2 x semillas 16 20 32 (6 carreras), xargs -P 7. Commit 28b640f (origin/grok/t/w_park_u_base): commit vacío sobre 2398305 (Park GEOM r13 rojoCerca + r14d stickyEarly), mismo árbol que w_park_leg_s35_base.

## Resumen (resumen.py)

w_park_u_base: 5/6 ok  media 42.44  mediana 43.22  mejor 37.75  fallas: n1/s20

## Carreras (última línea STOP)

n1 s16 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 42.986, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.934, 9.779, 10.174], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.738, "E": 6.755, "I": 0.909, "S": 35.024}, "collisions": [], "min_clearance_wall_mm": 31.01, "min_clearance_sign_mm": 52.96, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24253.4, "distance_backward_mm": 334.0, "mean_speed_mm_s": 572.0, "frames_processed": 1398, "mean_pi_frame_ms": 3.74, "reverse_motor_s": 3.149, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 24532, "render_s": 1.42, "pi_s": 5.232, "other_s": 4.497, "wall_time_s": 11.149, "wall_per_sim_s": 0.259}
n1 s20 STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 33.162, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.379, 8.342, 8.382], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.739, "E": 2.929, "I": 0.82, "S": 29.113}, "collisions": [{"cause": "cajón magenta", "t": 33.869678}], "min_clearance_wall_mm": 71.78, "min_clearance_sign_mm": 96.27, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 20564.3, "distance_backward_mm": 222.6, "mean_speed_mm_s": 626.8, "frames_processed": 1075, "mean_pi_frame_ms": 3.7, "reverse_motor_s": 1.499, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 19109, "render_s": 1.043, "pi_s": 3.975, "other_s": 3.93, "wall_time_s": 8.949, "wall_per_sim_s": 0.27}
n1 s32 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 44.235, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.697, 10.109, 10.23], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.779, "E": 7.788, "I": 0.818, "S": 35.289}, "collisions": [], "min_clearance_wall_mm": 34.61, "min_clearance_sign_mm": 82.64, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 25104.2, "distance_backward_mm": 361.9, "mean_speed_mm_s": 575.7, "frames_processed": 1436, "mean_pi_frame_ms": 3.2, "reverse_motor_s": 3.628, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 25372, "render_s": 1.41, "pi_s": 4.597, "other_s": 4.371, "wall_time_s": 10.378, "wall_per_sim_s": 0.235}
n2 s16 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 43.224, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.898, 9.989, 10.255], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.738, "E": 6.708, "I": 0.91, "S": 35.307}, "collisions": [], "min_clearance_wall_mm": 13.7, "min_clearance_sign_mm": 46.97, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24540.1, "distance_backward_mm": 326.9, "mean_speed_mm_s": 575.3, "frames_processed": 1410, "mean_pi_frame_ms": 3.64, "reverse_motor_s": 3.058, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 24659, "render_s": 1.421, "pi_s": 5.133, "other_s": 4.492, "wall_time_s": 11.046, "wall_per_sim_s": 0.256}
n2 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 37.746, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.503, 8.422, 8.47], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.779, "E": 7.151, "I": 0.791, "S": 29.465}, "collisions": [], "min_clearance_wall_mm": 41.32, "min_clearance_sign_mm": 118.77, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 21001.2, "distance_backward_mm": 330.9, "mean_speed_mm_s": 565.2, "frames_processed": 1231, "mean_pi_frame_ms": 3.52, "reverse_motor_s": 3.451, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 21930, "render_s": 1.243, "pi_s": 4.335, "other_s": 4.014, "wall_time_s": 9.592, "wall_per_sim_s": 0.254}
n2 s32 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 44.005, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.697, 10.051, 10.199], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.779, "E": 7.714, "I": 0.799, "S": 35.152}, "collisions": [], "min_clearance_wall_mm": 36.85, "min_clearance_sign_mm": 79.5, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24907.9, "distance_backward_mm": 353.8, "mean_speed_mm_s": 574.1, "frames_processed": 1423, "mean_pi_frame_ms": 3.36, "reverse_motor_s": 3.571, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 25220, "render_s": 1.482, "pi_s": 4.778, "other_s": 4.591, "wall_time_s": 10.851, "wall_per_sim_s": 0.247}

## Costo por carrera

COSTO n1 s16 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1398, "media_ms": 10.794789091416325, "p50_ms": 7.575150799997487, "p95_ms": 25.69935759999939, "max_ms": 244.14984660000067, "pct_mas_25ms": 5.436337625178827, "dp_ms": [1950, 1950, 222, 111, 136, 125, 115, 109, 113, 99, 97, 96, 81, 91, 92, 114]}
COSTO n1 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1075, "media_ms": 10.707486632930262, "p50_ms": 6.880176799999659, "p95_ms": 25.85280040000093, "max_ms": 248.89220600000024, "pct_mas_25ms": 5.674418604651163, "dp_ms": [1950, 1950, 226, 123, 106, 95, 112, 91, 79, 80, 64, 89, 64, 83, 85, 91]}
COSTO n1 s32 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1436, "media_ms": 9.30113437827294, "p50_ms": 6.49586000000113, "p95_ms": 20.78945960000009, "max_ms": 251.42539160000138, "pct_mas_25ms": 2.785515320334262, "dp_ms": [1950, 1950, 230, 113, 111, 164, 104, 108, 110, 102, 101, 78, 91, 75, 88, 109]}
COSTO n2 s16 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1410, "media_ms": 10.520098503120513, "p50_ms": 7.406574000000886, "p95_ms": 24.830923600000077, "max_ms": 244.91141199999987, "pct_mas_25ms": 4.680851063829787, "dp_ms": [1950, 1950, 222, 111, 138, 109, 109, 94, 80, 99, 96, 82, 73, 94, 99, 85]}
COSTO n2 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1231, "media_ms": 10.158790480097489, "p50_ms": 6.880809599999793, "p95_ms": 24.754805600001358, "max_ms": 259.1987163999998, "pct_mas_25ms": 4.6303818034118605, "dp_ms": [1950, 1950, 237, 125, 112, 106, 98, 81, 67, 87, 91, 104, 91]}
COSTO n2 s32 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1423, "media_ms": 9.764719922417399, "p50_ms": 6.780916799996105, "p95_ms": 20.79305760000061, "max_ms": 254.54379920000002, "pct_mas_25ms": 3.021784961349262, "dp_ms": [1950, 1950, 231, 115, 116, 170, 111, 112, 113, 106, 106, 80, 81, 74, 112, 167]}

## Tiempo de reloj total

11.3 s (6 carreras, -P 7 = 1 oleada; warm-up de 1 carrera fuera del cronómetro)

## Validez

VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx / mem máx 1.51 / 14332 MB, sin OOM/Killed/MemoryError, mean_pi_frame_ms 3.20-3.74, media de media_ms (COSTO) 10.2 ms.

## Comparación (mismo tipo: carrera completa con estacionamiento geométrico)

| carrera | w_park_leg_s35_base (mismo árbol, sin cámara calib) | w_geom_rojo_s35_base (094f93c, cámara calib) | r16_wstop | w_park_u_base |
|---|---|---|---|---|
| n1 s16 | 43.340 ok | 43.462 ok | 43.520 ok | 42.986 ok |
| n1 s20 | - | - | - | choque 33.162 |
| n1 s32 | 45.179 ok | 43.931 ok | 45.230 ok | 44.235 ok |
| n2 s16 | 42.928 ok | 43.264 ok | 43.708 ok | 43.224 ok |
| n2 s20 | - | - | - | 37.746 ok |
| n2 s32 | 47.035 ok | 42.828 ok | 43.429 ok | 44.005 ok |

- Carreras que terminan en ambas: vs w_park_leg_s35_base 4 carreras -4.03 s; vs w_geom_rojo_s35_base 4 carreras +0.96 s; vs r16_wstop 4 carreras -1.44 s.
- s20 no se había corrido antes en este tipo (estacionamiento geométrico), así que n1/s20 no tiene referencia directa.
