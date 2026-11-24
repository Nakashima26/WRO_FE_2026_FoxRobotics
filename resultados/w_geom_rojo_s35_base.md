# w_geom_rojo_s35_base

Tipo de prueba: carrera completa con estacionamiento (latencia/uno.sh: 3 vueltas + estacionamiento, FOX_PARK_POS=random) con estacionamiento geométrico (TWIN_FW_OVER='{"PARK_GEOM_EXT":"1","PARK_GEOM_ROJO":"1"}', TWIN_PI_EXTRA='{"ROUTE_PARK_GEOM_EXT":1}') en el banco NUEVO con latencia (latencia/uno.sh + costo.py, FF 2.8 / FD 3.8, primer plan 1.95 s, TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, TWIN_CAMERA_FROM_CALIB=1, TWIN_LENS_CALIB=pure_pursuit/lens_calib.npz), ruido 1-2 x semillas 35 32 16 (6 carreras), xargs -P 7, commit 2f23448 (origin/grok/t/w_geom_rojo_s35_base). Comparar solo contra tandas del mismo tipo y la misma configuración.

## Resumen (resumen.py)

w_geom_rojo_s35_base: 5/6 ok  media 43.85  mediana 43.46  mejor 42.83  fallas: n1/s35

## Carreras (última línea STOP)

n1 s35 STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 45.695, "turns_completed": 12, "tpr": 12, "lap_times_s": [16.527, 9.418, 9.386], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.812, "E": 8.263, "I": 0.874, "S": 36.218}, "collisions": [{"cause": "cajón magenta", "t": 46.443092}], "min_clearance_wall_mm": 51.31, "min_clearance_sign_mm": 78.61, "pass_side_violations": [{"sign": "Red W/T4", "t": 10.44531, "lateral_mm": -273.9}], "signs_passed": 21, "distance_forward_mm": 23237.1, "distance_backward_mm": 703.8, "mean_speed_mm_s": 523.9, "frames_processed": 1454, "mean_pi_frame_ms": 5.64, "reverse_motor_s": 4.878, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 27650, "render_s": 1.413, "pi_s": 8.197, "other_s": 4.697, "wall_time_s": 14.307, "wall_per_sim_s": 0.313}
n1 s32 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 43.931, "turns_completed": 12, "tpr": 12, "lap_times_s": [16.21, 10.079, 10.373], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.854, "E": 6.748, "I": 0.868, "S": 35.973}, "collisions": [], "min_clearance_wall_mm": 38.15, "min_clearance_sign_mm": 83.36, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 25189.1, "distance_backward_mm": 328.2, "mean_speed_mm_s": 580.8, "frames_processed": 1417, "mean_pi_frame_ms": 3.59, "reverse_motor_s": 3.48, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 25496, "render_s": 1.434, "pi_s": 5.085, "other_s": 4.698, "wall_time_s": 11.217, "wall_per_sim_s": 0.255}
n1 s16 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 43.462, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.691, 9.96, 10.188], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.861, "E": 7.251, "I": 0.816, "S": 34.973}, "collisions": [], "min_clearance_wall_mm": 41.77, "min_clearance_sign_mm": 56.83, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24383.2, "distance_backward_mm": 329.4, "mean_speed_mm_s": 568.6, "frames_processed": 1411, "mean_pi_frame_ms": 3.67, "reverse_motor_s": 3.151, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 24713, "render_s": 1.422, "pi_s": 5.172, "other_s": 4.546, "wall_time_s": 11.14, "wall_per_sim_s": 0.256}
n2 s35 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 45.782, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.688, 9.51, 9.383], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.863, "E": 9.101, "I": 0.813, "S": 35.487}, "collisions": [], "min_clearance_wall_mm": 34.69, "min_clearance_sign_mm": 52.07, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 23106.0, "distance_backward_mm": 655.2, "mean_speed_mm_s": 519.0, "frames_processed": 1452, "mean_pi_frame_ms": 3.69, "reverse_motor_s": 5.578, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 27578, "render_s": 1.443, "pi_s": 5.355, "other_s": 4.866, "wall_time_s": 11.664, "wall_per_sim_s": 0.255}
n2 s32 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 42.828, "turns_completed": 12, "tpr": 12, "lap_times_s": [14.963, 9.9, 10.223], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.819, "E": 7.385, "I": 0.759, "S": 34.303}, "collisions": [], "min_clearance_wall_mm": 46.89, "min_clearance_sign_mm": 77.88, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24710.1, "distance_backward_mm": 336.8, "mean_speed_mm_s": 584.8, "frames_processed": 1378, "mean_pi_frame_ms": 3.39, "reverse_motor_s": 3.27, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 24437, "render_s": 1.379, "pi_s": 4.673, "other_s": 4.893, "wall_time_s": 10.945, "wall_per_sim_s": 0.256}
n2 s16 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 43.264, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.842, 9.991, 10.182], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.79, "E": 6.84, "I": 0.961, "S": 35.113}, "collisions": [], "min_clearance_wall_mm": 37.76, "min_clearance_sign_mm": 55.27, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24533.1, "distance_backward_mm": 327.0, "mean_speed_mm_s": 574.6, "frames_processed": 1399, "mean_pi_frame_ms": 3.81, "reverse_motor_s": 3.178, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 24790, "render_s": 1.472, "pi_s": 5.326, "other_s": 4.639, "wall_time_s": 11.437, "wall_per_sim_s": 0.264}

## Costo por carrera

COSTO n1 s35 {"FF": 2.8, "FD": 3.8, "cuadros": 1454, "media_ms": 16.225374458321838, "p50_ms": 11.883619999999693, "p95_ms": 26.737421199997865, "max_ms": 282.0735441999999, "pct_mas_25ms": 8.940852819807429, "dp_ms": [1950, 1950, 257, 126, 154, 186, 131, 145, 194, 122, 97, 104, 112, 99, 108, 87, 89, 112, 89, 85, 99]}
COSTO n1 s32 {"FF": 2.8, "FD": 3.8, "cuadros": 1417, "media_ms": 10.400592096541963, "p50_ms": 6.312824000001882, "p95_ms": 28.94862600000003, "max_ms": 283.0336953999995, "pct_mas_25ms": 6.633733239237827, "dp_ms": [1950, 1950, 248, 199, 148, 240, 110, 118, 111, 106, 83, 83, 71, 88, 104]}
COSTO n1 s16 {"FF": 2.8, "FD": 3.8, "cuadros": 1411, "media_ms": 10.559742430333086, "p50_ms": 6.9411384000027, "p95_ms": 25.860071999999512, "max_ms": 335.6136651999995, "pct_mas_25ms": 5.527994330262225, "dp_ms": [1950, 1950, 305, 126, 154, 132, 113, 100, 88, 105, 129, 100, 79, 91, 99]}
COSTO n2 s35 {"FF": 2.8, "FD": 3.8, "cuadros": 1452, "media_ms": 10.877331755096519, "p50_ms": 7.0699972000006985, "p95_ms": 24.266087999999186, "max_ms": 321.6797181999997, "pct_mas_25ms": 4.40771349862259, "dp_ms": [1950, 1950, 298, 114, 171, 114, 240, 124, 112, 124, 138, 184, 115, 113, 81, 146, 143, 81, 98, 114, 74, 82, 104]}
COSTO n2 s32 {"FF": 2.8, "FD": 3.8, "cuadros": 1378, "media_ms": 9.869298800725629, "p50_ms": 6.739501999998864, "p95_ms": 21.155259999999743, "max_ms": 318.2816356000003, "pct_mas_25ms": 3.265602322206096, "dp_ms": [1950, 1950, 294, 150, 124, 192, 110, 114, 111, 93, 105, 148, 81, 84, 73, 89, 105]}
COSTO n2 s16 {"FF": 2.8, "FD": 3.8, "cuadros": 1399, "media_ms": 10.975655574267325, "p50_ms": 7.249827200000779, "p95_ms": 26.805690800000015, "max_ms": 277.1620918000002, "pct_mas_25ms": 6.433166547533952, "dp_ms": [1950, 1950, 249, 124, 205, 147, 113, 108, 97, 122, 101, 90, 92, 92, 166]}

## Tiempo de reloj total

14.5 s (6 carreras, -P 7 = 1 oleada, warm-up fuera del cronómetro, 8 CPU en la caja; firmware SIL precompilado fuera del cronómetro)

media de media_ms (COSTO) entre carreras: 11.5 ms; carga máx durante la tanda: 1.46

## Validez

VÁLIDA. Corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio). Reloj: 14.5 s para 6 carreras con -P 7 (1 oleada). Carga máx 1.46, sin OOM/Killed/MemoryError. media_ms (COSTO) de 9.9 a 11.0 ms en 5 carreras y 16.2 ms en n1/s35 (la que chocó).

## Comparación (mismo tipo: carrera completa con estacionamiento geométrico, PARK_GEOM_EXT=1 + PARK_GEOM_ROJO=1)

Commit 2f23448: commit de tanda vacío (sin cambios de código) sobre 094f93c "Park GEOM: r13 rojoCerca + r14d stickyEarly". r16_wstop = misma base + PARK_GEOM_WSTOP=262. r15b y r14 son tandas anteriores de la misma familia, en otros commits.

| carrera | r14 | r15b | r16_wstop | s35_base |
|---|---|---|---|---|
| n1 s35 | 45.753 ok | 45.009 ok | 46.169 ok | collision 45.695 |
| n1 s32 | 44.936 ok | 43.795 ok | 45.230 ok | 43.931 ok |
| n1 s16 | 43.390 ok | - | 43.520 ok | 43.462 ok |
| n2 s35 | 46.325 ok | 45.140 ok | 45.821 ok | 45.782 ok |
| n2 s32 | 44.855 ok | 44.331 ok | 43.429 ok | 42.828 ok |
| n2 s16 | 43.540 ok | - | 43.708 ok | 43.264 ok |

- Falla: n1/s35, colisión con el "cajón magenta" a t=46.44 s, después de completar los 12 giros (lap_times 16.53/9.42/9.39). Es decir, en la maniobra de estacionamiento. Con WSTOP=262 (r16_wstop) esa misma carrera terminó (46.169 s), y también en r15b y r14.
- En las carreras que terminan en ambas: frente a r16_wstop, 5 carreras, -2.44 s; frente a r15b, 3 carreras, -0.73 s; frente a r14, 5 carreras, -3.78 s.
- La muestra es de 6 carreras; no es una tanda completa.
