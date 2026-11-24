# w_park_u_uturn185

Tipo de prueba: carrera completa con estacionamiento geométrico y U (latencia/uno.sh: 3 vueltas + estacionamiento) en el banco NUEVO con latencia. TWIN_FW_OVER PARK_GEOM_EXT=1, PARK_GEOM_ROJO=1, PARK_UTURN_PWM=185 (carrera_bench.json trae 170). TWIN_PI_EXTRA ROUTE_PARK_GEOM_EXT=1, TWIN_MOTOR_REAL=1, VBAT 11.5, cámara desde calibración + lens_calib.npz, COST_FIRST 1.95, FF 2.8 / FD 3.8, n1-2 x s16 s20 s32 (6 carreras), -P 7. Commit 5ec9fa0 (origin/grok/t/w_park_u_uturn185): commit vacío sobre 2398305 (Park GEOM r13 rojoCerca + r14d stickyEarly).

## Resumen (resumen.py)

w_park_u_uturn185: 5/6 ok  media 41.33  mediana 41.60  mejor 37.62  fallas: n2/s32

## Carreras (última línea STOP)

n1 s16 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 41.602, "turns_completed": 12, "tpr": 12, "lap_times_s": [14.592, 9.72, 10.062], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.779, "E": 6.84, "I": 0.817, "S": 33.605}, "collisions": [], "min_clearance_wall_mm": 52.15, "min_clearance_sign_mm": 44.41, "pass_side_violations": [], "signs_passed": 20, "distance_forward_mm": 24053.5, "distance_backward_mm": 324.0, "mean_speed_mm_s": 586.0, "frames_processed": 1352, "mean_pi_frame_ms": 3.85, "reverse_motor_s": 3.107, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 23825, "render_s": 1.443, "pi_s": 5.208, "other_s": 7.256, "wall_time_s": 13.907, "wall_per_sim_s": 0.334}
n1 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 37.623, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.327, 8.406, 8.525], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.78, "E": 7.244, "I": 0.767, "S": 29.272}, "collisions": [], "min_clearance_wall_mm": 42.55, "min_clearance_sign_mm": 107.98, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 20844.6, "distance_backward_mm": 343.2, "mean_speed_mm_s": 563.2, "frames_processed": 1219, "mean_pi_frame_ms": 3.94, "reverse_motor_s": 3.239, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 21694, "render_s": 1.391, "pi_s": 4.798, "other_s": 7.226, "wall_time_s": 13.415, "wall_per_sim_s": 0.357}
n1 s32 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 43.442, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.156, 10.08, 10.219], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.779, "E": 7.64, "I": 0.806, "S": 34.656}, "collisions": [], "min_clearance_wall_mm": 36.89, "min_clearance_sign_mm": 86.49, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 25059.3, "distance_backward_mm": 358.9, "mean_speed_mm_s": 585.1, "frames_processed": 1404, "mean_pi_frame_ms": 3.5, "reverse_motor_s": 3.509, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 24850, "render_s": 1.493, "pi_s": 4.914, "other_s": 7.425, "wall_time_s": 13.832, "wall_per_sim_s": 0.318}
n2 s16 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 43.008, "turns_completed": 12, "tpr": 12, "lap_times_s": [14.81, 9.95, 10.201], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.779, "E": 7.708, "I": 0.809, "S": 34.152}, "collisions": [], "min_clearance_wall_mm": 40.62, "min_clearance_sign_mm": 56.89, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24518.4, "distance_backward_mm": 365.1, "mean_speed_mm_s": 578.6, "frames_processed": 1391, "mean_pi_frame_ms": 3.83, "reverse_motor_s": 3.569, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 24533, "render_s": 1.487, "pi_s": 5.33, "other_s": 7.485, "wall_time_s": 14.302, "wall_per_sim_s": 0.333}
n2 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 40.983, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.531, 8.352, 8.486], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.779, "E": 10.434, "I": 0.799, "S": 29.411}, "collisions": [], "min_clearance_wall_mm": 33.41, "min_clearance_sign_mm": 119.44, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 21086.0, "distance_backward_mm": 539.8, "mean_speed_mm_s": 527.7, "frames_processed": 1331, "mean_pi_frame_ms": 3.79, "reverse_motor_s": 5.207, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 23840, "render_s": 1.469, "pi_s": 5.038, "other_s": 7.445, "wall_time_s": 13.952, "wall_per_sim_s": 0.34}
n2 s32 STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 42.451, "turns_completed": 12, "tpr": 12, "lap_times_s": [16.373, 9.9, 10.178], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.779, "E": 5.399, "I": 0.815, "S": 35.899}, "collisions": [{"cause": "cajón magenta", "t": 43.162687}], "min_clearance_wall_mm": 71.78, "min_clearance_sign_mm": 82.41, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24678.5, "distance_backward_mm": 492.0, "mean_speed_mm_s": 592.9, "frames_processed": 1365, "mean_pi_frame_ms": 3.62, "reverse_motor_s": 3.779, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 25302, "render_s": 1.477, "pi_s": 4.944, "other_s": 7.429, "wall_time_s": 13.85, "wall_per_sim_s": 0.326}

## Costo por carrera

COSTO n1 s16 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1352, "media_ms": 11.131772928402402, "p50_ms": 7.804196400001118, "p95_ms": 26.447570799999195, "max_ms": 266.89402599999966, "pct_mas_25ms": 6.952662721893491, "dp_ms": [1950, 1950, 242, 132, 132, 121, 126, 105, 84, 103, 90, 101, 96, 179]}
COSTO n1 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1219, "media_ms": 11.314869077112325, "p50_ms": 7.896324799996534, "p95_ms": 26.44156200000065, "max_ms": 267.6072561999999, "pct_mas_25ms": 6.480721903199344, "dp_ms": [1950, 1950, 243, 137, 124, 106, 118, 92, 80, 68, 107, 82, 94, 88, 108]}
COSTO n1 s32 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1404, "media_ms": 10.15967133803419, "p50_ms": 7.331665599997805, "p95_ms": 21.02361239999979, "max_ms": 279.2772694000002, "pct_mas_25ms": 3.0626780626780628, "dp_ms": [1950, 1950, 245, 141, 128, 174, 116, 112, 120, 105, 107, 95, 100, 101, 129]}
COSTO n2 s16 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1391, "media_ms": 11.099628020416974, "p50_ms": 7.666229200003016, "p95_ms": 28.30174479999989, "max_ms": 289.06029119999977, "pct_mas_25ms": 6.9015097052480225, "dp_ms": [1950, 1950, 263, 127, 119, 121, 97, 104, 88, 95, 99, 85, 96, 91, 89, 92, 164]}
COSTO n2 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1331, "media_ms": 10.881312289707004, "p50_ms": 7.7869903999999925, "p95_ms": 25.921873599999934, "max_ms": 266.8224975999996, "pct_mas_25ms": 5.860255447032307, "dp_ms": [1950, 1950, 242, 191, 116, 109, 96, 78, 70, 92, 97, 89, 93, 103]}
COSTO n2 s32 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1365, "media_ms": 10.556729737142854, "p50_ms": 7.06697039999895, "p95_ms": 22.313093600000222, "max_ms": 279.8759928, "pct_mas_25ms": 3.5897435897435894, "dp_ms": [1950, 1950, 255, 145, 130, 208, 117, 124, 123, 96, 104, 105, 90, 88, 77, 91, 112]}

## Tiempo de reloj total

14.6 s

## Validez

VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx 2.84, mem máx 10288 MB, 0 OOM/Killed/MemoryError, mean_pi_frame_ms ~3.6. Reloj 14.6 s para 6 carreras (w_park_u_base 11.3 s, w_park_u_scan245 16.1 s).

## Comparación (mismo tipo, mismas 6 semillas, mismas banderas salvo la perilla)

| carrera | w_park_u_base (PWM U 170 por defecto) | w_park_u_scan245 | w_park_u_uturn185 |
|---|---|---|---|
| n1/s16 | 42.986 | 42.411 | 41.602 |
| n1/s20 | choque 33.162 | 37.306 | 37.623 |
| n1/s32 | 44.235 | 44.785 | 43.442 |
| n2/s16 | 43.224 | 45.174 | 43.008 |
| n2/s20 | 37.746 | 37.468 | 40.983 |
| n2/s32 | 44.005 | 44.279 | choque (cajón magenta, t=43.16) |

- En las 4 que terminan en base y uturn185: +0.84 s en total frente a base.
- En las 5 que terminan en scan245 y uturn185: -0.49 s en total frente a scan245.
