# w_geom_rojo_r15b

Tipo de prueba: carrera completa con estacionamiento (latencia/uno.sh: 3 vueltas + estacionamiento, FOX_PARK_POS=random) con estacionamiento geométrico (TWIN_FW_OVER='{"PARK_GEOM_EXT":"1","PARK_GEOM_ROJO":"1"}', TWIN_PI_EXTRA='{"ROUTE_PARK_GEOM_EXT":1}') en el banco NUEVO con latencia (latencia/uno.sh + costo.py, FF 2.8 / FD 3.8, primer plan 1.95 s, TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, TWIN_CAMERA_FROM_CALIB=1, TWIN_LENS_CALIB=pure_pursuit/lens_calib.npz), ruido 1-2 x semillas 5 32 35 36 (8 carreras), xargs -P 7, commit 9f0bbc1 (origin/grok/t/w_geom_rojo_r15b). Mismo tipo y misma configuración que w_geom_rojo_r14 y w_geom_rojo_r15_s5; comparar solo contra tandas de ese tipo.

## Resumen (resumen.py)

w_geom_rojo_r15b: 7/8 ok  media 45.02  mediana 45.01  mejor 43.80  fallas: n2/s36

## Carreras (última línea STOP)

n1 s5 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 46.416, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.259, 9.81, 9.556], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.855, "E": 9.391, "I": 0.818, "S": 35.867}, "collisions": [], "min_clearance_wall_mm": 49.06, "min_clearance_sign_mm": 63.6, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24479.9, "distance_backward_mm": 874.5, "mean_speed_mm_s": 546.2, "frames_processed": 1514, "mean_pi_frame_ms": 3.49, "reverse_motor_s": 5.728, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 27830, "render_s": 1.461, "pi_s": 5.289, "other_s": 3.352, "wall_time_s": 10.102, "wall_per_sim_s": 0.218}
n1 s32 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 43.795, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.509, 10.111, 10.308], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.8, "E": 7.554, "I": 0.798, "S": 35.099}, "collisions": [], "min_clearance_wall_mm": 40.61, "min_clearance_sign_mm": 87.45, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 25046.1, "distance_backward_mm": 352.5, "mean_speed_mm_s": 579.9, "frames_processed": 1410, "mean_pi_frame_ms": 3.27, "reverse_motor_s": 3.479, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 25066, "render_s": 1.393, "pi_s": 4.607, "other_s": 3.273, "wall_time_s": 9.273, "wall_per_sim_s": 0.212}
n1 s35 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 45.009, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.409, 9.065, 8.86], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.885, "E": 9.606, "I": 0.761, "S": 34.26}, "collisions": [], "min_clearance_wall_mm": 34.2, "min_clearance_sign_mm": 71.95, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 23652.6, "distance_backward_mm": 642.5, "mean_speed_mm_s": 539.8, "frames_processed": 1455, "mean_pi_frame_ms": 5.39, "reverse_motor_s": 5.491, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 27077, "render_s": 1.424, "pi_s": 7.847, "other_s": 3.37, "wall_time_s": 12.641, "wall_per_sim_s": 0.281}
n1 s36 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 44.382, "turns_completed": 12, "tpr": 12, "lap_times_s": [14.594, 9.754, 9.094], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.812, "E": 8.809, "I": 0.798, "S": 34.43}, "collisions": [], "min_clearance_wall_mm": 45.74, "min_clearance_sign_mm": 58.84, "pass_side_violations": [], "signs_passed": 18, "distance_forward_mm": 22765.6, "distance_backward_mm": 787.1, "mean_speed_mm_s": 530.7, "frames_processed": 1438, "mean_pi_frame_ms": 3.5, "reverse_motor_s": 5.309, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 26556, "render_s": 1.413, "pi_s": 5.03, "other_s": 3.295, "wall_time_s": 9.739, "wall_per_sim_s": 0.219}
n2 s5 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 46.041, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.423, 9.84, 9.525], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.855, "E": 8.854, "I": 0.799, "S": 36.045}, "collisions": [], "min_clearance_wall_mm": 32.05, "min_clearance_sign_mm": 41.26, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24426.2, "distance_backward_mm": 1048.0, "mean_speed_mm_s": 553.3, "frames_processed": 1492, "mean_pi_frame_ms": 3.65, "reverse_motor_s": 5.91, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 27648, "render_s": 1.446, "pi_s": 5.446, "other_s": 3.408, "wall_time_s": 10.299, "wall_per_sim_s": 0.224}
n2 s32 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 44.331, "turns_completed": 12, "tpr": 12, "lap_times_s": [16.268, 10.08, 10.182], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.801, "E": 7.449, "I": 0.801, "S": 35.737}, "collisions": [], "min_clearance_wall_mm": 40.18, "min_clearance_sign_mm": 64.56, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24908.0, "distance_backward_mm": 344.8, "mean_speed_mm_s": 569.6, "frames_processed": 1429, "mean_pi_frame_ms": 3.4, "reverse_motor_s": 3.359, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 25344, "render_s": 1.411, "pi_s": 4.858, "other_s": 3.273, "wall_time_s": 9.541, "wall_per_sim_s": 0.215}
n2 s35 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 45.14, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.637, 8.939, 8.713], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.79, "E": 9.75, "I": 0.943, "S": 34.108}, "collisions": [], "min_clearance_wall_mm": 37.0, "min_clearance_sign_mm": 68.73, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 23387.2, "distance_backward_mm": 752.5, "mean_speed_mm_s": 534.8, "frames_processed": 1452, "mean_pi_frame_ms": 3.56, "reverse_motor_s": 5.4, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 27215, "render_s": 1.395, "pi_s": 5.164, "other_s": 3.459, "wall_time_s": 10.018, "wall_per_sim_s": 0.222}
n2 s36 STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 6.627, "turns_completed": 1, "tpr": 12, "lap_times_s": [], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.779, "I": 0.811, "S": 5.477}, "collisions": [{"cause": "señal Green S/T1", "t": 7.34024}], "min_clearance_wall_mm": 71.65, "min_clearance_sign_mm": 106.86, "pass_side_violations": [], "signs_passed": 1, "distance_forward_mm": 1553.3, "distance_backward_mm": 0.0, "mean_speed_mm_s": 234.4, "frames_processed": 206, "mean_pi_frame_ms": 7.39, "reverse_motor_s": 0.0, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 3887, "render_s": 0.21, "pi_s": 1.522, "other_s": 1.241, "wall_time_s": 2.974, "wall_per_sim_s": 0.449}

## Costo por carrera

COSTO n1 s5 {"FF": 2.8, "FD": 3.8, "cuadros": 1514, "media_ms": 10.018866316380443, "p50_ms": 7.009578800000326, "p95_ms": 24.436199200000882, "max_ms": 269.41190520000004, "pct_mas_25ms": 4.7556142668428, "dp_ms": [1950, 1950, 245, 126, 128, 181, 122, 119, 109, 105, 93, 92, 88, 88, 95]}
COSTO n1 s32 {"FF": 2.8, "FD": 3.8, "cuadros": 1410, "media_ms": 9.52437597843973, "p50_ms": 6.188518000000087, "p95_ms": 21.113439199999995, "max_ms": 275.1281740000003, "pct_mas_25ms": 3.546099290780142, "dp_ms": [1950, 1950, 251, 136, 134, 176, 120, 116, 118, 97, 102, 102, 94, 86, 72, 97, 90, 90]}
COSTO n1 s35 {"FF": 2.8, "FD": 3.8, "cuadros": 1455, "media_ms": 15.441119702955412, "p50_ms": 11.903847200002105, "p95_ms": 27.242387199999026, "max_ms": 301.6097042000002, "pct_mas_25ms": 8.45360824742268, "dp_ms": [1950, 1950, 268, 135, 132, 137, 135, 108, 191, 119, 99, 106, 104, 100, 81, 85, 87, 88]}
COSTO n1 s36 {"FF": 2.8, "FD": 3.8, "cuadros": 1438, "media_ms": 10.149765328233718, "p50_ms": 6.984961200000228, "p95_ms": 23.337781599999907, "max_ms": 276.262418, "pct_mas_25ms": 4.033379694019471, "dp_ms": [1950, 1950, 252, 128, 135, 108, 242, 113, 121, 95, 104, 96, 67, 69, 79, 86, 85]}
COSTO n2 s5 {"FF": 2.8, "FD": 3.8, "cuadros": 1492, "media_ms": 10.540133152144762, "p50_ms": 7.051665599998458, "p95_ms": 26.144476400000194, "max_ms": 274.8679504000001, "pct_mas_25ms": 5.697050938337802, "dp_ms": [1950, 1950, 249, 134, 127, 190, 126, 110, 109, 109, 87, 95, 95, 71, 79, 94]}
COSTO n2 s32 {"FF": 2.8, "FD": 3.8, "cuadros": 1429, "media_ms": 9.872836685794235, "p50_ms": 6.762851200000952, "p95_ms": 21.931672000000013, "max_ms": 266.2149856, "pct_mas_25ms": 3.778866340097971, "dp_ms": [1950, 1950, 243, 137, 145, 198, 116, 115, 115, 99, 83, 103, 80, 84, 71, 94, 94]}
COSTO n2 s35 {"FF": 2.8, "FD": 3.8, "cuadros": 1452, "media_ms": 10.389298522176302, "p50_ms": 6.954805199999469, "p95_ms": 22.18067880000021, "max_ms": 270.17766100000017, "pct_mas_25ms": 3.443526170798898, "dp_ms": [1950, 1950, 245, 126, 155, 114, 136, 141, 191, 114, 117, 79, 104, 100, 79, 94, 101, 70, 88, 95]}
COSTO n2 s36 {"FF": 2.8, "FD": 3.8, "cuadros": 206, "media_ms": 21.97913869611656, "p50_ms": 15.099758799999739, "p95_ms": 31.88767680000044, "max_ms": 249.92868700000045, "pct_mas_25ms": 21.35922330097087, "dp_ms": [1950, 1950, 228, 109, 131, 98, 196, 102, 114]}

## Tiempo de reloj total

12.8 s (8 carreras, -P 7 = 2 oleadas, 8 CPU en la caja; warm-up fuera del cronómetro; firmware SIL precompilado fuera del cronómetro)

media de media_ms (COSTO) entre carreras: 12.2 ms; carga máx durante la tanda: 1.75

## Validez

VÁLIDA. Corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), reloj 12.8 s para 8 carreras con -P 7 (2 oleadas; esperado ~20 s), carga máx 1.75, sin OOM/Killed/MemoryError. media_ms (COSTO) 9.5-10.5 ms en 6 de 8 carreras; n1/s35 15.4 ms (mean_pi_frame_ms 5.39, igual que r14 n2/s35 con 5.59) y n2/s36 22.0 ms (carrera de 6.6 s cortada por choque temprano: con pocos cuadros la media sube, igual que r14 n1/s36 con 7.5 ms). Media de las 8: 12.2 ms; sin n2/s36: 10.8 ms.

## Comparación (mismo tipo y misma configuración)

r14 = w_geom_rojo_r14 (5a19b40, n1-2 x s1-48), r15_s5 = w_geom_rojo_r15_s5 (3fa24ef, n1-4 x s5 s21 s32). r15b = r15 + stickyEarly solo si pka>=100.

| carrera | r14 | r15_s5 | r15b |
|---|---|---|---|
| n1 s5 | 45.458 ok | 45.398 ok | 46.416 ok |
| n1 s32 | 44.936 ok | 43.284 ok | 43.795 ok |
| n1 s35 | 45.753 ok | - | 45.009 ok |
| n1 s36 | collision 6.555 (señal Green S/T1, 1 giro) | - | 44.382 ok |
| n2 s5 | 46.852 ok | 46.336 ok | 46.041 ok |
| n2 s32 | 44.855 ok | 43.646 ok | 44.331 ok |
| n2 s35 | 46.325 ok | - | 45.140 ok |
| n2 s36 | 44.801 ok | - | collision 6.627 (señal Green S/T1, 1 giro) |

- La única falla (n2/s36) es el mismo choque temprano con la señal verde de S/T1 a ~6.6 s que r14 tuvo en n1/s36: pasa antes del estacionamiento, así que no lo causa el parche. Cambió de ruido (n1 a n2).
- s35: n1 y n2 terminan bien y bajan 0.74 y 1.19 s frente a r14. n1/s36, que en r14 chocaba temprano, ahora termina bien (44.38 s).
- s5/s32 frente a r15_s5: n2/s5 baja 0.30 s; n1/s5 sube 1.02 s, n1/s32 sube 0.51 s y n2/s32 sube 0.69 s. Siguen todas ok. Frente a r14, n1/s5 queda 0.96 s arriba; las otras 3 quedan debajo.
- Muestra de 8 carreras; no es una tanda completa.
