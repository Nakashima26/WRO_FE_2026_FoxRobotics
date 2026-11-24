# w_park_u_base_s20

Tipo de prueba: carrera completa con estacionamiento geométrico y U (latencia/uno.sh), banco NUEVO con latencia; mismo tipo que w_park_u_base / w_park_u_abort185 / w_park_u_abort185_s20. TWIN_FW_OVER PARK_GEOM_EXT=1, PARK_GEOM_ROJO=1 (sin PARK_UTURN_ABORT_PWM); TWIN_PI_EXTRA ROUTE_PARK_GEOM_EXT=1; TWIN_MOTOR_REAL=1, VBAT 11.5, cámara desde calibración + lens_calib.npz, COST_FIRST 1.95, FF 2.8 / FD 3.8, n1-4 x s20 (4 carreras), -P 7. Commit e0bbb78 (origin/grok/t/w_park_u_base_s20).

## Resumen (resumen.py)

w_park_u_base_s20: 4/4 ok  media 37.49  mediana 37.48  mejor 36.89  fallas:

## Carreras (última línea STOP)

n1 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 37.306, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.325, 8.308, 8.38], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.739, "E": 7.094, "I": 0.818, "S": 29.094}, "collisions": [], "min_clearance_wall_mm": 39.03, "min_clearance_sign_mm": 93.61, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 20677.9, "distance_backward_mm": 345.3, "mean_speed_mm_s": 563.5, "frames_processed": 1212, "mean_pi_frame_ms": 3.71, "reverse_motor_s": 3.181, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 21457, "render_s": 1.246, "pi_s": 4.492, "other_s": 6.259, "wall_time_s": 11.996, "wall_per_sim_s": 0.322}
n2 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 36.893, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.413, 8.309, 8.497], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.738, "E": 6.496, "I": 0.837, "S": 29.261}, "collisions": [], "min_clearance_wall_mm": 41.62, "min_clearance_sign_mm": 116.62, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 20864.4, "distance_backward_mm": 327.3, "mean_speed_mm_s": 574.4, "frames_processed": 1205, "mean_pi_frame_ms": 3.53, "reverse_motor_s": 3.121, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 21341, "render_s": 1.235, "pi_s": 4.259, "other_s": 6.1, "wall_time_s": 11.594, "wall_per_sim_s": 0.314}
n3 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 37.651, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.604, 8.304, 8.378], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.82, "E": 7.193, "I": 0.75, "S": 29.328}, "collisions": [], "min_clearance_wall_mm": 40.65, "min_clearance_sign_mm": 95.05, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 20686.6, "distance_backward_mm": 349.4, "mean_speed_mm_s": 558.7, "frames_processed": 1217, "mean_pi_frame_ms": 3.64, "reverse_motor_s": 3.27, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 21681, "render_s": 1.218, "pi_s": 4.429, "other_s": 6.214, "wall_time_s": 11.861, "wall_per_sim_s": 0.315}
n4 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 38.125, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.249, 8.242, 8.345], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.738, "E": 8.111, "I": 0.813, "S": 28.902}, "collisions": [], "min_clearance_wall_mm": 43.64, "min_clearance_sign_mm": 89.02, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 20494.5, "distance_backward_mm": 329.3, "mean_speed_mm_s": 546.2, "frames_processed": 1245, "mean_pi_frame_ms": 3.45, "reverse_motor_s": 3.781, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 22308, "render_s": 1.239, "pi_s": 4.291, "other_s": 6.101, "wall_time_s": 11.631, "wall_per_sim_s": 0.305}

## Costo por carrera

COSTO n1 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1212, "media_ms": 10.718742012211234, "p50_ms": 7.0343476000005, "p95_ms": 26.822387200000005, "max_ms": 247.61558060000016, "pct_mas_25ms": 6.435643564356436, "dp_ms": [1950, 1950, 226, 125, 113, 95, 122, 97, 79, 75, 88, 66, 103, 81, 88]}
COSTO n2 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1205, "media_ms": 10.181449779751034, "p50_ms": 6.9177807999999175, "p95_ms": 24.546653599999324, "max_ms": 235.88108260000027, "pct_mas_25ms": 4.813278008298756, "dp_ms": [1950, 1950, 213, 124, 113, 134, 82, 99, 85, 77, 80, 87]}
COSTO n3 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1217, "media_ms": 10.528668395069886, "p50_ms": 6.831190800002672, "p95_ms": 25.66189639999976, "max_ms": 318.1298602000004, "pct_mas_25ms": 5.751848808545604, "dp_ms": [1950, 1950, 288, 124, 106, 95, 124, 86, 89, 89, 67, 97, 77, 86]}
COSTO n4 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1245, "media_ms": 9.952978981204804, "p50_ms": 6.70614840000212, "p95_ms": 24.508447599998995, "max_ms": 241.27867839999973, "pct_mas_25ms": 4.578313253012048, "dp_ms": [1950, 1950, 219, 119, 109, 115, 77, 79, 67, 113, 73, 78, 73, 86]}

## Tiempo de reloj total

12.3 s

## Validez

VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx 0.95, mem máx 3697 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.45-3.71.
