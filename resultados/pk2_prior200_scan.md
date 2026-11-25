# pk2_prior200_scan

Tipo de prueba: carrera completa con estacionamiento (latencia/uno.sh, FF 2.8 / FD 3.8), banco NUEVO con latencia, campo por defecto, variante pk2_prior200_scan; TWIN_CAMERA_FROM_CALIB=1, TWIN_MOTOR_REAL=1, MOTOR_CURVA_REAL=1, VBAT 11.5, TOF_SHORT_K 0.5, COST_FIJO=1, COST_FIRST 1.95, MOTOR_CURVE_LIVE false, n1-2 x s20-21 (4 carreras), -P 7. Commit aa7ac1c2 (origin/grok/t/pk2_prior200_scan).

## Resumen (resumen.py)

pk2_prior200_scan: 3/4 ok  media 32.88  mediana 32.74  mejor 32.26  fallas: n2/s21

## Carreras (última línea STOP)

n1 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 32.743, "turns_completed": 12, "tpr": 12, "lap_times_s": [11.446, 7.156, 7.755], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.577, "E": 6.223, "I": 0.861, "S": 25.521}, "collisions": [], "min_clearance_wall_mm": 44.17, "min_clearance_sign_mm": 64.79, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 20533.4, "distance_backward_mm": 333.2, "mean_speed_mm_s": 637.3, "frames_processed": 1101, "mean_pi_frame_ms": 3.95, "reverse_motor_s": 3.54, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 19284, "render_s": 1.164, "pi_s": 4.35, "other_s": 6.225, "wall_time_s": 11.739, "wall_per_sim_s": 0.359}
n1 s21 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 33.634, "turns_completed": 12, "tpr": 12, "lap_times_s": [12.222, 7.702, 7.632], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.577, "E": 5.875, "I": 0.861, "S": 26.759}, "collisions": [], "min_clearance_wall_mm": 43.89, "min_clearance_sign_mm": 59.98, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 21868.7, "distance_backward_mm": 316.5, "mean_speed_mm_s": 659.6, "frames_processed": 1128, "mean_pi_frame_ms": 5.83, "reverse_motor_s": 2.851, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 19400, "render_s": 1.26, "pi_s": 6.58, "other_s": 6.377, "wall_time_s": 14.218, "wall_per_sim_s": 0.423}
n2 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 32.256, "turns_completed": 12, "tpr": 12, "lap_times_s": [11.333, 7.171, 7.587], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.658, "E": 6.031, "I": 0.842, "S": 25.219}, "collisions": [], "min_clearance_wall_mm": 42.02, "min_clearance_sign_mm": 95.27, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 20895.9, "distance_backward_mm": 334.5, "mean_speed_mm_s": 658.2, "frames_processed": 1083, "mean_pi_frame_ms": 3.97, "reverse_motor_s": 3.194, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 18964, "render_s": 1.177, "pi_s": 4.301, "other_s": 6.486, "wall_time_s": 11.963, "wall_per_sim_s": 0.371}
n2 s21 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 34.806, "turns_completed": 12, "tpr": 12, "lap_times_s": [12.231, 7.743, 8.039], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.577, "E": 6.313, "I": 0.895, "S": 27.463}, "collisions": [], "min_clearance_wall_mm": 63.44, "min_clearance_sign_mm": 41.23, "pass_side_violations": [], "signs_passed": 14, "distance_forward_mm": 22124.3, "distance_backward_mm": 359.3, "mean_speed_mm_s": 646.0, "frames_processed": 1166, "mean_pi_frame_ms": 5.76, "reverse_motor_s": 2.738, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 19376, "render_s": 1.235, "pi_s": 6.713, "other_s": 6.388, "wall_time_s": 14.337, "wall_per_sim_s": 0.412}

## Costo por carrera

COSTO n1 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1101, "media_ms": 8.596059369319248, "p50_ms": 6.935405121299896, "p95_ms": 19.35785134402609, "max_ms": 53.28156855313167, "pct_mas_25ms": 2.0890099909173476, "dp_ms": [1950, 1950, 36, 25, 43, 27, 26, 30, 31, 18, 43, 41, 31, 37, 38, 40]}
COSTO n1 s21 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1128, "media_ms": 8.93095038534955, "p50_ms": 7.187094926983125, "p95_ms": 19.95385105134439, "max_ms": 75.07438028176624, "pct_mas_25ms": 2.6595744680851063, "dp_ms": [1950, 1950, 26, 24, 36, 34, 58, 36, 28, 36, 35]}
COSTO n2 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1083, "media_ms": 8.741171686089828, "p50_ms": 6.9975258201994475, "p95_ms": 19.19623339024814, "max_ms": 84.6282986547352, "pct_mas_25ms": 2.21606648199446, "dp_ms": [1950, 1950, 26, 25, 40, 27, 41, 22, 28, 41, 31, 20, 24, 78]}
COSTO n2 s21 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1166, "media_ms": 9.13393584514249, "p50_ms": 7.311000907763476, "p95_ms": 21.452037093386835, "max_ms": 80.92755882963544, "pct_mas_25ms": 3.173241852487135, "dp_ms": [1950, 1950, 35, 59, 37, 27, 29, 37, 35, 48, 15, 31, 23, 44]}

## Tiempo de reloj total

14.6 s

## Validez

VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 14.6 s para 4 carreras (referencia ~10-20 s sin carga), carga máx 7.24, mem máx 10069 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.95-5.83 (media 4.88).
