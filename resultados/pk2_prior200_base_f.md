# pk2_prior200_base_f

Tipo de prueba: carrera completa con estacionamiento (latencia/uno.sh, FF 2.8 / FD 3.8), banco NUEVO con latencia, parche pk2_prior200_base; TWIN_CAMERA_FROM_CALIB=1, TWIN_MOTOR_REAL=1, MOTOR_CURVA_REAL=1, VBAT 11.5, TOF_SHORT_K 0.5, COST_FIJO=1, COST_FIRST 1.95, MOTOR_CURVE_LIVE "false" (texto; rerun de pk2_prior200_base, que salió 0/384 ERROR por False en C++), n1-2 x s20-21 (4 carreras), -P 7. Commit 75051700 (origin/grok/t/pk2_prior200_base_f).

## Resumen (resumen.py)

pk2_prior200_base_f: 0/4 ok  fallas: n1/s20 n1/s21 n2/s20 n2/s21

## Carreras (última línea STOP)

n1 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 33.445, "turns_completed": 12, "tpr": 12, "lap_times_s": [11.446, 7.156, 7.755], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.577, "E": 6.926, "I": 0.861, "S": 25.519}, "collisions": [], "min_clearance_wall_mm": 68.95, "min_clearance_sign_mm": 64.79, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 20854.3, "distance_backward_mm": 329.9, "mean_speed_mm_s": 633.4, "frames_processed": 1124, "mean_pi_frame_ms": 3.83, "reverse_motor_s": 2.814, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 18761, "render_s": 1.136, "pi_s": 4.305, "other_s": 5.766, "wall_time_s": 11.206, "wall_per_sim_s": 0.335}
n1 s21 STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 32.807, "turns_completed": 12, "tpr": 12, "lap_times_s": [12.222, 7.702, 7.632], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.577, "E": 5.048, "I": 0.861, "S": 26.759}, "collisions": [{"cause": "cajón magenta", "t": 33.52421}], "min_clearance_wall_mm": 71.71, "min_clearance_sign_mm": 59.98, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 21966.8, "distance_backward_mm": 125.0, "mean_speed_mm_s": 673.4, "frames_processed": 1101, "mean_pi_frame_ms": 5.59, "reverse_motor_s": 1.83, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 18622, "render_s": 1.158, "pi_s": 6.15, "other_s": 5.909, "wall_time_s": 13.216, "wall_per_sim_s": 0.403}
n2 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 34.551, "turns_completed": 12, "tpr": 12, "lap_times_s": [11.333, 7.171, 7.587], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.658, "E": 8.326, "I": 0.842, "S": 25.219}, "collisions": [], "min_clearance_wall_mm": 62.51, "min_clearance_sign_mm": 95.27, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 21252.5, "distance_backward_mm": 350.5, "mean_speed_mm_s": 625.3, "frames_processed": 1159, "mean_pi_frame_ms": 3.71, "reverse_motor_s": 2.911, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 19455, "render_s": 1.18, "pi_s": 4.299, "other_s": 5.885, "wall_time_s": 11.363, "wall_per_sim_s": 0.329}
n2 s21 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 34.806, "turns_completed": 12, "tpr": 12, "lap_times_s": [12.231, 7.743, 8.039], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.577, "E": 6.313, "I": 0.895, "S": 27.463}, "collisions": [], "min_clearance_wall_mm": 63.44, "min_clearance_sign_mm": 41.23, "pass_side_violations": [], "signs_passed": 14, "distance_forward_mm": 22124.3, "distance_backward_mm": 359.3, "mean_speed_mm_s": 646.0, "frames_processed": 1166, "mean_pi_frame_ms": 5.77, "reverse_motor_s": 2.738, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 19376, "render_s": 1.248, "pi_s": 6.725, "other_s": 6.007, "wall_time_s": 13.98, "wall_per_sim_s": 0.402}

## Costo por carrera

COSTO n1 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1124, "media_ms": 8.571718913430741, "p50_ms": 6.9156988232971175, "p95_ms": 19.35785134402609, "max_ms": 53.28156855313167, "pct_mas_25ms": 2.135231316725979, "dp_ms": [1950, 1950, 36, 25, 43, 27, 26, 30, 31, 18, 43, 41, 31, 37, 38, 40]}
COSTO n1 s21 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1101, "media_ms": 8.905528780194764, "p50_ms": 7.187094926983125, "p95_ms": 19.768712800514418, "max_ms": 75.07438028176624, "pct_mas_25ms": 2.5431425976385107, "dp_ms": [1950, 1950, 26, 24, 36, 34, 58, 36, 28, 36, 35]}
COSTO n2 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1159, "media_ms": 8.705271451733507, "p50_ms": 6.988981125282044, "p95_ms": 19.143434062124026, "max_ms": 84.6282986547352, "pct_mas_25ms": 2.1570319240724762, "dp_ms": [1950, 1950, 26, 25, 40, 27, 41, 22, 28, 41, 31, 20, 24, 78]}
COSTO n2 s21 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1166, "media_ms": 9.13393584514249, "p50_ms": 7.311000907763476, "p95_ms": 21.452037093386835, "max_ms": 80.92755882963544, "pct_mas_25ms": 3.173241852487135, "dp_ms": [1950, 1950, 35, 59, 37, 27, 29, 37, 35, 48, 15, 31, 23, 44]}

## Tiempo de reloj total

14.2 s

## Validez

VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 14.2 s para 4 carreras (referencia ~10-20 s sin carga), carga máx 6.39, mem máx 3741 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.71-5.77 (media 4.72).
