# w_park_u_abort185_s20

Tipo de prueba: carrera completa con estacionamiento geométrico y U (latencia/uno.sh), banco NUEVO con latencia; mismo tipo que w_park_u_abort185 / w_park_u_uturn185 / w_park_u_base. TWIN_FW_OVER PARK_GEOM_EXT=1, PARK_GEOM_ROJO=1, PARK_UTURN_ABORT_PWM=185; TWIN_PI_EXTRA ROUTE_PARK_GEOM_EXT=1; TWIN_MOTOR_REAL=1, VBAT 11.5, cámara desde calibración + lens_calib.npz, COST_FIRST 1.95, FF 2.8 / FD 3.8, n1-4 x s20 (4 carreras), -P 7. Commit 3ce6497 (origin/grok/t/w_park_u_abort185_s20).

## Resumen (resumen.py)

w_park_u_abort185_s20: 3/4 ok  media 37.49  mediana 37.80  mejor 36.46  fallas: n4/s20

## Carreras (última línea STOP)

n1 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 37.802, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.381, 8.394, 8.554], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.78, "E": 7.335, "I": 0.789, "S": 29.337}, "collisions": [], "min_clearance_wall_mm": 42.37, "min_clearance_sign_mm": 117.93, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 20885.2, "distance_backward_mm": 347.4, "mean_speed_mm_s": 561.7, "frames_processed": 1229, "mean_pi_frame_ms": 3.65, "reverse_motor_s": 3.299, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 21799, "render_s": 1.278, "pi_s": 4.49, "other_s": 6.091, "wall_time_s": 11.858, "wall_per_sim_s": 0.314}
n2 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 38.21, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.476, 8.485, 8.702], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.779, "E": 7.357, "I": 0.789, "S": 29.725}, "collisions": [], "min_clearance_wall_mm": 50.26, "min_clearance_sign_mm": 122.33, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 21178.9, "distance_backward_mm": 317.0, "mean_speed_mm_s": 562.6, "frames_processed": 1245, "mean_pi_frame_ms": 3.47, "reverse_motor_s": 3.182, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 22331, "render_s": 1.288, "pi_s": 4.315, "other_s": 6.106, "wall_time_s": 11.709, "wall_per_sim_s": 0.306}
n3 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 36.456, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.322, 8.251, 8.432], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.779, "E": 6.33, "I": 0.783, "S": 29.003}, "collisions": [], "min_clearance_wall_mm": 42.85, "min_clearance_sign_mm": 101.83, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 20667.3, "distance_backward_mm": 319.6, "mean_speed_mm_s": 575.7, "frames_processed": 1182, "mean_pi_frame_ms": 3.66, "reverse_motor_s": 2.939, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 21044, "render_s": 1.2, "pi_s": 4.32, "other_s": 5.997, "wall_time_s": 11.518, "wall_per_sim_s": 0.316}
n4 s20 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 34.806, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.317, 8.254, 8.374], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.779, "E": 4.709, "I": 0.798, "S": 28.959}, "collisions": [], "min_clearance_wall_mm": 71.78, "min_clearance_sign_mm": 94.49, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 20576.3, "distance_backward_mm": 194.8, "mean_speed_mm_s": 596.8, "frames_processed": 1128, "mean_pi_frame_ms": 3.83, "reverse_motor_s": 1.379, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 20106, "render_s": 1.173, "pi_s": 4.318, "other_s": 6.097, "wall_time_s": 11.587, "wall_per_sim_s": 0.333}

## Costo por carrera

COSTO n1 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1229, "media_ms": 10.561319436126862, "p50_ms": 7.059376800002326, "p95_ms": 25.556588399999388, "max_ms": 258.20675859999955, "pct_mas_25ms": 5.451586655817739, "dp_ms": [1950, 1950, 235, 128, 119, 109, 91, 88, 73, 70, 83, 68, 84, 77, 82]}
COSTO n2 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1245, "media_ms": 10.01395755662653, "p50_ms": 6.7147948000013, "p95_ms": 25.119477599999662, "max_ms": 257.6031795999997, "pct_mas_25ms": 5.0602409638554215, "dp_ms": [1950, 1950, 234, 126, 118, 109, 88, 76, 64, 87, 71, 85, 77, 83]}
COSTO n3 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1182, "media_ms": 10.596916202707266, "p50_ms": 7.031564400003986, "p95_ms": 25.17938079999968, "max_ms": 251.8226812, "pct_mas_25ms": 5.245346869712352, "dp_ms": [1950, 1950, 229, 125, 108, 105, 107, 91, 88, 75, 66, 83, 71, 84, 85, 81]}
COSTO n4 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1128, "media_ms": 11.080317572340405, "p50_ms": 7.1992564000019845, "p95_ms": 27.394099600000384, "max_ms": 266.3238276000001, "pct_mas_25ms": 6.914893617021277, "dp_ms": [1950, 1950, 241, 125, 107, 100, 105, 83, 79, 69, 84, 88, 86, 81, 83]}

## Tiempo de reloj total

12.1 s

## Validez

VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx 5.10, mem máx 3650 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.47-3.83.
