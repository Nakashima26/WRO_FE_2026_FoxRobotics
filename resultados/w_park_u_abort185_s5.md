# w_park_u_abort185_s5

Tipo de prueba: carrera completa con estacionamiento geométrico y U (latencia/uno.sh), banco NUEVO con latencia; mismo tipo que w_park_u_abort185 / w_park_u_abort185_cen / w_park_u_abort185_s20 / w_park_u_uturn185(_cen) / w_park_u_base(_cen). TWIN_FW_OVER PARK_GEOM_EXT=1, PARK_GEOM_ROJO=1, PARK_UTURN_ABORT_PWM=185; TWIN_PI_EXTRA ROUTE_PARK_GEOM_EXT=1; TWIN_MOTOR_REAL=1, VBAT 11.5, cámara desde calibración + lens_calib.npz, COST_FIRST 1.95, FF 2.8 / FD 3.8, n1-4 x s5 (4 carreras), -P 7. Commit f7de10b (origin/grok/t/w_park_u_abort185_s5).

## Resumen (resumen.py)

w_park_u_abort185_s5: 4/4 ok  media 46.03  mediana 45.93  mejor 45.53  fallas:

## Carreras (última línea STOP)

n1 s5 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 45.532, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.153, 9.811, 9.641], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.79, "E": 8.497, "I": 0.813, "S": 35.879}, "collisions": [], "min_clearance_wall_mm": 46.68, "min_clearance_sign_mm": 68.23, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24537.0, "distance_backward_mm": 866.0, "mean_speed_mm_s": 557.9, "frames_processed": 1488, "mean_pi_frame_ms": 3.51, "reverse_motor_s": 5.459, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 27283, "render_s": 1.431, "pi_s": 5.226, "other_s": 6.221, "wall_time_s": 12.878, "wall_per_sim_s": 0.283}
n2 s5 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 45.899, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.635, 9.75, 9.679], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.79, "E": 8.495, "I": 0.802, "S": 36.262}, "collisions": [], "min_clearance_wall_mm": 52.11, "min_clearance_sign_mm": 54.83, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24309.7, "distance_backward_mm": 809.6, "mean_speed_mm_s": 547.3, "frames_processed": 1499, "mean_pi_frame_ms": 3.51, "reverse_motor_s": 5.312, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 27531, "render_s": 1.424, "pi_s": 5.258, "other_s": 6.222, "wall_time_s": 12.904, "wall_per_sim_s": 0.281}
n3 s5 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 45.959, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.282, 9.93, 9.671], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.749, "E": 8.677, "I": 0.845, "S": 36.135}, "collisions": [], "min_clearance_wall_mm": 39.82, "min_clearance_sign_mm": 60.16, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24868.4, "distance_backward_mm": 876.3, "mean_speed_mm_s": 560.2, "frames_processed": 1506, "mean_pi_frame_ms": 3.36, "reverse_motor_s": 5.67, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 27714, "render_s": 1.438, "pi_s": 5.054, "other_s": 6.232, "wall_time_s": 12.724, "wall_per_sim_s": 0.277}
n4 s5 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 46.716, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.505, 9.78, 9.669], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.79, "E": 9.332, "I": 0.809, "S": 36.239}, "collisions": [], "min_clearance_wall_mm": 34.23, "min_clearance_sign_mm": 70.07, "pass_side_violations": [], "signs_passed": 21, "distance_forward_mm": 24647.7, "distance_backward_mm": 909.9, "mean_speed_mm_s": 547.1, "frames_processed": 1531, "mean_pi_frame_ms": 3.41, "reverse_motor_s": 5.82, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 27976, "render_s": 1.471, "pi_s": 5.213, "other_s": 6.217, "wall_time_s": 12.901, "wall_per_sim_s": 0.276}

## Costo por carrera

COSTO n1 s5 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1488, "media_ms": 10.137995308736608, "p50_ms": 7.1642619999988, "p95_ms": 24.413765599999945, "max_ms": 260.27610119999974, "pct_mas_25ms": 4.368279569892473, "dp_ms": [1950, 1950, 238, 105, 111, 177, 132, 104, 95, 97, 98, 84, 69, 72, 92]}
COSTO n2 s5 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1499, "media_ms": 10.132031463375599, "p50_ms": 6.963577599999126, "p95_ms": 25.052770399999954, "max_ms": 272.9193347999998, "pct_mas_25ms": 5.003335557038025, "dp_ms": [1950, 1950, 250, 107, 113, 173, 99, 96, 95, 96, 82, 80, 80, 68, 67, 98]}
COSTO n3 s5 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1506, "media_ms": 9.685598166799396, "p50_ms": 7.047314399999038, "p95_ms": 22.9829488, "max_ms": 251.6890663999999, "pct_mas_25ms": 2.921646746347941, "dp_ms": [1950, 1950, 221, 105, 109, 163, 98, 106, 96, 100, 81, 89, 74, 73, 91]}
COSTO n4 s5 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1531, "media_ms": 9.822988952057573, "p50_ms": 7.252394799996863, "p95_ms": 23.68815680000029, "max_ms": 250.37846339999976, "pct_mas_25ms": 3.3964728935336383, "dp_ms": [1950, 1950, 227, 104, 111, 161, 105, 98, 98, 97, 83, 97, 73, 77, 88]}

## Tiempo de reloj total

13.2 s

## Validez

VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx 0.58, mem máx 3754 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.36-3.51.
