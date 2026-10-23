# Prueba de instalación del gemelo digital: semilla 17

- Rama/commit: fox/perfil-isla @ 75fef49
- nproc: 8
- Python: Python 3.13.5
- Compilador C++: zig c++ (pip ziglang) vía FOX_CXX; el box no tiene g++/clang++
- Duración de la carrera (reloj): 41.3 s (rc=0)
- Comando: `drive.py 17 /tmp/fox_s17 ruta_perfil '{}' --solo-cajon` con TWIN_FW_PARAMS="serial2_us_per_char=21.7,serial_us_per_char=21.7" TWIN_PI_UART_US_PER_BYTE=21.7 OMP_NUM_THREADS=1 PYTHONPATH=. FOX_PARK_POS=random TWIN_NOISE_SEED=1

## Última línea de /tmp/fox_s17.out

```
STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 52.345, "turns_completed": 12, "tpr": 12, "lap_times_s": [27.857, 11.701, 11.727], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.52, "I": 2.296, "S": 48.81, "T": 1.084}, "collisions": [], "min_clearance_wall_mm": 71.67, "min_clearance_sign_mm": 63.09, "pass_side_violations": [], "signs_passed": 15, "distance_forward_mm": 19834.3, "distance_backward_mm": 0.0, "mean_speed_mm_s": 378.9, "frames_processed": 2108, "mean_pi_frame_ms": 9.1, "reverse_motor_s": 0.0, "sign_contacts": [], "signs_touched": 0, "ignored_contacts": [], "sim_loops": 3015, "render_s": 3.471, "pi_s": 19.192, "other_s": 18.428, "wall_time_s": 41.091, "wall_per_sim_s": 0.785}
```
