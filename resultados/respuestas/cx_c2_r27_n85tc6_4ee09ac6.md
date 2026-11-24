# Respuesta: cx_c2_r27_n85tc6_4ee09ac6

_2026-11-24 01:39 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/cx_c2_r27_n85tc6_4ee09ac6 (8d54b6d) tal como la pediste: 384 carreras (n1-8 x s1-48), -P 7, todas las variables de entorno del payload. Corrió sola bajo twin.lock y no faltó ninguna dependencia del venv.

- resumen.py: `cx_c2_r27_n85tc6_4ee09ac6: 373/384 ok  media 42.06  mediana 41.50  mejor 36.70  fallas: n1/s36 n2/s16 n2/s21 n2/s37 n2/s44 n3/s21 n3/s27 n5/s36 n7/s31 n7/s39 n8/s17`
- Reloj total: 524.4 s.
- Tipo de prueba: carrera completa desde el cajón (salida, 3 vueltas y estacionamiento) en el banco nuevo con latencia, con LENS_CALIB, COST_FIJO=1 y FW_OVER={}.
- Validez: la doy por válida, con una nota. Sin OOM/Killed, carga máx 8.74 y media_ms 8.74 ms. El reloj salió ~25-30 % por encima de lo que dan las tandas recientes de 48 carreras y la memoria llegó a 14.7/16 GB, pero no afectó el costo por cuadro.
- Referencia: couple_n85 tiene el mismo TWIN_PI_EXTRA y es del mismo tipo de prueba, pero su banco no usaba LENS_CALIB, COST_FIJO=1 ni FW_OVER={}, así que la configuración no es la misma y la comparación es solo orientativa. En las 48 carreras comunes (n1-2 x s1-24): esta tanda termina 47/48 y couple_n85 terminaba 19/48, contando por STOP race_finished. En las 18 carreras que terminan en ambas, la suma de tiempos cambia -26.91 s.

Fallas según resumen.py (11), con la última línea STOP recortada. Ojo: n2/s21, n2/s44, n3/s27 y n8/s17 terminan con STOP race_finished, pero resumen.py igual las cuenta como fallas, supongo que por su criterio de estacionamiento o validez. Revisa resumen.py si eso no es lo que esperas:
- n1 s36 STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 6.338, "turns_completed": 1, "tpr": 12, "lap_times_s": [], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.577, 
- n2 s16 STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 42.189, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.31, 9.974, 10.335], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_s
- n2 s21 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 39.755, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.349, 9.21, 9.186], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time
- n2 s37 STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 40.508, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.703, 8.852, 9.284], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_s
- n2 s44 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 40.011, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.352, 9.181, 9.371], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "tim
- n3 s21 STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 35.535, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.482, 9.158, 9.145], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_s
- n3 s27 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 44.895, "turns_completed": 12, "tpr": 12, "lap_times_s": [15.51, 10.811, 10.544], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "ti
- n5 s36 STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 6.518, "turns_completed": 1, "tpr": 12, "lap_times_s": [], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_state_s": {"?": 0.618, 
- n7 s31 STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 43.616, "turns_completed": 12, "tpr": 12, "lap_times_s": [14.372, 9.323, 9.113], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_s
- n7 s39 STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 30.471, "turns_completed": 12, "tpr": 12, "lap_times_s": [11.619, 8.704, 9.638], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "time_per_s
- n8 s17 STOP race_finished {"finished": true, "stop_reason": "race_finished", "total_time_s": 38.794, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.833, 9.405, 8.195], "mean_corner_time_s": 0.0, "mean_time_in_g_s": 0.0, "tim

Resultados completos (STOP y COSTO por carrera): `grok/resultados:resultados/cx_c2_r27_n85tc6_4ee09ac6.md`.

Siguiente: ya está corriendo la tanda cx_c2_r27_base_4ee09ac6, que llegó justo detrás y esperó el lock. Esa es la comparación limpia con la misma configuración de banco. Con 373/384 aquí, la diferencia frente a la base va a estar en pocas carreras: conviene revisar si las fallas de arriba se repiten en la base con el mismo n/s.
