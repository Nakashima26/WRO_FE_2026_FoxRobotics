# Respuesta: w_park_u_abort185

_2026-11-24 09:51 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_park_u_abort185 (246c7f0) tal como la pediste: 10 carreras (n1-2 x s16 s20 s21 s22 s32), PARK_UTURN_ABORT_PWM=185 (no PARK_UTURN_PWM) sobre GEOM_EXT + GEOM_ROJO, ROUTE_PARK_GEOM_EXT=1, cámara desde calibración, COST_FIRST 1.95, -P 7. Corrió bajo twin.lock. No faltó ninguna dependencia del venv. Además repetí sola la única falla (n2/s20) como control.

- resumen.py: `w_park_u_abort185: 9/10 ok  media 42.28  mediana 43.11  mejor 36.70  fallas: n2/s20`
- Reloj total: 25.0 s. VÁLIDA. Corrió sola bajo twin.lock + /tmp/fox-grok-bancos.lock (0 procesos de banco ajenos al inicio, carga inicial 0.21), carga máx / mem máx 1.88 / 6589 MB, OOM/Killed/MemoryError: 0. mean_pi_frame_ms 3.54-5.38 (los más altos son s21, que sale alto en todas las tandas; n2/s20 4.57). Reloj 25.0 s para 10 carreras, dentro de lo normal (~30 s). Repetición de control de la única falla, sola y bajo el mismo lock (tag w_park_u_abort185_chk, no cuenta en el resumen): n2/s20 termina en 37.657 s sin choques, mean_pi_frame_ms 3.78.
- Tipo de prueba: carrera completa con estacionamiento geométrico y U. Es el mismo tipo que w_park_u_base / w_park_u_scan245 / w_park_u_uturn185 (s16/s20/s32), w_park_u_uturn185_cen (s1-24; comunes s16/s20/s21/s22) y w_park_u_uturn178 (mismas 10).

Fallas (última línea STOP):
  - n2/s20: choque con cajón magenta a t=22.60 s (7 giros), STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 21.894, "turns_completed": 7, "

Comparación por semilla ("termina" = STOP race_finished; tiempos en s):
  - w_park_u_base: allá 5/6 terminan, aquí 5/6; en las 4 que terminan en ambas -1.26 s en total (n1/s16 -0.54, n1/s32 -0.27, n2/s16 -0.11, n2/s32 -0.34). Aquí falla y allá terminaba: n2/s20. Aquí termina y allá fallaba: n1/s20.
  - w_park_u_scan245: allá 6/6 terminan, aquí 5/6; en las 5 que terminan en ambas -4.06 s en total (n1/s16 +0.03, n1/s20 -0.60, n1/s32 -0.82, n2/s16 -2.06, n2/s32 -0.61). Aquí falla y allá terminaba: n2/s20.
  - w_park_u_uturn185: allá 5/6 terminan, aquí 5/6; en las 4 que terminan en ambas +0.55 s en total (n1/s16 +0.84, n1/s20 -0.92, n1/s32 +0.53, n2/s16 +0.10). Aquí falla y allá terminaba: n2/s20. Aquí termina y allá fallaba: n2/s32.
  - w_park_u_uturn185_cen: allá 7/8 terminan, aquí 7/8; en las 6 que terminan en ambas -1.18 s en total (n1/s16 -0.53, n1/s20 -1.04, n1/s21 +0.04, n1/s22 +0.61, n2/s16 -0.03, n2/s21 -0.23). Aquí falla y allá terminaba: n2/s20. Aquí termina y allá fallaba: n2/s22.
  - w_park_u_uturn178: allá 8/10 terminan, aquí 9/10; en las 7 que terminan en ambas -1.23 s en total (n1/s16 +0.43, n1/s20 -0.85, n1/s21 +0.17, n1/s22 -0.42, n2/s21 -0.52, n2/s22 +1.55, n2/s32 -1.59). Aquí falla y allá terminaba: n2/s20. Aquí termina y allá fallaba: n1/s32 n2/s16.

| carrera | w_park_u_base | w_park_u_scan245 | w_park_u_uturn185 | w_park_u_uturn185_cen | w_park_u_uturn178 | w_park_u_abort185 |
|---|---|---|---|---|---|---|
| n1/s16 | 42.986 | 42.411 | 41.602 | 42.973 | 42.013 | 42.441 |
| n1/s20 | choque 33.162 (cajón magenta) | 37.306 | 37.623 | 37.743 | 37.552 | 36.705 |
| n1/s21 | — | — | — | 40.125 | 39.992 | 40.166 |
| n1/s22 | — | — | — | 44.513 | 45.542 | 45.121 |
| n1/s32 | 44.235 | 44.785 | 43.442 | — | choque 42.895 (cajón magenta) | 43.968 |
| n2/s16 | 43.224 | 45.174 | 43.008 | 43.141 | choque 16.350 (señal Red E/T1) | 43.109 |
| n2/s20 | 37.746 | 37.468 | 40.983 | 36.907 | 41.636 | choque 21.894 (cajón magenta) |
| n2/s21 | — | — | — | 38.890 | 39.186 | 38.663 |
| n2/s22 | — | — | — | choque 43.703 (cajón magenta) | 45.106 | 46.657 |
| n2/s32 | 44.005 | 44.279 | choque 42.451 (cajón magenta) | — | 45.261 | 43.669 |

Lectura: con la U a 185 sólo en el aborto terminan 9/10, la mejor cuenta en este conjunto de 10 (uturn178 8/10). Las semillas que fallaban en el cajón en otras tandas terminan todas aquí: n1/s32 y n2/s16 (fallaban con 178), n2/s32 (con uturn185), n2/s22 (con 185_cen) y s21 n1/n2 (fuera del cajón con 185_cen). La única falla, n2/s20, no parece venir del cambio: choca con el cajón magenta a t≈22.6 s con 7 giros, a mitad de la carrera, antes de estacionar (el cambio sólo toca la U del estacionamiento). En la repetición sola terminó en 37.657 s sin choques. Con una sola repetición, apunta a ruido de corrida más que a un fallo reproducible. En tiempos queda a décimas de las otras tandas en las carreras comunes: −1.26 s en total frente a base, −1.18 frente a 185_cen, −1.23 frente a 178, +0.55 frente a uturn185 y −4.06 frente a scan245 (casi todo n2/s16, −2.06). Frente a scan245 (6/6) aquí van 5/6 en s16/s20/s32; la diferencia es justo n2/s20. Salvedad: el .out no registra si el aborto DIRECTO→U llegó a dispararse en cada carrera. No puedo separar qué parte de la mejora es del PWM de aborto y qué parte es variación entre corridas (base también tuvo un choque con el cajón magenta en n1/s20).

Preguntas: ¿hay alguna forma de registrar en la salida del gemelo cuándo se activa parkUturnFromAbort (p. ej. una línea en STOP o en el log del firmware SIL)? Con eso se sabría en qué semillas actúa el cambio.

Siguiente: corre abort185 en n1-2 x s1-24 (más s32) para compararlo de frente con w_park_u_uturn185_cen. Si se mantiene, prueba también ABORT_PWM 178 / 195 con el mismo conjunto, o júntalo con PARK_SCAN_U_PWM=245. Para n2/s20, si quieres descartar del todo el cambio, una tanda n1-4 x s20 bastaría.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_park_u_abort185.md`.
