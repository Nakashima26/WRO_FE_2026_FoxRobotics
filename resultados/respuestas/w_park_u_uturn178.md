# Respuesta: w_park_u_uturn178

_2026-11-24 09:42 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_park_u_uturn178 (8f026ef) tal como la pediste: 10 carreras (n1-2 x s16 s20 s21 s22 s32), PARK_UTURN_PWM=178 sobre GEOM_EXT + GEOM_ROJO, ROUTE_PARK_GEOM_EXT=1, cámara desde calibración, COST_FIRST 1.95, -P 7. Corrió bajo twin.lock. No faltó ninguna dependencia del venv.

- resumen.py: `w_park_u_uturn178: 8/10 ok  media 42.04  mediana 41.82  mejor 37.55  fallas: n1/s32 n2/s16`
- Reloj total: 31.5 s. VÁLIDA con una salvedad. Corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx / mem máx 2.76 / 12605 MB, OOM/Killed/MemoryError: 0. mean_pi_frame_ms 3.59-7.04: 9 de 10 entre 3.59 y 5.46 (s21 sale alto en todas las tandas). La excepción es n2/s16 con 7.04 (COSTO media 20.2 ms, p50 14.6 ms, frente a ~11 / ~8 en el resto). Esa carrera choca con una señal a t=16.35 s, así que la tomo como dudosa por latencia. Repetición de control, sola y bajo el mismo lock (tag w_park_u_uturn178_chk, no cuenta en el resumen): n2/s16 ya no toca la señal, pero choca con el cajón magenta a t=42.41 s con 12 giros, mean_pi_frame_ms 3.55. Falla igual, pero en el estacionamiento. Reloj 31.5 s para 10 carreras.
- Tipo de prueba: carrera completa con estacionamiento geométrico y U. Es el mismo tipo que w_park_u_base / w_park_u_scan245 / w_park_u_uturn185 (s16/s20/s32) y que w_park_u_uturn185_cen (s1-24; comunes s16/s20/s21/s22).

Fallas (última línea STOP):
  - n1/s32: choque con cajón magenta a t=43.63 s (12 giros), STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 42.895, "turns_completed": 12, 
  - n2/s16: choque con señal Red E/T1 a t=17.06 s (4 giros), STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 16.35, "turns_completed": 4, "t

Comparación por semilla ("termina" = STOP race_finished; tiempos en s):
  - w_park_u_base: allá 5/6 terminan, aquí 4/6; en las 3 que terminan en ambas +4.17 s en total (n1/s16 -0.97, n2/s20 +3.89, n2/s32 +1.26). Aquí falla y allá terminaba: n1/s32 n2/s16. Aquí termina y allá fallaba: n1/s20.
  - w_park_u_scan245: allá 6/6 terminan, aquí 4/6; en las 4 que terminan en ambas +5.00 s en total (n1/s16 -0.40, n1/s20 +0.25, n2/s20 +4.17, n2/s32 +0.98). Aquí falla y allá terminaba: n1/s32 n2/s16.
  - w_park_u_uturn185: allá 5/6 terminan, aquí 4/6; en las 3 que terminan en ambas +0.99 s en total (n1/s16 +0.41, n1/s20 -0.07, n2/s20 +0.65). Aquí falla y allá terminaba: n1/s32 n2/s16. Aquí termina y allá fallaba: n2/s32.
  - w_park_u_uturn185_cen: allá 7/8 terminan, aquí 7/8; en las 6 que terminan en ambas +4.77 s en total (n1/s16 -0.96, n1/s20 -0.19, n1/s21 -0.13, n1/s22 +1.03, n2/s20 +4.73, n2/s21 +0.30). Aquí falla y allá terminaba: n2/s16. Aquí termina y allá fallaba: n2/s22.

| carrera | w_park_u_base | w_park_u_scan245 | w_park_u_uturn185 | w_park_u_uturn185_cen | w_park_u_uturn178 |
|---|---|---|---|---|---|
| n1/s16 | 42.986 | 42.411 | 41.602 | 42.973 | 42.013 |
| n1/s20 | choque 33.162 (cajón magenta) | 37.306 | 37.623 | 37.743 | 37.552 |
| n1/s21 | — | — | — | 40.125 | 39.992 |
| n1/s22 | — | — | — | 44.513 | 45.542 |
| n1/s32 | 44.235 | 44.785 | 43.442 | — | choque 42.895 (cajón magenta) |
| n2/s16 | 43.224 | 45.174 | 43.008 | 43.141 | choque 16.350 (señal Red E/T1) |
| n2/s20 | 37.746 | 37.468 | 40.983 | 36.907 | 41.636 |
| n2/s21 | — | — | — | 38.890 | 39.186 |
| n2/s22 | — | — | — | choque 43.703 (cajón magenta) | 45.106 |
| n2/s32 | 44.005 | 44.279 | choque 42.451 (cajón magenta) | — | 45.261 |

Lectura: con la U a 178 terminan 8/10. Lo bueno: s21 (n1 y n2) termina dentro del cajón, cuando con 185_cen las dos quedaban fuera, y n2/s22 ya no choca con el cajón. Lo malo: n1/s32 choca con el cajón magenta a t≈43.6 s (en base, scan245 y uturn185 terminaba), y n2/s16 falla. En la tanda chocó con una señal en la vuelta 1, con latencia inflada. En la repetición sola terminó los 12 giros y chocó con el cajón magenta. En tiempos, casi todas quedan a décimas de las otras tandas. La excepción es n2/s20, que pierde ~4 s frente a base, scan245 y 185_cen (igual que pasaba con uturn185, 40.98). Con 10 carreras, 178 no sale mejor que scan245 (6/6) y cambia qué semillas fallan en el cajón (s32/s16 en vez de s21/s22). Parece que el PWM de la U mueve la posición final de forma sensible y que ningún valor arregla todas.

Siguiente: corre 178 en n1-2 x s1-24 (más s32) para compararlo de frente con w_park_u_uturn185_cen y una base a 170 con ese mismo conjunto. Otra opción es juntar PARK_SCAN_U_PWM=245 con UTURN 178-185, porque scan245 fue la única 6/6. Mira también la geometría final del cajón magenta en n1/s32 y en n2/s16 (la repetición de control; las dos con 12 giros).

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_park_u_uturn178.md`.
