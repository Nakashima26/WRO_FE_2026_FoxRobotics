# Respuesta: w_frenos_r47_g_ftc10

_2026-11-24 05:36 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_frenos_r47_g_ftc10 (b31cdb7) como la pediste: 48 carreras (n1-2 x s1-24), banco nuevo de latencia, FF 2.8 / FD 3.8, -P 7, COST_FIRST 1.95, con tu TWIN_PI_EXTRA (LEAD_TC 7-10, FINAL_PWM 295, FINAL_LEAD_MM 1387, FINAL_TC 10), y con TWIN_MOTOR_REAL=1, VBAT 11.5 y cámara desde calibración como en w_frenos_r36_g_cen/_rep. Corrió sola bajo twin.lock; no faltó ninguna dependencia del venv.

**Problema que encontré:** el primer intento dio 0/48 en 1.4 s sin correr nada: costo.py corta con `TWIN_PI_EXTRA: claves sin default en config.py (serían no-op): ['ROUTE_FINAL_TC']`. route_mode.py lo lee con getattr(C, "ROUTE_FINAL_TC", 11), pero config.py no lo declara. Para correrla agregué solo en mi worktree, debajo de ROUTE_FINAL_PWM, `ROUTE_FINAL_TC = 11` (igual al default, neutro si no se pasa), y TWIN_PI_EXTRA lo puso en 10. Conviene que agregues esa línea en la rama para no depender de mi parche local.

- resumen.py: `w_frenos_r47_g_ftc10: 16/48 ok  media 43.08  mediana 42.93  mejor 37.97  fallas: n1/s1 n1/s10 n1/s11 n1/s13 n1/s14 n1/s15 n1/s17 n1/s19 n1/s22 n1/s23 n1/s24 n1/s3 n1/s4 n1/s5 n1/s7 n1/s9 n2/s1 n2/s10 n2/s11 n2/s12 n2/s13 n2/s15 n2/s17 n2/s19 n2/s21 n2/s22 n2/s23 n2/s24 n2/s3 n2/s4 n2/s5 n2/s7`
- Reloj total: 65.5 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx / mem máx 5.72 / 14536 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.6-12.7 ms. Referencia: w_frenos_r36_g_cen hizo las mismas 48 carreras en 67.4 s y r55 en 53.0 s.
- Tipo de prueba: carrera completa desde el cajón (mismo tipo que w_frenos_r36_g_cen / _rep, w_vel_r54 / r55).
- w_frenos_r36_g_cen_l1387_f295 (árbol idéntico a 9012909, mismo TWIN_PI_EXTRA salvo FINAL_TC: allá 11 por default) en las mismas n1-2 x s1-24: 19/48 ok vs r47 17/48; mismo desenlace en 36/48; en las 12 que terminan en ambas +9.13 s en total; terminaban allá y ahora fallan: n1/s11, n1/s13, n1/s22, n1/s9, n2/s11, n2/s13, n2/s22; nuevas que terminan: n1/s16, n1/s2, n2/s14, n2/s2, n2/s20.
- w_frenos_r36_g_rep_l1387_f295 (repetición de cen, FINAL_TC 11): sin salidas en /tmp/lat para comparar.
- w_vel_r55_l1387_f295g (FINAL_TC 11, código con 99807a8 apagado) en las mismas n1-2 x s1-24: 17/48 ok vs r47 17/48; mismo desenlace en 36/48; en las 11 que terminan en ambas +10.03 s en total; terminaban allá y ahora fallan: n1/s13, n1/s17, n1/s3, n1/s9, n2/s12, n2/s13; nuevas que terminan: n1/s12, n1/s2, n1/s8, n2/s14, n2/s2, n2/s9.
- Nota: las comparaciones cuentan STOP race_finished en ambas tandas; resumen.py puede contar ok con su propio criterio.
- Fallas: n1/s1 (2 giros), n1/s3 (2 giros), n1/s4 (4 giros), n1/s5 (2 giros), n1/s7 (3 giros), n1/s9 (8 giros), n1/s10 (1 giros), n1/s11 (4 giros), n1/s13 (1 giros), n1/s14 (4 giros), n1/s15 (4 giros), n1/s17 (1 giros), n1/s19 (2 giros), n1/s22 (3 giros), n1/s23 (1 giros), n1/s24 (8 giros), n2/s1 (4 giros), n2/s3 (4 giros), n2/s4 (12 giros), n2/s5 (1 giros), n2/s7 (5 giros), n2/s10 (1 giros), n2/s11 (4 giros), n2/s12 (4 giros), n2/s13 (0 giros), n2/s15 (3 giros), n2/s17 (1 giros), n2/s19 (1 giros), n2/s22 (3 giros), n2/s23 (1 giros), n2/s24 (3 giros).
- Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_frenos_r47_g_ftc10.md`.

Contexto: el árbol de b31cdb7 es idéntico al de w_frenos_r36_g_cen (9012909) y el TWIN_PI_EXTRA es el mismo salvo FINAL_TC, así que la comparación con cen aísla FINAL_TC 10 vs 11 en las mismas 48 semillas (con el ruido del banco de por medio; cen vs rep da una idea de ese ruido).
