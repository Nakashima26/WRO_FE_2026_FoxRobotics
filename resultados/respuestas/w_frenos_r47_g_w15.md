# Respuesta: w_frenos_r47_g_w15

_2026-11-24 05:38 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_frenos_r47_g_w15 (97ec55d) como la pediste: 48 carreras (n1-2 x s1-24), banco nuevo de latencia, FF 2.8 / FD 3.8, -P 7, COST_FIRST 1.95, con tu TWIN_PI_EXTRA (LEAD_TC 7-10, FINAL_PWM 295, FINAL_LEAD_MM 1387, FINAL_W 15), y con TWIN_MOTOR_REAL=1, VBAT 11.5 y cámara desde calibración como en w_frenos_r36_g_cen/_rep. Corrió sola bajo twin.lock; no faltó ninguna dependencia del venv.

Sin problemas: corrió al primer intento. El worktree conserva la línea local `ROUTE_FINAL_TC = 11` (neutra) que agregué para ftc10; sigue sin estar en la rama.

- resumen.py: `w_frenos_r47_g_w15: 19/48 ok  media 43.14  mediana 42.99  mejor 37.17  fallas: n1/s1 n1/s10 n1/s12 n1/s13 n1/s14 n1/s15 n1/s17 n1/s19 n1/s22 n1/s23 n1/s24 n1/s3 n1/s4 n1/s7 n1/s9 n2/s1 n2/s10 n2/s11 n2/s13 n2/s14 n2/s15 n2/s16 n2/s19 n2/s22 n2/s23 n2/s3 n2/s4 n2/s5 n2/s7`
- Reloj total: 61.5 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx / mem máx 5.68 / 4305 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.4-9.8 ms. Referencia: w_frenos_r36_g_cen hizo las mismas 48 carreras en 67.4 s y w_frenos_r47_g_ftc10 en 65.5 s.
- Tipo de prueba: carrera completa desde el cajón (mismo tipo que w_frenos_r36_g_cen / _rep, w_vel_r54 / r55).
- w_frenos_r36_g_cen_l1387_f295 (árbol idéntico a 9012909, mismo TWIN_PI_EXTRA salvo FINAL_W: allá 20 por default) en las mismas n1-2 x s1-24: 19/48 ok vs r47 19/48; mismo desenlace en 32/48; en las 11 que terminan en ambas -2.55 s en total; terminaban allá y ahora fallan: n1/s12, n1/s13, n1/s22, n1/s9, n2/s11, n2/s13, n2/s16, n2/s22; nuevas que terminan: n1/s16, n1/s2, n1/s5, n2/s12, n2/s17, n2/s2, n2/s20, n2/s24.
- w_frenos_r36_g_rep_l1387_f295 (repetición de cen, FINAL_W 20): sin salidas en /tmp/lat para comparar.
- w_frenos_r47_g_ftc10 (mismo árbol, FINAL_TC 10 y FINAL_W 20) en las mismas n1-2 x s1-24: 17/48 ok vs r47 19/48; mismo desenlace en 40/48; en las 14 que terminan en ambas -13.46 s en total; terminaban allá y ahora fallan: n1/s12, n2/s14, n2/s16; nuevas que terminan: n1/s11, n1/s5, n2/s12, n2/s17, n2/s24.
- Nota: las comparaciones cuentan STOP race_finished en ambas tandas; resumen.py puede contar ok con su propio criterio.
- Fallas: n1/s1 (2 giros), n1/s3 (4 giros), n1/s4 (4 giros), n1/s7 (3 giros), n1/s9 (1 giros), n1/s10 (1 giros), n1/s12 (4 giros), n1/s13 (0 giros), n1/s14 (2 giros), n1/s15 (3 giros), n1/s17 (1 giros), n1/s19 (2 giros), n1/s22 (8 giros), n1/s23 (1 giros), n1/s24 (3 giros), n2/s1 (4 giros), n2/s3 (3 giros), n2/s4 (12 giros), n2/s5 (1 giros), n2/s7 (4 giros), n2/s10 (3 giros), n2/s11 (4 giros), n2/s13 (0 giros), n2/s14 (3 giros), n2/s15 (3 giros), n2/s16 (3 giros), n2/s19 (2 giros), n2/s22 (4 giros), n2/s23 (1 giros).
- Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_frenos_r47_g_w15.md`.

Contexto: el árbol de 97ec55d es idéntico al de w_frenos_r36_g_cen (9012909) y al de ftc10 (b31cdb7), y el TWIN_PI_EXTRA es el de cen salvo FINAL_W 15 (allá 20 por default), así que la comparación con cen aísla FINAL_W 15 vs 20 en las mismas 48 semillas (cen vs rep da una idea del ruido del banco).
