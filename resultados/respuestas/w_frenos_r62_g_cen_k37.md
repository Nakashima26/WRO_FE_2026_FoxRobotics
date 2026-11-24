# Respuesta: w_frenos_r62_g_cen_k37

_2026-11-24 06:39 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_frenos_r62_g_cen_k37 (6cd1c5c) como la pediste: 48 carreras (n1-2 x s1-24), banco nuevo de latencia, FF 2.8 / FD 3.8, -P 7, COST_FIRST 1.95, con tu TWIN_PI_EXTRA (LEAD_TC 7-10, FINAL_PWM 295, FINAL_LEAD_MM 1387, ROUTE_L23_K_MM_S_PER_PWM 3.7), con TWIN_MOTOR_REAL=1 y VBAT 11.5. Tu comando no traía TWIN_CAMERA_FROM_CALIB=1; lo agregué igual que en w_frenos_r36_g_cen/_rep y r47 para comparar contra ellas (si lo querías apagado, dímelo y la repito). Corrió sola bajo twin.lock; no faltó ninguna dependencia del venv.

Sin problemas: corrió al primer intento. El worktree conserva la línea local `ROUTE_FINAL_TC = 11` (neutra) que agregué para ftc10; sigue sin estar en la rama.

- resumen.py: `w_frenos_r62_g_cen_k37: 18/48 ok  media 42.87  mediana 43.35  mejor 37.04  fallas: n1/s1 n1/s10 n1/s14 n1/s15 n1/s16 n1/s17 n1/s19 n1/s2 n1/s23 n1/s24 n1/s3 n1/s4 n1/s7 n1/s8 n1/s9 n2/s1 n2/s10 n2/s11 n2/s13 n2/s14 n2/s15 n2/s17 n2/s19 n2/s2 n2/s22 n2/s23 n2/s3 n2/s4 n2/s5 n2/s7`
- Reloj total: 71.4 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx / mem máx 6.07 / 14617 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.9-20.2 ms. Referencia: w_frenos_r36_g_cen hizo las mismas 48 carreras en 67.4 s y w_frenos_r47_g_w15 con el mismo árbol; esta tanda empezó sin otros bancos.
- Tipo de prueba: carrera completa desde el cajón (mismo tipo que w_frenos_r36_g_cen / _rep, w_vel_r54 / r55).
- w_frenos_r36_g_cen_l1387_f295 (árbol idéntico a 9012909, mismo TWIN_PI_EXTRA salvo L23_K: allá 3.82 por default) en las mismas n1-2 x s1-24: 19/48 ok vs r62 19/48; mismo desenlace en 40/48; en las 15 que terminan en ambas -3.25 s en total; terminaban allá y ahora fallan: n1/s8, n1/s9, n2/s13, n2/s22; nuevas que terminan: n1/s5, n2/s12, n2/s20, n2/s24.
- w_frenos_r36_g_rep_l1387_f295 (repetición de cen, L23_K 3.82): sin salidas en /tmp/lat para comparar.
- w_frenos_r47_g_w15 (mismo árbol, FINAL_W 15, L23_K 3.82) en las mismas n1-2 x s1-24: 19/48 ok vs r62 19/48; mismo desenlace en 38/48; en las 14 que terminan en ambas -12.05 s en total; terminaban allá y ahora fallan: n1/s16, n1/s2, n1/s8, n2/s17, n2/s2; nuevas que terminan: n1/s12, n1/s13, n1/s22, n2/s11, n2/s16.
- Nota: las comparaciones cuentan STOP race_finished en ambas tandas; resumen.py puede contar ok con su propio criterio.
- Fallas: n1/s1 (2 giros), n1/s2 (3 giros), n1/s3 (4 giros), n1/s4 (4 giros), n1/s7 (2 giros), n1/s8 (0 giros), n1/s9 (9 giros), n1/s10 (2 giros), n1/s14 (3 giros), n1/s15 (1 giros), n1/s16 (6 giros), n1/s17 (1 giros), n1/s19 (2 giros), n1/s23 (1 giros), n1/s24 (5 giros), n2/s1 (4 giros), n2/s2 (3 giros), n2/s3 (4 giros), n2/s4 (9 giros), n2/s5 (1 giros), n2/s7 (2 giros), n2/s10 (4 giros), n2/s13 (0 giros), n2/s14 (4 giros), n2/s15 (4 giros), n2/s17 (1 giros), n2/s19 (1 giros), n2/s22 (3 giros), n2/s23 (1 giros).
- Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_frenos_r62_g_cen_k37.md`.

Contexto: el árbol de 6cd1c5c es idéntico al de w_frenos_r36_g_cen (9012909) y al de r47_g_w15 (97ec55d), y el TWIN_PI_EXTRA es el de cen salvo L23_K 3.7 (allá 3.82 por default), así que la comparación con cen aísla L23_K 3.7 vs 3.82 en las mismas 48 semillas (cen vs rep da una idea del ruido del banco).
