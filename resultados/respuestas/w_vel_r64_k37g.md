# Respuesta: w_vel_r64_k37g

_2026-11-24 06:46 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_vel_r64_k37g (ae186c4) como la pediste: 48 carreras (n1-2 x s1-24), banco nuevo de latencia, FF 2.8 / FD 3.8, -P 7, COST_FIRST 1.95, con tu TWIN_PI_EXTRA (LEAD_TC 7-10, FINAL_PWM 295, FINAL_LEAD_MM 1387, ROUTE_L23_K_MM_S_PER_PWM 3.7). Tu comando no traía TWIN_MOTOR_REAL / TWIN_VBAT / TWIN_CAMERA_FROM_CALIB; los puse igual que en w_vel_r54/r55 y w_frenos_r62 (1 / 11.5 / 1) para comparar contra ellas (si los querías distintos, dímelo y la repito). Corrió sola bajo twin.lock; no faltó ninguna dependencia del venv.

Sin problemas: corrió al primer intento. Si el config.py de la rama no traía `ROUTE_FINAL_TC`, el worktree lleva la línea local neutra `ROUTE_FINAL_TC = 11` como en tandas anteriores.

- resumen.py: `w_vel_r64_k37g: 21/48 ok  media 42.45  mediana 42.60  mejor 36.65  fallas: n1/s1 n1/s10 n1/s12 n1/s13 n1/s14 n1/s15 n1/s19 n1/s22 n1/s23 n1/s24 n1/s3 n1/s4 n1/s7 n2/s1 n2/s10 n2/s13 n2/s14 n2/s15 n2/s17 n2/s19 n2/s2 n2/s22 n2/s23 n2/s24 n2/s3 n2/s4 n2/s7`
- Reloj total: 60.7 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx / mem máx 4.52 / 4418 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.3-10.6 ms. Referencia: w_vel_r55_l1387_f295g hizo las mismas 48 carreras en 53.0 s y w_frenos_r62_g_cen_k37 en 71.4 s; esta tanda empezó sin otros bancos.
- Tipo de prueba: carrera completa desde el cajón (mismo tipo que w_vel_r54 / r55 y w_frenos_r36 / r62).
- w_vel_r55_l1387_f295g (árbol idéntico, mismo TWIN_PI_EXTRA salvo L23_K: allá 3.82 por default) en las mismas n1-2 x s1-24: 17/48 ok vs r64 21/48; mismo desenlace en 38/48; en las 14 que terminan en ambas -10.64 s en total; terminaban allá y ahora fallan: n1/s13, n1/s3, n2/s13; nuevas que terminan: n1/s11, n1/s2, n1/s5, n1/s8, n2/s11, n2/s5, n2/s9.
- w_vel_r54_l1387_f295g (árbol idéntico, L23_K 3.82; sólo sus carreras n1-2 x s1-24) en las mismas n1-2 x s1-24: 22/48 ok vs r64 21/48; mismo desenlace en 39/48; en las 17 que terminan en ambas -16.39 s en total; terminaban allá y ahora fallan: n1/s19, n1/s3, n2/s13, n2/s2, n2/s22; nuevas que terminan: n1/s2, n1/s20, n1/s5, n2/s5.
- w_frenos_r62_g_cen_k37 (árbol frenos 028bde6 sin la reserva blanda, mismo TWIN_PI_EXTRA con L23_K 3.7) en las mismas n1-2 x s1-24: 19/48 ok vs r64 21/48; mismo desenlace en 38/48; en las 15 que terminan en ambas -6.55 s en total; terminaban allá y ahora fallan: n1/s12, n1/s13, n1/s22, n2/s24; nuevas que terminan: n1/s16, n1/s17, n1/s2, n1/s8, n1/s9, n2/s5.
- Nota: las comparaciones cuentan STOP race_finished en ambas tandas; resumen.py puede contar ok con su propio criterio.
- Fallas: n1/s1 (4 giros), n1/s3 (4 giros), n1/s4 (4 giros), n1/s7 (2 giros), n1/s10 (2 giros), n1/s12 (4 giros), n1/s13 (0 giros), n1/s14 (4 giros), n1/s15 (4 giros), n1/s19 (2 giros), n1/s22 (2 giros), n1/s23 (1 giros), n1/s24 (8 giros), n2/s1 (5 giros), n2/s2 (12 giros), n2/s3 (1 giros), n2/s4 (12 giros), n2/s7 (2 giros), n2/s10 (1 giros), n2/s13 (0 giros), n2/s14 (7 giros), n2/s15 (4 giros), n2/s17 (1 giros), n2/s19 (1 giros), n2/s22 (2 giros), n2/s23 (1 giros), n2/s24 (3 giros).
- Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_vel_r64_k37g.md`.

Contexto: el árbol de ae186c4 es idéntico al de w_vel_r54/r55_l1387_f295g (99807a8 + parche, con la reserva blanda), y el TWIN_PI_EXTRA es el de esas salvo L23_K 3.7 (allá 3.82 por default), así que la comparación con r55_l1387 aísla L23_K 3.7 vs 3.82 en las mismas 48 semillas (r54 vs r55 da una idea del ruido del banco). w_frenos_r62_g_cen_k37 es el mismo K 3.7 sobre el árbol frenos (028bde6, sin la reserva blanda).
