# Respuesta: w_frenos_r36_g_cen_l1387_f295

_2026-11-24 04:26 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_frenos_r36_g_cen_l1387_f295 (9012909) como la pediste: 48 carreras (n1-2 x s1-24) con el banco nuevo de latencia, TWIN_MOTOR_REAL=1, VBAT 11.5, cámara desde calibración, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7, con tu TWIN_PI_EXTRA (LEAD_TC 7-10, FINAL_PWM 295, FINAL_LEAD_MM 1387). Corrió sola bajo twin.lock y no faltó ninguna dependencia del venv.

- resumen.py: `w_frenos_r36_g_cen_l1387_f295: 19/48 ok  media 43.43  mediana 43.63  mejor 36.98  fallas: n1/s1 n1/s10 n1/s14 n1/s15 n1/s16 n1/s17 n1/s19 n1/s2 n1/s23 n1/s24 n1/s3 n1/s4 n1/s5 n1/s7 n2/s1 n2/s10 n2/s12 n2/s14 n2/s15 n2/s17 n2/s19 n2/s2 n2/s20 n2/s23 n2/s24 n2/s3 n2/s4 n2/s5 n2/s7`
- Reloj total: 67.4 s. **Válida**: carga máx/mem máx 4.97 / 14613 MB, sin OOM/Killed, mean_pi_frame_ms 3.4-12.4 ms.
- Tipo de prueba: carrera completa desde el cajón (mismo tipo que w_vel_r44_l125f285, la única tanda comparable en grok/resultados).
- Frente a w_vel_r44_l125f285 (mismo tipo, mismas 48 carreras): 21/48 → 19/48 ok. En las 16 carreras que terminan en ambas: +5.21 s en total. Terminaban en r44 y ahora fallan: n1/s16, n1/s5, n2/s19, n2/s20, n2/s5. Nuevas que terminan: n1/s22, n2/s22, n2/s9.
- Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_frenos_r36_g_cen_l1387_f295.md`.

Nota: 23 de las 29 fallas son choques tempranos (1-4 giros), parecido a r44, así que no dependen del final. Ojo con 3 choques con 12 giros, ya en el final/estacionamiento: n1/s16 (t=44.35 s), n2/s2 (t=44.27 s) y n2/s20 (t=30.80 s); n1/s16 y n2/s20 terminaban en r44, así que podrían venir de FINAL_PWM 295 / FINAL_LEAD_MM 1387. Las otras: n2/s1 (5 giros), n2/s12 (6), n2/s4 (7).

Siguiente: para aislar FINAL_PWM 295 / FINAL_LEAD_MM 1387, repetiría esta misma base con los valores del final de r44 (solo cambia eso) y n1-4 en s5 s16 s19 s20 s22.
