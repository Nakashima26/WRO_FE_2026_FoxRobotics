# Respuesta: w_frenos_r36_g_rep_l1387_f295

_2026-11-24 04:31 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_frenos_r36_g_rep_l1387_f295 (fe333cc) tal como la mandaste: 288 carreras (ruido n3-n8 x semillas 1-48), TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, TWIN_CAMERA_FROM_CALIB=1, TWIN_COST_FIRST=1.95, FF 2.8 / FD 3.8, -P 7, TWIN_PI_EXTRA con ROUTE_FINAL_PWM 295 + ROUTE_FINAL_LEAD_MM 1387 (resto igual: latencia 0.125, clear-curve TC 7-10, clear 90 mm, no FF, lead reduction 0.02). Corrió sola bajo twin.lock y no faltó ninguna dependencia del venv.

- resumen.py: `w_frenos_r36_g_rep_l1387_f295: 140/288 ok  media 43.61  mediana 43.58  mejor 35.99` (las 288 carreras tienen STOP: 142 race_finished y 146 collision; resumen.py cuenta 140 ok).
- Choques (146): unos 96 contra señales (rojas y verdes, en muchas posiciones distintas), 44 contra pared y 6 contra la isla. La mayoría ocurre pronto: 134 de las 146 no pasan de 4 giros.
- Reloj total: 312.8 s, que equivale a 104.3 s por cada 96 carreras. La doy por **válida**: 0 procesos ajenos al inicio, carga máx 7.76, sin OOM/Killed. media_ms (COSTO) promedio es 15.8. mean_pi_frame_ms va de 3.17 a 16.3; el máximo es más alto que en tandas anteriores (unos 9 ms), pero el promedio de COSTO sigue en el rango normal.
- Tipo de prueba: carrera completa desde el cajón, en el banco nuevo con latencia y el env "real" (motor real, VBAT 11.5, cámara de calibración). En la caja no tengo otra tanda con este mismo set de 288 carreras (n3-8 x s1-48) para comparar. Las couple_* / w_vel_r44 usan otras semillas y otro número de carreras, así que no las comparo.

Resultados completos (STOP y COSTO por carrera): `grok/resultados:resultados/w_frenos_r36_g_rep_l1387_f295.md`.

**Aviso:** las tandas w_frenos_r28_{base,c91,f275,f280,t7c90,t8c90}_scr de esta madrugada (~01:55 PST) **no se completaron**: el disco de la caja se llenó (git "Out of diskspace"). base_scr salió 0/12 en 0.7 s, que es un resultado basura, y las demás no llegaron a correr. Por eso no hay resultados de ellas en grok/resultados. Ya liberé espacio (ahora 57 GB libres). Si las sigues necesitando, vuelve a mandarlas.

Siguiente: para saber si el freno final (PWM 295 / lead 1387) ayuda, haría falta una tanda base con las mismas 288 carreras y el mismo env, pero sin ROUTE_FINAL_PWM ni ROUTE_FINAL_LEAD_MM. Las fallas son tempranas (giros 0-4) y casi todas contra señales, así que es poco probable que las cause un parámetro del tramo final. Conviene comparar por semilla contra esa base.
