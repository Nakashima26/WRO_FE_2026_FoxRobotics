# Respuesta: w_park_leg_recover

_2026-11-23 23:11 PST (Tijuana)_

Hecho: tanda corrida en origin/grok/t/w_park_leg_recover (72846db), tal como la pediste (24 carreras: n1-n2 x s1 2 3 5 8 12 16 19 20 26 32 37, TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, ROUTE_PLAN_ASYNC, COST_FIRST 1.95, -P 7), sola bajo twin.lock. Válida: reloj 36.8 s, carga máx 5.94, sin OOM, mean_pi_frame_ms 3.58 (máx 4.55).

- resumen.py: `w_park_leg_recover: 19/24 ok  media 42.55  mediana 42.62  mejor 37.64  fallas: n1/s3 n1/s5 n2/s19 n2/s3 n2/s5`
- Resultado completo: grok/resultados:resultados/w_park_leg_recover.md

Lo encontrado: **regresión en el estacionamiento**. Las 5 fallas no chocan (race_finished, collisions [] e ignored_contacts []), pero terminan fuera del cajón: fuera_mm 521.6 (n1/s3), 684.7 (n1/s5), 543.1 (n2/s3), 1373.2 (n2/s5), 771.9 (n2/s19). Además terminan 7.6-8.7 s antes que en w_park_leg_abort_chueco (n1/s3 46.38 -> 37.92, n1/s5 46.61 -> 38.55, n2/s3 47.03 -> 37.12, n2/s5 46.38 -> 39.35, n2/s19 40.20 -> 31.98). Todo indica que la carrera se declara terminada sin completar la pierna de estacionamiento (¿la lógica de recover del parche corta o salta la maniobra?). En w_park_leg_abort_chueco esas mismas 5 carreras estacionaban bien (24/24 ok en este subconjunto, media 43.09).

En las 19 carreras que pasan en ambas tandas, la diferencia pareada es de +0.05 s, o sea que no hay mejora de tiempo.

Pregunta: ¿el parche de recover cambia la condición de fin/STOP del estacionamiento? Si quieres, mándame una tanda con trazas (o solo n1/s3 y n2/s5) para ver en qué fase se corta.
