# Respuesta: blq_c120

_2026-11-25 23:49 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/blq_c120 (fa89e555) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `blq_c120: 372/384 ok  media 37.93  mediana 37.60  mejor 32.90  fallas: n1/s39 n1/s41 n1/s42 n3/s27 n3/s39 n3/s44 n5/s16 n5/s21 n5/s36 n6/s30 n6/s8 n7/s30`
- Paradas: collision 5, race_finished 377, stuck 2
- Reloj total: 604.3 s. VÁLIDA: corrió bajo twin.lock (espera de lock 1646 s, no incluida en el reloj; 0 procesos de banco ajenos al inicio), 604.3 s para 384 carreras (referencia: tandas de 384 recientes 487-555 s, p. ej. vis2_base 551.6 s; umbral de validez 620 s), carga máx 8.00, mem máx 4527 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 4.00-7.84 (media 4.77).
- Tipo de prueba: carrera completa, blq_c120 (main 52f42d03 + FOX_LAYOUT en el twin sin usar; FW MOTOR_CURVE_LIVE=false; PI: ROUTE_MOTOR_CURVE_LIVE=false, ROUTE_CORNER_EXIT_CLEAR_MM=120 en vez de 70), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

- Comparación (mismo tipo: carrera completa FF 4.28/FD 5.8, mismas semillas n1-8 x s1-48; base directa blq_base (misma rama base, CORNER_EXIT 70) si ya está publicada; referencia vis2_base (mismo env, commit a966eec8, no idéntico a main); regr_main = main 52f42d03 con CORNER_EXIT 70, si ya estaba publicada):
  - vs vis2_base: 378/384 ok, media 37.21, mediana 36.73, mejor 32.63 -> blq_c120 372/384 ok, media 37.93 (+0.72), mediana 37.60 (+0.87), mejor 32.90; fallas 12 vs 6; fallas en ambas: n3/s27 n5/s36

Qué mandar después: si regr_main (main 52f42d03, CORNER_EXIT 70, mismo comando y semillas) no está publicada todavía, esa es la base directa para medir el efecto de 120; si 120 mejora fallas sin costar mucho tiempo, barrer 90/150 con las mismas semillas; si empeora, revisar las semillas que fallan nuevas (n/s listadas en resumen.py) con traza.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/blq_c120.md`.
