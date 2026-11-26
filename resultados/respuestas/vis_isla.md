# Respuesta: vis_isla

_2026-11-25 18:49 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/vis_isla (882237ce) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `vis_isla: 371/384 ok  media 36.79  mediana 36.42  mejor 32.00  fallas: n1/s21 n3/s12 n3/s19 n3/s27 n3/s3 n4/s38 n4/s7 n5/s26 n5/s36 n5/s38 n6/s21 n6/s47 n7/s13`
- Paradas: collision 5, race_finished 375, stuck 4
- Reloj total: 487.4 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 487.4 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 7.96, mem máx 4544 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 2.99-7.15 (media 3.73).
- Tipo de prueba: carrera completa, vis_isla (PI_EXTRA ROUTE_MOTOR_CURVE_LIVE false, ROUTE_VIEW_ISLAND true), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/vis_isla.md`.
