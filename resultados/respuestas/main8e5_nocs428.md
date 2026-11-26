# Respuesta: main8e5_nocs428

_2026-11-25 16:12 PST (Tijuana)_

Hecho: corrí la tanda en 8e544e57 local (main 8e544e57 con PI_EXTRA ROUTE_L1_CORNER_SIGN null; verificación de Grok de ab_nocs) (8e544e57) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `main8e5_nocs428: 373/384 ok  media 35.93  mediana 35.37  mejor 32.62  fallas: n2/s16 n2/s26 n3/s19 n3/s27 n5/s10 n5/s36 n6/s7 n7/s27 n7/s3 n8/s17 n8/s7`
- Paradas: collision 8, race_finished 376
- Reloj total: 1105.7 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 1105.7 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 9.33, mem máx 13577 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.25-74.67 (media 5.11).
- Tipo de prueba: carrera completa, main8e5_nocs428 (MOTOR_CURVE_LIVE como string "false", PI_EXTRA vacío), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/main8e5_nocs428.md`.
