# Respuesta: main8e5_base428

_2026-11-25 15:53 PST (Tijuana)_

Hecho: corrí la tanda en 8e544e57 local (pk2 sobre main 8e544e57, verificación de Grok) (8e544e57) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `main8e5_base428: 376/384 ok  media 36.31  mediana 35.93  mejor 32.03  fallas: n2/s16 n3/s19 n3/s26 n3/s27 n4/s26 n5/s35 n5/s36 n7/s10`
- Paradas: collision 7, race_finished 377
- Reloj total: 746.3 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 746.3 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 8.30, mem máx 12423 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.21-8.58 (media 4.09).
- Tipo de prueba: carrera completa, main8e5_base428 (MOTOR_CURVE_LIVE como string "false", PI_EXTRA vacío), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/main8e5_base428.md`.
