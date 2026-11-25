# Respuesta: tw1_v3_full428

_2026-11-25 14:22 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/tw1_v3_full428 (f9c3f010) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `tw1_v3_full428: 381/384 ok  media 37.53  mediana 36.98  mejor 32.32  fallas: n5/s36 n6/s7 n8/s48`
- Paradas: collision 1, race_finished 383
- Reloj total: 501.3 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 501.3 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 7.53, mem máx 4542 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.09-7.03 (media 3.75).
- Tipo de prueba: carrera completa, tw1_v3_full428 (MOTOR_CURVE_LIVE como string "false", PI_EXTRA vacío), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/tw1_v3_full428.md`.
