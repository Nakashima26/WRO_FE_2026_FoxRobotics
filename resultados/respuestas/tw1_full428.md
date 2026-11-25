# Respuesta: tw1_full428

_2026-11-25 13:29 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/tw1_full428 (f7197be6) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `tw1_full428: 380/384 ok  media 37.25  mediana 36.75  mejor 32.32  fallas: n1/s37 n3/s38 n5/s36 n6/s7`
- Paradas: collision 1, race_finished 383
- Reloj total: 519.9 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 519.9 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 8.02, mem máx 10878 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.13-7.44 (media 3.86).
- Tipo de prueba: carrera completa, tw1_full428 (MOTOR_CURVE_LIVE como string "false", PI_EXTRA vacío), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/tw1_full428.md`.
