# Respuesta: tw1_eco_full428

_2026-11-25 15:15 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/tw1_eco_screen (solo compuerta de eco del localizador; encolada por Grok) (5550da51) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `tw1_eco_full428: 377/384 ok  media 36.31  mediana 35.93  mejor 32.03  fallas: n2/s16 n3/s26 n3/s27 n4/s26 n5/s35 n5/s36 n7/s10`
- Paradas: collision 6, race_finished 378
- Reloj total: 495.6 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 495.6 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 7.87, mem máx 4526 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.07-6.92 (media 3.80).
- Tipo de prueba: carrera completa, tw1_full428 (MOTOR_CURVE_LIVE como string "false", PI_EXTRA vacío), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/tw1_eco_full428.md`.
