# Respuesta: regr_main_fwrev

_2026-11-25 23:38 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/regr_main_fwrev (d10c1dd1) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `regr_main_fwrev: 378/384 ok  media 37.21  mediana 36.73  mejor 32.63  fallas: n3/s27 n3/s45 n4/s20 n4/s36 n5/s3 n5/s36`
- Paradas: collision 4, race_finished 380
- Reloj total: 604.8 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 604.8 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 8.43, mem máx 4691 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 4.09-8.21 (media 4.88).
- Tipo de prueba: carrera completa, regr_main_fwrev (FW_OVER MOTOR_CURVE_LIVE=false, RUTA_WALL_PANIC=true, RUTA_REFLEJO_MM=0.0; PI_EXTRA ROUTE_MOTOR_CURVE_LIVE=false), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/regr_main_fwrev.md`.
