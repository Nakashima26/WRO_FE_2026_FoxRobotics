# Respuesta: blq_base

_2026-11-25 23:29 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/blq_base (30926eb3) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `blq_base: 378/384 ok  media 37.21  mediana 36.73  mejor 32.63  fallas: n3/s27 n3/s45 n4/s20 n4/s36 n5/s3 n5/s36`
- Paradas: collision 4, race_finished 380
- Reloj total: 603.2 s. INVÁLIDA por reloj (sin OOM, carga ok, sin procesos ajenos; el reloj quedó por encima del umbral de 560 s; mismo criterio que real2_cand 620.9 s): corrió bajo twin.lock (espera de lock 453 s, no incluida en el reloj; 0 procesos de banco ajenos al inicio), 603.2 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 8.03, mem máx 4618 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 4.00-7.76 (media 4.86) (vis2_base válida: 551.6 s, media 4.45). El box se reinició ~23:02 PST; las tandas posteriores (real2_cand 620.9 s, esta 603.2 s) corren más lentas.
- Tipo de prueba: carrera completa, blq_base (main 52f42d03 + twin FOX_LAYOUT sin activar, FW MOTOR_CURVE_LIVE=false, PI ROUTE_MOTOR_CURVE_LIVE=false), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas. Como FOX_LAYOUT no se pasó por env, debería reproducir main 52f42d03 (sirve de control de que el parche del twin no cambia nada por defecto).

- Comparación (mismo tipo: carrera completa FF 4.28/FD 5.8, mismas semillas n1-8 x s1-48; referencia vis2_base; vis2_base916/regr_main si ya estaban publicadas):
  - vs vis2_base: 378/384 ok, media 37.21, mediana 36.73, mejor 32.63 -> blq_base 378/384 ok, media 37.21 (+0.00), mediana 36.73 (+0.00), mejor 32.63; fallas 6 vs 6; fallas en ambas: n3/s27 n3/s45 n4/s20 n4/s36 n5/s3 n5/s36

Algo raro: el box se reinició a las ~23:02 PST y mató los procesos en cola (tuve que relanzar esta tanda); el venv se recreó sin scipy (no hizo falta, 0 errores). Los resultados por carrera salieron idénticos a vis2_base (Δ0 en todo), así que el parche FOX_LAYOUT sin activar no cambia nada y la invalidez por reloj no parece afectar los números (con ese Δ0 se pueden tomar como control). Sugiero revisar el umbral de reloj (~400-440 s ya no es realista en este banco: las últimas válidas rondan 550 s).

Qué mandar después: las variantes blq_* con FOX_LAYOUT=<acomodo real> (mismo comando y semillas) contra esta base. Si querías FOX_LAYOUT activo en esta tanda, avísame: este comando no lo pasaba.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/blq_base.md`.
