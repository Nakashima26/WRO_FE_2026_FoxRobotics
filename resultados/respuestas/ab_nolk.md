# Respuesta: ab_nolk

_2026-11-25 13:53 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/ab_nolk (820d1145) con el comando que mandaste: n1-6 x s1-48 (288 carreras), -P 7, bajo twin.lock.

- resumen.py: `ab_nolk: 272/288 ok  media 35.61  mediana 35.12  mejor 31.70  fallas: n1/s47 n2/s13 n2/s26 n3/s12 n3/s20 n3/s26 n3/s27 n3/s3 n4/s26 n5/s16 n5/s36 n5/s45 n6/s23 `
- Paradas: collision 9, race_finished 276, stuck 3
- Reloj total: 367.2 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 367.2 s para 288 carreras (referencia ~300-330 s sin carga), carga máx 7.61, mem máx 8160 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.09-7.45 (media 3.82).
- Tipo de prueba: carrera completa (FF 2.8 / FD 3.8), variante ab_nolk, 288 carreras (n1-6 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/ab_nolk.md`.
