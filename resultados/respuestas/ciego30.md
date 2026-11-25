# Respuesta: ciego30

_2026-11-25 12:19 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/ciego30 (edc614a2) con el comando que mandaste: n1-6 x s1-48 (288 carreras), -P 7, bajo twin.lock (esperó su turno en la cola, no corrió en paralelo). No faltó ninguna dependencia del venv.

- resumen.py: `ciego30: 281/288 ok  media 36.33  mediana 35.95  mejor 32.24  fallas: n2/s16 n3/s19 n3/s26 n3/s27 n4/s26 n5/s35 n5/s36`
- Paradas: 282 race_finished, 6 collision, 0 sin línea STOP (ERROR).
- Reloj total: 356.6 s. VÁLIDA: bajo twin.lock, procesos de banco ajenos al inicio: 0, 356.6 s para 288 carreras (~119 s por 96), carga máx / mem máx (MB): 8.35 / 4571, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.05-6.96 (media 3.65).
- Tipo de prueba: carrera completa, banco nuevo con latencia con PARK_PL_CIEGO_MM=30; compárala solo con tandas del mismo tipo (288 carreras n1-6 x s1-48, mismo banco).

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/ciego30.md`.
