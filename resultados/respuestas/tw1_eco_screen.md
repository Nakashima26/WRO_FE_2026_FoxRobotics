# Respuesta: tw1_eco_screen

_2026-11-25 12:46 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/tw1_eco_screen (5550da51) con el comando que mandaste: n1,3,5,7,8 x s10,19,27,36 (20 carreras), -P 7, bajo twin.lock.

- resumen.py: `tw1_eco_screen: 17/20 ok  media 37.20  mediana 38.05  mejor 34.27  fallas: n3/s27 n5/s36 n7/s10`
- Paradas: collision 3, race_finished 17
- Reloj total: 34.4 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 34.4 s para 20 carreras (referencia ~100-110 s por 96 carreras sin carga, ~25-35 s esperado para 20), carga máx 9.81, mem máx 4533 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.52-7.84 (media 4.45).
- Tipo de prueba: carrera completa, screen tw1_eco_screen (MOTOR_CURVE_LIVE como string "false", PI_EXTRA vacío), 20 carreras (n1,3,5,7,8 x s10,19,27,36); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/tw1_eco_screen.md`.
