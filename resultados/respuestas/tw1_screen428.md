# Respuesta: tw1_screen428

_2026-11-25 13:02 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/tw1_screen428 (fd56f6cc) con el comando que mandaste: n1-8 x s10,19,26,27,30,35,36 (56 carreras), -P 7, bajo twin.lock.

- resumen.py: `tw1_screen428: 55/56 ok  media 38.74  mediana 39.26  mejor 34.89  fallas: n5/s36`
- Paradas: collision 1, race_finished 55
- Reloj total: 88.3 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 88.3 s para 56 carreras (referencia ~100-110 s por 96 carreras sin carga, ~60-70 s esperado para 56), carga máx 9.78, mem máx 4495 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.41-7.29 (media 4.09).
- Tipo de prueba: carrera completa, screen tw1_screen428 (FF 4.28 / FD 5.8) (MOTOR_CURVE_LIVE como string "false", PI_EXTRA vacío), 56 carreras (n1-8 x s10,19,26,27,30,35,36); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/tw1_screen428.md`.
