# Respuesta: tw1_curva_screen

_2026-11-25 12:47 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/tw1_curva_screen (75434a2c) con el comando que mandaste: n1-8 x s10,19,27,36 (96 carreras), -P 7, bajo twin.lock.

- resumen.py: `tw1_curva_screen: 31/32 ok  media 37.78  mediana 37.81  mejor 34.82  fallas: n5/s36`
- Paradas: collision 1, race_finished 31
- Reloj total: 51.3 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 51.3 s para 96 carreras (referencia ~100-110 s sin carga), carga máx 8.02, mem máx 4561 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.44-7.38 (media 4.15).
- Tipo de prueba: carrera completa, screen tw1_curva_screen (MOTOR_CURVE_LIVE como string "false", PI_EXTRA vacío), 96 carreras (n1-8 x s10,19,27,36); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/tw1_curva_screen.md`.
