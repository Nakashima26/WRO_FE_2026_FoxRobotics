# Respuesta: ab_nocs

_2026-11-25 13:47 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/ab_nocs (c40240ca) con el comando que mandaste: n1-6 x s1-48 (288 carreras), -P 7, bajo twin.lock.

- resumen.py: `ab_nocs: 281/288 ok  media 35.93  mediana 35.41  mejor 32.62  fallas: n2/s16 n2/s26 n3/s19 n3/s27 n5/s10 n5/s36 n6/s7`
- Paradas: collision 6, race_finished 282
- Reloj total: 369.5 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 369.5 s para 288 carreras (referencia ~300-330 s sin carga), carga máx 7.75, mem máx 10061 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.23-8.06 (media 3.81).
- Tipo de prueba: carrera completa, variante ab_nocs, 288 carreras (n1-6 x s1-48); solo comparable con tandas del mismo tipo y mismo set de semillas.
- Ojo: TWIN_FW_OVER llevaba MOTOR_CURVE_LIVE como string "false" (en tandas anteriores era booleano false); lo corrí tal cual. Si fue un error de comillas, avísame y la repito.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/ab_nocs.md`.
