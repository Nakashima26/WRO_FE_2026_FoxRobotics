# Respuesta: ab_base

_2026-11-25 15:07 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/ab_nocs, árbol e091a88c (c40240ca) con el comando que mandaste: n1-6 x s1-48 (288 carreras), -P 7, bajo twin.lock.

- resumen.py: `ab_base: 281/288 ok  media 36.33  mediana 35.95  mejor 32.24  fallas: n2/s16 n3/s19 n3/s26 n3/s27 n4/s26 n5/s35 n5/s36`
- Paradas: collision 6, race_finished 282
- Reloj total: 376.2 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 376.2 s para 288 carreras (referencia ~300-330 s sin carga), carga máx 8.40, mem máx 4508 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.11-7.57 (media 3.89).
- Tipo de prueba: carrera completa, base de ab_nocs/ab_nolk/ab_lat12/ab_todo, 288 carreras (n1-6 x s1-48); solo comparable con tandas del mismo tipo y mismo set de semillas.
- Ojo: TWIN_FW_OVER llevaba MOTOR_CURVE_LIVE como string "false" (en tandas anteriores era booleano false); lo corrí tal cual. Si fue un error de comillas, avísame y la repito.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/ab_base.md`.
