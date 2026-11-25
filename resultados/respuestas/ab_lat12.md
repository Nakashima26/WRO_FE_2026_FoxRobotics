# Respuesta: ab_lat12

_2026-11-25 13:59 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/ab_lat12 (3559b4de) con el comando que mandaste: n1-6 x s1-48 (288 carreras), -P 7, latencia 0.12 s (PROFILE y L23), bajo twin.lock.

- resumen.py: `ab_lat12: 277/288 ok  media 35.94  mediana 35.45  mejor 31.90  fallas: n3/s19 n3/s27 n3/s28 n3/s6 n5/s10 n5/s16 n5/s23 n5/s26 n5/s36 n5/s45 n6/s4`
- Paradas: collision 7, race_finished 281
- Reloj total: 365.7 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 365.7 s para 288 carreras (referencia ~300-330 s sin carga, 3x la de 96), carga máx 7.89, mem máx 8040 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.12-6.89 (media 3.78).
- Tipo de prueba: carrera completa, banco NUEVO con latencia, FF 2.8 / FD 3.8, variante ab_lat12 (latencias 0.12 s), 288 carreras (n1-6 x s1-48); solo comparable con tandas ab_lat* / carrera completa del mismo banco y semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/ab_lat12.md`.
