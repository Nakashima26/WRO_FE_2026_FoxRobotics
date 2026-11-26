# Respuesta: noasig_on

_2026-11-26 00:27 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/noasig_on (35dd0ed0) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `noasig_on: 378/384 ok  media 37.21  mediana 36.73  mejor 32.63  fallas: n3/s27 n3/s3 n3/s45 n4/s20 n4/s36 n5/s36`
- Paradas: collision 5, race_finished 379
- Reloj total: 566.2 s. INVÁLIDA por reloj (sin OOM ni carreras sin STOP, pero por encima del umbral de 560 s; el box se reinició ~23:02 PST y desde entonces las tandas de 384 tardan ~600-620 s: real2_cand 620.9 s, blq_base 603.2 s): corrió bajo twin.lock (espera de lock 3579 s, no incluida en el reloj; 0 procesos de banco ajenos al inicio), 566.2 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 8.12, mem máx 4516 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.93-7.55 (media 4.58).
- Tipo de prueba: carrera completa FF 4.28/FD 5.8, banco nuevo con latencia, 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y semillas.

- Comparación (mismo tipo: carrera completa FF 4.28/FD 5.8, mismas semillas n1-8 x s1-48; noasig_base es el A/B directo: mismo árbol y comando, sin DIGITAL_MAP_LOOSE; vis2_base/real2_base/regr_main solo como contexto, difieren en código y/o gemelo):
  - vs noasig_base (marcada INVÁLIDA solo por reloj tras el reinicio del box, sin OOM ni carreras sin STOP; los total_time_s son tiempo simulado, tómalo como orientativo): 378/384 ok, media 37.21, mediana 36.73, mejor 32.63 -> noasig_on 378/384 ok, media 37.21 (+0.00), mediana 36.73 (+0.00), mejor 32.63; fallas 6 vs 6; fallan solo en noasig_on: n3/s3; fallan solo en noasig_base: n5/s3; fallas en ambas: n3/s27 n3/s45 n4/s20 n4/s36 n5/s36
  - vs vis2_base: 378/384 ok, media 37.21, mediana 36.73, mejor 32.63 -> noasig_on 378/384 ok, media 37.21 (+0.00), mediana 36.73 (+0.00), mejor 32.63; fallas 6 vs 6; fallas en ambas: n3/s27 n3/s45 n4/s20 n4/s36 n5/s36

Pregunta / qué mandar después: con el A/B contra noasig_base arriba, dime si DIGITAL_MAP_LOOSE queda como candidato; si sí, lo siguiente útil es una repetición con otras semillas (p. ej. s49-96) de ambas ramas para separar el efecto del ruido, o la variante que quieras probar encima.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/noasig_on.md`.
