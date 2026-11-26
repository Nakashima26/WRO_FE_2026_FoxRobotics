# Respuesta: noasig_base

_2026-11-26 00:17 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/noasig_base (db432326) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `noasig_base: 378/384 ok  media 37.21  mediana 36.73  mejor 32.63  fallas: n3/s27 n3/s45 n4/s20 n4/s36 n5/s3 n5/s36`
- Paradas: collision 4, race_finished 380
- Reloj total: 606.4 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (espera de lock 3026 s, no incluida en el reloj; 0 procesos de banco ajenos al inicio), 606.4 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 7.76, mem máx 4502 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 4.06-8.24 (media 4.89).
- Tipo de prueba: carrera completa FF 4.28/FD 5.8, banco nuevo con latencia, 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y semillas.

- Comparación (mismo tipo: carrera completa FF 4.28/FD 5.8, mismas semillas n1-8 x s1-48; vis2_base/real2_base/regr_main como contexto; difieren en código y/o gemelo):
  - vs vis2_base: 378/384 ok, media 37.21, mediana 36.73, mejor 32.63 -> noasig_base 378/384 ok, media 37.21 (+0.00), mediana 36.73 (+0.00), mejor 32.63; fallas 6 vs 6; fallas en ambas: n3/s27 n3/s45 n4/s20 n4/s36 n5/s3 n5/s36

Qué mandar después: las variantes (p. ej. con DIGITAL_MAP_LOOSE activo) contra noasig_base con este mismo comando y semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/noasig_base.md`.
