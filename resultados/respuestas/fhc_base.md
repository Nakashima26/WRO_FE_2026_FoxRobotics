# Respuesta: fhc_base

_2026-11-26 00:45 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/fhc_base (ea27d3ea) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `fhc_base: 378/384 ok  media 37.21  mediana 36.73  mejor 32.63  fallas: n3/s27 n3/s45 n4/s20 n4/s36 n5/s3 n5/s36`
- Paradas: collision 4, race_finished 380
- Reloj total: 564.6 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (espera de lock 4086 s, no incluida en el reloj; 0 procesos de banco ajenos al inicio), 564.6 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 8.05, mem máx 4518 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.82-7.74 (media 4.56).
- Tipo de prueba: carrera completa FF 4.28/FD 5.8, banco nuevo con latencia, 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y semillas.

- Comparación (mismo tipo: carrera completa FF 4.28/FD 5.8, mismas semillas n1-8 x s1-48; noasig_base es la referencia más cercana: mismo comando y TWIN_PI_EXTRA, su árbol solo añade el código opt-in DIGITAL_MAP_LOOSE apagado; noasig_on lleva DIGITAL_MAP_LOOSE true; vis2_base/real2_base/regr_main solo como contexto, difieren en código y/o gemelo):
  - vs vis2_base: 378/384 ok, media 37.21, mediana 36.73, mejor 32.63 -> fhc_base 378/384 ok, media 37.21 (+0.00), mediana 36.73 (+0.00), mejor 32.63; fallas 6 vs 6; fallas en ambas: n3/s27 n3/s45 n4/s20 n4/s36 n5/s3 n5/s36

Pregunta / qué mandar después: noasig_base todavía no tiene resultado válido publicado en grok/resultados (sigue en la cola del twin.lock); cuando salga, compara su línea de resumen.py con la de esta tanda (mismas 384 carreras y semillas). Mientras tanto, mándame la rama candidata (grok/t/fhc_<variante>) con el mismo comando para el A/B contra fhc_base.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/fhc_base.md`.
