# Respuesta: fhc110_l2off

_2026-11-26 01:20 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/fhc110_l2off (dfe4e7e2) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `fhc110_l2off: 44/384 ok  media 39.49  mediana 39.31  mejor 35.10  fallas: n1/s1 n1/s11 n1/s13 n1/s15 n1/s16 n1/s17 n1/s18 n1/s19 n1/s2 n1/s20 n1/s21 n1/s22 n1/s`
- Paradas: collision 340, race_finished 44
- Reloj total: 1696.8 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (espera de lock 4703 s, no incluida en el reloj; 0 procesos de banco ajenos al inicio), 1696.8 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 8.86, mem máx 13790 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 4.09-15.63 (media 5.81).
- Tipo de prueba: carrera completa FF 4.28/FD 5.8, banco nuevo con latencia, 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y semillas.

- Comparación (mismo tipo: carrera completa FF 4.28/FD 5.8, mismas semillas n1-8 x s1-48; fhc110 es la referencia A/B directa (mismo comando sin ROUTE_L23_SIGN_MM 0.0) si su rama tiene el mismo árbol; fhc_base es la base sin ROUTE_FULL_HARD_CLEAR_MM; las demás solo como contexto):
  - vs fhc110: 45/384 ok, media 40.04, mediana 39.38, mejor 34.59 -> fhc110_l2off 44/384 ok, media 39.49 (-0.55), mediana 39.31 (-0.07), mejor 35.10; fallas 340 vs 339; fallan solo en fhc110_l2off: n1/s6 n2/s29 n3/s15 n4/s8 n5/s6 n5/s8 n7/s12 n8/s29 n8/s41; fallan solo en fhc110: n1/s10 n1/s12 n3/s8 n4/s15 n4/s41 n5/s10 n6/s14 n8/s12

Pregunta / qué mandar después: si fhc110 o fhc_base aún no aparecen arriba es porque seguían en la cola del twin.lock cuando terminó esta tanda; compara sus líneas de resumen.py en grok/resultados cuando salgan (mismas 384 carreras y semillas). Dime si quieres otra variante de ROUTE_L23_SIGN_MM.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/fhc110_l2off.md`.
