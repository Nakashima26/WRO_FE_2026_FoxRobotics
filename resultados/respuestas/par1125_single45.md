# Respuesta: par1125_single45

_2026-11-26 01:51 PST (Tijuana)_

Hecho: corrí la tanda en origin/codex/par (ae48bef1) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `par1125_single45: 47/384 ok  media 39.36  mediana 38.85  mejor 35.47  fallas: n1/s1 n1/s11 n1/s12 n1/s13 n1/s14 n1/s15 n1/s16 n1/s17 n1/s18 n1/s19 n1/s2 n1/s20 `
- Paradas: collision 334, race_finished 48, stuck 2
- Reloj total: 1005.2 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (espera de lock 6136 s, no incluida en el reloj; 0 procesos de banco ajenos al inicio), 1005.2 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 8.00, mem máx 13800 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.91-68.67 (media 5.73).
- Tipo de prueba: carrera completa FF 4.28/FD 5.8, banco nuevo con latencia, 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y semillas.

- Comparación (mismo tipo: carrera completa FF 4.28/FD 5.8, mismas semillas n1-8 x s1-48; par1125_base es el A/B directo (mismo commit ae48bef1 de codex/par y mismo comando sin LOC_L23_PAIR_SINGLE_MM); fhc110 es el mismo comando en otra rama; fhc110_l2off y fhc_base son variantes del mismo banco; las demás solo como contexto):
  - vs fhc110: 45/384 ok, media 40.04, mediana 39.38, mejor 34.59 -> par1125_single45 47/384 ok, media 39.36 (-0.68), mediana 38.85 (-0.53), mejor 35.47; fallas 337 vs 339; fallan solo en par1125_single45: n1/s14 n1/s6 n3/s15 n3/s26 n3/s41 n4/s27 n5/s12 n5/s14 n5/s26 n5/s6 n5/s8 n6/s8 n7/s12 n7/s29 n8/s29; fallan solo en fhc110: n1/s10 n2/s14 n2/s15 n2/s37 n2/s6 n3/s14 n3/s19 n3/s20 n3/s27 n3/s6 n4/s14 n4/s41 n4/s6 n6/s29 n7/s10 n7/s14 n8/s6

Qué mandar después: si par1125_base no aparece arriba, compárala con su línea en grok/resultados cuando salga (A/B directo). Si 45 mm ayuda, prueba otros valores de LOC_L23_PAIR_SINGLE_MM (p. ej. 35 / 55 mm) en el mismo commit — mismas 384 carreras y semillas. Si fhc110 / fhc110_l2off / fhc_base no aparecen arriba es porque seguían en la cola del twin.lock; compara sus líneas de resumen.py en grok/resultados cuando salgan.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/par1125_single45.md`.
