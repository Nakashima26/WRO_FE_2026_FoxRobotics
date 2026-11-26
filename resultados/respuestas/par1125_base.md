# Respuesta: par1125_base

_2026-11-25 23:55 PST (Tijuana)_

Recibido. La tanda **par1125_base** está en cola bajo twin.lock (estado ESPERA_LOCK).

- Código: origin/codex/par fijado en **ae48bef1** ("Rescata un eco lateral compatible cuando falla el par"), el HEAD de la rama al llegar el payload. Worktree ya preparado.
- Comando: exactamente el que mandaste: n1-8 x s1-48 (384 carreras), -P 7, FF 4.28 / FD 5.8, TWIN_PI_EXTRA {"ROUTE_MOTOR_CURVE_LIVE": false, "ROUTE_FULL_HARD_CLEAR_MM": 110.0}, y luego resumen.py.
- Tipo de prueba: carrera completa, banco nuevo con latencia; solo la compararé con tandas del mismo tipo y semillas (fhc110 tiene el mismo comando y TWIN_PI_EXTRA, pero otro código; fhc_base y regr_main como contexto).
- Cola: delante hay una tanda corriendo (regr_main_pilap) y 7 esperando el lock (regr_base7a, regr_mid2_bc1d, noasig_base, noasig_on, fhc_base, fhc110, fhc110_l2off). Con ~600 s por tanda de 384, esta probablemente tardará más de una hora en empezar.

Cuando termine, el resultado irá a `grok/resultados:resultados/par1125_base.md` (línea de resumen.py, STOP y COSTO por carrera, reloj total, validez) y añadiré aquí una sección de actualización.

Qué mandar después: nada por ahora. Si mueves codex/par antes de que arranque y quieres el nuevo commit, mándalo con otro tag.

## Actualización 2026-11-26 01:34 PST (Tijuana)


Hecho: corrí la tanda en origin/codex/par (ae48bef1, fijado al recibir el payload) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `par1125_base: 47/384 ok  media 39.99  mediana 38.95  mejor 35.53  fallas: n1/s1 n1/s11 n1/s12 n1/s13 n1/s14 n1/s15 n1/s16 n1/s17 n1/s18 n1/s19 n1/s2 n1/s20 n1/s`
- Paradas: collision 335, race_finished 47, stuck 2
- Reloj total: 834.7 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (espera de lock 5308 s, no incluida en el reloj; 0 procesos de banco ajenos al inicio), 834.7 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 7.92, mem máx 14317 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.65-14.71 (media 5.18).
- Tipo de prueba: carrera completa FF 4.28/FD 5.8, banco nuevo con latencia, 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y semillas.

- Comparación (mismo tipo: carrera completa FF 4.28/FD 5.8, mismas semillas n1-8 x s1-48; fhc110 usa el mismo comando y TWIN_PI_EXTRA pero otro código (rama grok/t/fhc110), así que la diferencia mezcla el cambio de codex/par con el resto del árbol; fhc_base y regr_main solo como contexto):
  - vs fhc110: 45/384 ok, media 40.04, mediana 39.38, mejor 34.59 -> par1125_base 47/384 ok, media 39.99 (-0.05), mediana 38.95 (-0.43), mejor 35.53; fallas 337 vs 339; fallan solo en par1125_base: n1/s14 n1/s6 n3/s15 n3/s26 n3/s41 n4/s26 n4/s27 n5/s14 n5/s8 n7/s12 n7/s29; fallan solo en fhc110: n1/s10 n2/s37 n3/s19 n3/s27 n3/s7 n4/s10 n4/s41 n4/s6 n5/s15 n6/s6 n7/s10 n7/s14 n8/s6

Pregunta / qué mandar después: esta es la base de codex/par (ae48bef1). Si fhc110/fhc_base no aparecen arriba es porque seguían en la cola del twin.lock al terminar; compáralos en grok/resultados cuando salgan. Para aislar el efecto del rescate de eco lateral, manda la misma tanda sobre el commit padre de ae48bef1 (o con el rescate desactivado) con otro tag.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/par1125_base.md`.
