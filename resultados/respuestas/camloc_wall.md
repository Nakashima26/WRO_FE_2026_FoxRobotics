# Respuesta: camloc_wall

_2026-11-26 00:00 PST (Tijuana)_

Recibida la tanda camloc_wall. Hecho hasta ahora:

- Worktree fijado en origin/grok/t/camloc_wall = 49d6f97c ("grok: tanda camloc_wall (parche de prueba, no mergear)").
- Comando preparado tal cual lo mandaste: n1-8 x s1-48 (384 carreras), -P 7, FF 4.28 / FD 5.8, TWIN_PI_EXTRA {"ROUTE_MOTOR_CURVE_LIVE": false, "LOC_CAM_WALL": true}, TWIN_COST_FIRST=1.95.
- Estado: en cola del twin.lock (solo corre una tanda a la vez en el box). Hay unas 11 tandas por delante (regr_*, noasig_*, fhc*, par1125_base, par1125_single45 y camloc_base si llegó antes), así que puede tardar bastante.
- Tipo de prueba: carrera completa FF 4.28/FD 5.8, banco nuevo con latencia, 384 carreras; solo la comparo con tandas de ese mismo tipo y semillas (camloc_base es el A/B directo).

Cuando termine, el resultado sale en `grok/resultados:resultados/camloc_wall.md` (resumen.py, STOP y COSTO por carrera, reloj total, validez) y añado una actualización a esta respuesta con la comparación. No hace falta reenviar nada.

## Actualización 2026-11-26 10:01 PST (Tijuana)


Hecho: corrí la tanda en origin/grok/t/camloc_wall (49d6f97c) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `camloc_wall: 376/384 ok  media 36.92  mediana 36.38  mejor 31.67  fallas: n2/s20 n2/s36 n3/s12 n3/s36 n4/s36 n5/s36 n6/s26 n8/s20`
- Paradas: collision 5, race_finished 378, stuck 1
- Reloj total: 29403.5 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (espera de lock 6781 s, no incluida en el reloj; 0 procesos de banco ajenos al inicio), 29403.5 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 13.80, mem máx 13768 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.75-744.68 (media 15.64).
- Tipo de prueba: carrera completa FF 4.28/FD 5.8, banco nuevo con latencia, 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y semillas.

- Comparación (mismo tipo: carrera completa FF 4.28/FD 5.8, mismas semillas n1-8 x s1-48; camloc_base, si aparece, es el A/B directo; las demás son contexto de otras ramas):
  - vs fhc110: 45/384 ok, media 40.04, mediana 39.38, mejor 34.59 -> camloc_wall 376/384 ok, media 36.92 (-3.12), mediana 36.38 (-3.00), mejor 31.67; fallas 8 vs 339; fallan solo en camloc_wall: n6/s26; fallan solo en fhc110: n1/s1 n1/s10 n1/s11 n1/s12 n1/s13 n1/s15 n1/s16 n1/s17 n1/s18 n1/s19 n1/s2 n1/s20 n1/s21 n1/s22 n1/s23 n1/s24 n1/s25 n1/s27 n1/s28 n1/s29 n1/s3 n1/s30 n1/s32 n1/s33 n1/s34 n1/s35 n1/s36 n1/s37 n1/s38 n1/s39 n1/s4 n1/s40 n1/s42 n1/s43 n1/s44 n1/s45 n1/s47 n1/s48 n1/s5 n1/s7 n1/s8 n1/s9 n2/s1 n2/s10 n2/s11 n2/s12 n2/s13 n2/s14 n2/s15 n2/s16 n2/s17 n2/s18 n2/s19 n2/s2 n2/s21 n2/s22 n2/s23 n2/s24 n2/s25 n2/s27 n2/s28 n2/s3 n2/s30 n2/s32 n2/s33 n2/s34 n2/s35 n2/s37 n2/s38 n2/s39 n2/s4 n2/s40 n2/s42 n2/s43 n2/s44 n2/s45 n2/s47 n2/s48 n2/s5 n2/s6 n2/s7 n2/s8 n2/s9 n3/s1 n3/s10 n3/s11 n3/s13 n3/s14 n3/s16 n3/s17 n3/s18 n3/s19 n3/s2 n3/s20 n3/s21 n3/s22 n3/s23 n3/s24 n3/s25 n3/s27 n3/s28 n3/s29 n3/s3 n3/s30 n3/s32 n3/s33 n3/s34 n3/s35 n3/s37 n3/s38 n3/s39 n3/s4 n3/s40 n3/s42 n3/s43 n3/s44 n3/s45 n3/s47 n3/s48 n3/s5 n3/s6 n3/s7 n3/s8 n3/s9 n4/s1 n4/s10 n4/s11 n4/s12 n4/s13 n4/s14 n4/s15 n4/s16 n4/s17 n4/s18 n4/s19 n4/s2 n4/s20 n4/s21 n4/s22 n4/s23 n4/s24 n4/s25 n4/s28 n4/s29 n4/s3 n4/s30 n4/s32 n4/s33 n4/s34 n4/s35 n4/s37 n4/s38 n4/s39 n4/s4 n4/s40 n4/s41 n4/s42 n4/s43 n4/s44 n4/s45 n4/s47 n4/s48 n4/s5 n4/s6 n4/s7 n4/s9 n5/s1 n5/s10 n5/s11 n5/s13 n5/s15 n5/s16 n5/s17 n5/s18 n5/s19 n5/s2 n5/s20 n5/s21 n5/s22 n5/s23 n5/s24 n5/s25 n5/s27 n5/s28 n5/s29 n5/s3 n5/s30 n5/s32 n5/s33 n5/s34 n5/s35 n5/s37 n5/s38 n5/s39 n5/s4 n5/s40 n5/s42 n5/s43 n5/s44 n5/s45 n5/s47 n5/s48 n5/s5 n5/s7 n5/s9 n6/s1 n6/s10 n6/s11 n6/s12 n6/s13 n6/s14 n6/s15 n6/s16 n6/s17 n6/s18 n6/s19 n6/s2 n6/s20 n6/s21 n6/s22 n6/s23 n6/s24 n6/s25 n6/s27 n6/s28 n6/s29 n6/s3 n6/s30 n6/s32 n6/s33 n6/s34 n6/s35 n6/s36 n6/s37 n6/s38 n6/s39 n6/s4 n6/s40 n6/s42 n6/s43 n6/s44 n6/s45 n6/s47 n6/s48 n6/s5 n6/s6 n6/s7 n6/s9 n7/s1 n7/s10 n7/s11 n7/s13 n7/s14 n7/s15 n7/s16 n7/s17 n7/s18 n7/s19 n7/s2 n7/s20 n7/s21 n7/s22 n7/s23 n7/s24 n7/s25 n7/s27 n7/s28 n7/s3 n7/s30 n7/s32 n7/s33 n7/s34 n7/s35 n7/s36 n7/s37 n7/s38 n7/s39 n7/s4 n7/s40 n7/s42 n7/s43 n7/s44 n7/s45 n7/s47 n7/s48 n7/s5 n7/s6 n7/s7 n7/s8 n7/s9 n8/s1 n8/s10 n8/s11 n8/s12 n8/s13 n8/s14 n8/s15 n8/s16 n8/s17 n8/s18 n8/s19 n8/s2 n8/s21 n8/s22 n8/s23 n8/s24 n8/s25 n8/s27 n8/s28 n8/s3 n8/s30 n8/s32 n8/s33 n8/s34 n8/s35 n8/s36 n8/s37 n8/s38 n8/s39 n8/s4 n8/s40 n8/s42 n8/s43 n8/s44 n8/s45 n8/s47 n8/s48 n8/s5 n8/s6 n8/s7 n8/s8 n8/s9

Qué mandar después: si camloc_base no aparece arriba es porque seguía en la cola del twin.lock; compara su línea de resumen.py en grok/resultados cuando salga (A/B directo de LOC_CAM_WALL). Si LOC_CAM_WALL baja fallas sin subir la media, repítelo con la réplica de 78 semillas antes de proponerlo para main.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/camloc_wall.md`.
