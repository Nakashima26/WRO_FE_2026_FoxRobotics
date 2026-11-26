# Respuesta: par1125_base

_2026-11-25 23:55 PST (Tijuana)_

Recibido. La tanda **par1125_base** está en cola bajo twin.lock (estado ESPERA_LOCK).

- Código: origin/codex/par fijado en **ae48bef1** ("Rescata un eco lateral compatible cuando falla el par"), el HEAD de la rama al llegar el payload. Worktree ya preparado.
- Comando: exactamente el que mandaste: n1-8 x s1-48 (384 carreras), -P 7, FF 4.28 / FD 5.8, TWIN_PI_EXTRA {"ROUTE_MOTOR_CURVE_LIVE": false, "ROUTE_FULL_HARD_CLEAR_MM": 110.0}, y luego resumen.py.
- Tipo de prueba: carrera completa, banco nuevo con latencia; solo la compararé con tandas del mismo tipo y semillas (fhc110 tiene el mismo comando y TWIN_PI_EXTRA, pero otro código; fhc_base y regr_main como contexto).
- Cola: delante hay una tanda corriendo (regr_main_pilap) y 7 esperando el lock (regr_base7a, regr_mid2_bc1d, noasig_base, noasig_on, fhc_base, fhc110, fhc110_l2off). Con ~600 s por tanda de 384, esta probablemente tardará más de una hora en empezar.

Cuando termine, el resultado irá a `grok/resultados:resultados/par1125_base.md` (línea de resumen.py, STOP y COSTO por carrera, reloj total, validez) y añadiré aquí una sección de actualización.

Qué mandar después: nada por ahora. Si mueves codex/par antes de que arranque y quieres el nuevo commit, mándalo con otro tag.
