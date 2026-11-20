# cxv1_desbloqueo — diagnóstico de la cola v1 (cxv1_*_1113a_r1)

Fecha: 2026-11-13 19:44 PST. Tipo de prueba: repetición aislada de latencia (bancos cxv1 *_1113a_r1, 96 carreras, VBAT 12.5). No se lanzaron bancos nuevos.

## Causa (no era un bloqueo de locks)
- La cola **no estaba colgada**: `cxv1_r1_seq.sh` ya había terminado ("SECUENCIA TERMINADA", 19:19:21 PST). No había waiter, flock ni sleep v1 vivos y /proc/locks no tenía nada de v1. No hubo inversión de locks ni un detector de procesos que se contara a sí mismo.
- **1er intento (locks 19:09:15):** lock_base salió INVÁLIDA (85 carreras sin STOP, wall 21.4 s) y lock3 también (96 sin STOP, wall 7.9 s). A las 19:12 el disco se llenó (**ENOSPC**, "No space left on device") y fallaron .md/_put.json de debt_base, debt, stale_base y stale.
- **2º intento (locks 19:18:50):** las seis tandas fallaron al instante (wall 0.1 s, 96/96 sin STOP) con `../../../.venv/bin/python: No such file or directory`. Los seis worktrees `cxv1_*_1113a_r1_wt/.venv` eran symlinks a `/workspace/fox/gH85_wt/.venv`, que ya no existe (se borró, al parecer al liberar espacio).
- No se publicó ningún cxv1_r1 porque el Mac de Jesse está offline. Los seis _put.json (todos INVÁLIDOS) quedaron en pend/ sin subir.

## Corrección (solo dentro de lo nuestro)
- Los symlinks `.venv` de los seis worktrees cxv1_*_1113a_r1 ahora apuntan a `/workspace/fox/perfil-isla/.venv`, el mismo venv que usa la cadena rv23 (cxv23_rb05 terminó bien con él, wall 104.3 s).
- `cxv1_r1_seq.sh`: se quitó `/tmp/fox-grok-bancos.lock`, así que solo serializa con `/workspace/fox/twin.lock`. Esto evita la posible inversión con rv23/chain.sh, que toma twin y luego bancos. También se añadió un aborto explícito si falta `.venv/bin/python`, para no volver a generar 96 carreras vacías. Respaldo: `cxv1_r1_seq.sh.bak_1941`.
- Los seis _put.json inválidos se movieron de pend/ a `invalid_cxv1_r1_1919/` para que no se publiquen como resultado.
- No se tocaron procesos ajenos ni carreras activas.

## Estado
- Hay **exactamente una** cola v1 reiniciada (PID 291014, 19:42 PST) con los mismos seis tags: lock_base, lock3, debt_base, debt, stale_base y stale (*_1113a_r1). Ninguna de ellas tenía antes una carrera válida, así que no se duplica nada.
- Ahora espera a `twin.lock`, que tiene rv23/chain.sh (rec4: rb05 terminó a las 19:42:14 PST, sigue cxv23_rg14c). Cuando se libere, cada tanda vuelve a exigir aislamiento: cero uno.sh/costo.py ajenos y carga<10.
- Disco ahora: 73 GB libres (43 % usado). /tmp/lat ocupa 11 GB.
- Los resultados irán a pend/ (o a grok/resultados cuando el Mac vuelva a estar en línea).
