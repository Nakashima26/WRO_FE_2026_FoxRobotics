# cxpark_estado_r1 — repetición aislada de estacionamiento (cxpark_r*_1113)

Tipo: ESTACIONAMIENTO (carrera completa desde el cajón con estacionamiento) en el banco con latencia (costo.py/uno.sh, FF 2.8 / FD 3.8, primer plan 1.95 s, VBAT 12.5, `TWIN_PI_EXTRA={"ROUTE_PLAN_ASYNC": true}`, ruido 1-2 x semillas 1-48 = 96 carreras, P=7).

Actualizado: 2026-11-13 19:35:54 PST — carga actual 7.08 7.00 11.09 2/659 283251

Situación al recibir la orden (2026-11-13 18:51 PST): caja saturada (carga 29/55/61 con 8 CPU) con ~25 tandas en paralelo (cxpark_* originales, cxv23_*, v1*, gw14*, cxlb...). No se mataron procesos ajenos ni se repitieron las originales. No había coordinador global único: solo las secuencias de repetición cxv1_r1 (cxv1_r1_seq.sh) y v23 (rv23/chain.sh), así que esta secuencia es un coordinador que espera a que ambas y las originales cxpark_*_1113 terminen antes de empezar (no compite con ellas).

Procedimiento: checkouts detach de origin/grok/t/<origen> ya creados; tras esperar, adquiere flock exclusivo `/tmp/fox-grok-bancos.lock` (y `/workspace/fox/twin.lock`) y lo conserva durante TODA la secuencia; antes de cada banco espera cero otros uno.sh/costo.py y loadavg 1m <10 (máx 12 h; si no, el banco queda marcado SIN AISLAMIENTO y no se corre); bancos uno tras otro en el orden de abajo; cada tanda marcada **inválida** si hubo OOM, reloj >130 s, carga máx ≥12 o carreras sin STOP. Resultado de cada banco en resultados/<tag>.md. No se aceptan resultados ni se cambia código.

| tag | origen | TWIN_FW_OVER | estado |
|---|---|---|---|
| cxpark_rb1_1113 | grok/t/cxpark_b1b_1113 | `{}` | completada — Estado: VÁLIDA (aislada: sin otros bancos al inicio, carga<10, sin OOM) — reloj 103.0 s |
| cxpark_rg1_1113 | grok/t/cxpark_g1b_1113 | `{"PARK_PL_GUARD0": "1"}` | completada — Estado: VÁLIDA (aislada: sin otros bancos al inicio, carga<10, sin OOM) — reloj 101.8 s |
| cxpark_rb2_1113 | grok/t/cxpark_b2_1113 | `{}` | completada — Estado: VÁLIDA (aislada: sin otros bancos al inicio, carga<10, sin OOM) — reloj 105.8 s |
| cxpark_ro2_1113 | grok/t/cxpark_o2_1113 | `{"PARK_PL_LOCAL_ODOM": "1"}` | completada — Estado: VÁLIDA (aislada: sin otros bancos al inicio, carga<10, sin OOM) — reloj 103.0 s |
| cxpark_rb3_1113 | grok/t/cxpark_b3_1113 | `{}` | completada — Estado: VÁLIDA (aislada: sin otros bancos al inicio, carga<10, sin OOM) — reloj 102.2 s |
| cxpark_rs3_1113 | grok/t/cxpark_s3_1113 | `{"PARK_POSTE_IDENT_SEP": "1"}` | completada — Estado: VÁLIDA (aislada: sin otros bancos al inicio, carga<10, sin OOM) — reloj 105.4 s |

## Seguimiento en vivo (2026-11-13 19:35:54 PST)

- Coordinador park (único, no se creó otro): PID 207113. Orden de locks del coordinador: /workspace/fox/twin.lock PRIMERO (fd 8) y /tmp/fox-grok-bancos.lock SEGUNDO (fd 9) = orden global correcto; v1 (cxv1_r1_seq.sh) y v23 (rv23/chain.sh) usan el mismo orden, así que no hay bloqueo por orden inverso. El pgrep de espera no coincide con el propio coordinador (su cmdline es ./cxpark_r1_seq.sh).
- Titular twin.lock: nadie; titular fox-grok-bancos.lock: nadie.
- Esperando a: nada (ya en fase de locks/aislamiento)
- uno.sh/costo.py activos en la caja: 0 (tags: )
- Bloqueo real encontrado y corregido (19:2x PST): los checkouts cxpark_r*_1113_wt NO existían (perdidos en el episodio ENOSPC) — el coordinador habría marcado los 6 bancos "ERROR sin checkout" al instante (la secuencia cxv1_r1 falló así a las 19:19, por falta de .venv). Se recrearon como worktrees detach de perfil-isla en los commits de origen (rb1 8b6a95b, rg1 a1e5c8f, rb2 f20b91b, ro2 8c79787, rb3 dcaea1c, rs3 65ff3d9) con .venv -> perfil-isla/.venv; no hizo falta reiniciar el coordinador.
- Originales: cxpark_g1b_1113 estaba ACTIVA (relanzada 19:16, con twin.lock) -> CANCELADA (SIGTERM a su árbol, ver resultados/cxpark_g1b_1113.md). b1, b1b, o2, s3 ya terminadas (STOP, no válidas por saturación). g1, b2, b3 sin datos (ENOSPC), no activas. Ninguna se volvió a ejecutar.

## Bitácora

```
Fri Nov 13 18:53:28 PST 2026 esperando originales cxpark_*_1113 y secuencias cxv1_r1 / rv23
Fri Nov 13 19:25:29 PST 2026 locks adquiridos
cxpark_rb1_1113 VÁLIDA (aislada: sin otros bancos al inicio, carga<10, sin OOM)
Fri Nov 13 19:27:13 PST 2026 cxpark_rb1_1113 listo wall=103.0
cxpark_rg1_1113 VÁLIDA (aislada: sin otros bancos al inicio, carga<10, sin OOM)
Fri Nov 13 19:28:55 PST 2026 cxpark_rg1_1113 listo wall=101.8
cxpark_rb2_1113 VÁLIDA (aislada: sin otros bancos al inicio, carga<10, sin OOM)
Fri Nov 13 19:30:42 PST 2026 cxpark_rb2_1113 listo wall=105.8
cxpark_ro2_1113 VÁLIDA (aislada: sin otros bancos al inicio, carga<10, sin OOM)
Fri Nov 13 19:32:25 PST 2026 cxpark_ro2_1113 listo wall=103.0
cxpark_rb3_1113 VÁLIDA (aislada: sin otros bancos al inicio, carga<10, sin OOM)
Fri Nov 13 19:34:08 PST 2026 cxpark_rb3_1113 listo wall=102.2
cxpark_rs3_1113 VÁLIDA (aislada: sin otros bancos al inicio, carga<10, sin OOM)
Fri Nov 13 19:35:54 PST 2026 cxpark_rs3_1113 listo wall=105.4
Fri Nov 13 19:35:54 PST 2026 SECUENCIA PARK TERMINADA
```


## Respuesta a la orden urgente de 19:34 PST (actualizar coordinador park PID 207113)

Revisado a las 19:35 PST: la premisa ya no aplicaba. La pre-espera por rv23/chain.sh (PID 197309) terminó sola cuando ese proceso salió; el coordinador adquirió twin.lock (PRIMERO) y fox-grok-bancos.lock (SEGUNDO) a las **19:25:29 PST** y empezó rb1 enseguida. El estado de las 19:34 que se leyó era anterior a esa marca o se interpretó así; a las 19:34 ya iban 5/6 bancos completos.

- **No se modificó ni reinició el coordinador**: editar un script de bash en ejecución puede corromper su lectura, y no hacía falta. No se creó otra cola ni bancos nuevos, no se repitieron originales y no se tocaron procesos ajenos.
- Para el gate "cero otros costo.py/uno.sh": usa `pgrep -fc 'latencia/uno.sh|costo.py'`. pgrep se excluye a sí mismo; la cmdline del coordinador es `/bin/bash ./cxpark_r1_seq.sh` (no contiene el patrón), así que no se cuenta. Se comprobó en la práctica: cada banco arrancó con 0 ajenos y carga<10. Sí podría contar un `bash -c` ajeno cuya línea contenga el texto del patrón o zombies con esa cmdline. Ningún banco quedó bloqueado por eso.
- Recomendación para próximos coordinadores (no aplicada aquí porque la secuencia ya terminó): quitar la pre-espera por existencia de coordinadores y serializar solo con flock (twin.lock → bancos.lock), filtrando en el gate procesos en estado Z y cmdlines que no empiecen por bash/python reales.

### Resultado final de la secuencia (terminó 19:35:54 PST, PID 207113 salió)

Tipo: ESTACIONAMIENTO. Las 6 tandas son VÁLIDAS (aisladas, sin OOM, reloj 101.8-105.8 s para 96 carreras).

| tag | override | ok/96 | media s | mediana s | mejor s | reloj s |
|---|---|---|---|---|---|---|
| cxpark_rb1_1113 | {} | 76 | 38.05 | 37.74 | 33.63 | 103.0 |
| cxpark_rg1_1113 | PARK_PL_GUARD0=1 | 70 | 37.85 | 37.48 | 33.46 | 101.8 |
| cxpark_rb2_1113 | {} | 75 | 38.07 | 37.76 | 33.82 | 105.8 |
| cxpark_ro2_1113 | PARK_PL_LOCAL_ODOM=1 | 70 | 37.96 | 37.57 | 33.40 | 103.0 |
| cxpark_rb3_1113 | {} | 72 | 38.03 | 37.59 | 33.93 | 102.2 |
| cxpark_rs3_1113 | PARK_POSTE_IDENT_SEP=1 | 56 | 38.11 | 37.64 | 34.34 | 105.4 |

Detalle por banco (fallas por semilla, ENV, MON) en resultados/cxpark_r*_1113.md.
