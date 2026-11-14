# cxpark_estado_r1 — repetición aislada de estacionamiento (cxpark_r*_1113)

Tipo: ESTACIONAMIENTO (carrera completa desde el cajón con estacionamiento) en el banco con latencia (costo.py/uno.sh, FF 2.8 / FD 3.8, primer plan 1.95 s, VBAT 12.5, `TWIN_PI_EXTRA={"ROUTE_PLAN_ASYNC": true}`, ruido 1-2 x semillas 1-48 = 96 carreras, P=7).

Actualizado: 2026-11-13 19:22:33 PST — carga actual 2.42 6.66 17.04 1/669 265531

Situación al recibir la orden (2026-11-13 18:51 PST): caja saturada (carga 29/55/61 con 8 CPU) con ~25 tandas en paralelo (cxpark_* originales, cxv23_*, v1*, gw14*, cxlb...). No se mataron procesos ajenos ni se repitieron las originales. No había coordinador global único: solo las secuencias de repetición cxv1_r1 (cxv1_r1_seq.sh) y v23 (rv23/chain.sh), así que esta secuencia es un coordinador que espera a que ambas y las originales cxpark_*_1113 terminen antes de empezar (no compite con ellas).

Procedimiento: checkouts detach de origin/grok/t/<origen> ya creados; tras esperar, adquiere flock exclusivo `/tmp/fox-grok-bancos.lock` (y `/workspace/fox/twin.lock`) y lo conserva durante TODA la secuencia; antes de cada banco espera cero otros uno.sh/costo.py y loadavg 1m <10 (máx 12 h; si no, el banco queda marcado SIN AISLAMIENTO y no se corre); bancos uno tras otro en el orden de abajo; cada tanda marcada **inválida** si hubo OOM, reloj >130 s, carga máx ≥12 o carreras sin STOP. Resultado de cada banco en resultados/<tag>.md. No se aceptan resultados ni se cambia código.

| tag | origen | TWIN_FW_OVER | estado |
|---|---|---|---|
| cxpark_rb1_1113 | grok/t/cxpark_b1b_1113 | `{}` | pendiente |
| cxpark_rg1_1113 | grok/t/cxpark_g1b_1113 | `{"PARK_PL_GUARD0": "1"}` | pendiente |
| cxpark_rb2_1113 | grok/t/cxpark_b2_1113 | `{}` | pendiente |
| cxpark_ro2_1113 | grok/t/cxpark_o2_1113 | `{"PARK_PL_LOCAL_ODOM": "1"}` | pendiente |
| cxpark_rb3_1113 | grok/t/cxpark_b3_1113 | `{}` | pendiente |
| cxpark_rs3_1113 | grok/t/cxpark_s3_1113 | `{"PARK_POSTE_IDENT_SEP": "1"}` | pendiente |

## Seguimiento en vivo (2026-11-13 19:22:33 PST)

- Coordinador park (único, no se creó otro): PID 207113. Orden de locks del coordinador: /workspace/fox/twin.lock PRIMERO (fd 8) y /tmp/fox-grok-bancos.lock SEGUNDO (fd 9) = orden global correcto; v1 (cxv1_r1_seq.sh) y v23 (rv23/chain.sh) usan el mismo orden, así que no hay bloqueo por orden inverso. El pgrep de espera no coincide con el propio coordinador (su cmdline es ./cxpark_r1_seq.sh).
- Titular twin.lock: nadie; titular fox-grok-bancos.lock: nadie.
- Esperando a: 197309 bash /workspace/fox/rv23/chain.sh;
- uno.sh/costo.py activos en la caja: 0 (tags: )
- Bloqueo real encontrado y corregido (19:2x PST): los checkouts cxpark_r*_1113_wt NO existían (perdidos en el episodio ENOSPC) — el coordinador habría marcado los 6 bancos "ERROR sin checkout" al instante (la secuencia cxv1_r1 falló así a las 19:19, por falta de .venv). Se recrearon como worktrees detach de perfil-isla en los commits de origen (rb1 8b6a95b, rg1 a1e5c8f, rb2 f20b91b, ro2 8c79787, rb3 dcaea1c, rs3 65ff3d9) con .venv -> perfil-isla/.venv; no hizo falta reiniciar el coordinador.
- Originales: cxpark_g1b_1113 estaba ACTIVA (relanzada 19:16, con twin.lock) -> CANCELADA (SIGTERM a su árbol, ver resultados/cxpark_g1b_1113.md). b1, b1b, o2, s3 ya terminadas (STOP, no válidas por saturación). g1, b2, b3 sin datos (ENOSPC), no activas. Ninguna se volvió a ejecutar.

## Bitácora

```
Fri Nov 13 18:53:28 PST 2026 esperando originales cxpark_*_1113 y secuencias cxv1_r1 / rv23
```
