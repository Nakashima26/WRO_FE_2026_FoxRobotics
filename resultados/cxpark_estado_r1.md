# cxpark_estado_r1 — repetición aislada de estacionamiento (cxpark_r*_1113)

Tipo: ESTACIONAMIENTO (carrera completa desde el cajón con estacionamiento) en el banco con latencia (costo.py/uno.sh, FF 2.8 / FD 3.8, primer plan 1.95 s, VBAT 12.5, `TWIN_PI_EXTRA={"ROUTE_PLAN_ASYNC": true}`, ruido 1-2 x semillas 1-48 = 96 carreras, P=7).

Actualizado: 2026-11-13 18:53:28 PST — carga actual 33.74 48.48 57.70 35/850 207129

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

## Bitácora

```
Fri Nov 13 18:53:28 PST 2026 esperando originales cxpark_*_1113 y secuencias cxv1_r1 / rv23
```
