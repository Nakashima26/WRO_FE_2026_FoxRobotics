# cxv23 — estado r2 (RONDA2 v23, pareja rb06 + rlos3) — consulta SOLO LECTURA

Actualizado: 14/11/2026 03:27 (hora de Tijuana, UTC-8). Solicitado por: codex-v23 (webhook 03:27, consulta de estado de la solicitud e4bab3ec-a6ff-4de7-aaef-82757085926f).
Tipo de prueba: carrera completa desde el cajón (3 vueltas), banco con latencia (96 carreras = ruido 1-2 × semillas 1-48, TWIN_VBAT=12.5, ROUTE_PLAN_ASYNC=true, P=7). Solo comparable con otras cxv23 del mismo banco.

## ¿Sigue activo? NO
- La pareja rb06/rlos3 **no se volvió a ejecutar** en RONDA2: ambas tandas ya tenían resultado **válido** del 13/11 (rb06 empezó 19:36, rlos3 19:38; secuenciales bajo flock /workspace/fox/twin.lock). Se publicaron tal cual (ver resultados/cxv23_rb06.md y resultados/cxv23_rlos3.md, y el estado r2 previo de las 00:40).
- PID: **n/a**, no hay ningún proceso costo.py / uno.sh / chain.sh / coordinador v23 / pytest vivo (ps a las 03:27).
- Locks: /workspace/fox/twin.lock y /tmp/fox-grok-bancos.lock **libres** (flock -n OK). El último banco de la caja (cxpark_r2lead_c_1114a, otro tipo de prueba) terminó ~03:11.
- Carga: 0.35 1.38 2.86. Disco: 66 GB libres de 128 GB.
- Logs en la caja: rv23/cxv23_rb06.log y rv23/cxv23_rlos3.log (vacíos; la salida va a rv23/cxv23_rb06_md.log / rv23/cxv23_rlos3_md.log y a los .md publicados). Worktrees: cxv23_rb06_wt, cxv23_rlos3_wt.

## Resultados (ya válidos, sin cambios)
| tanda | fuente | extra | reloj 96 | resultado |
|---|---|---|---|---|
| cxv23_rb06 | origin/grok/t/cxv23_b06 (5784bc9) | ROUTE_PLAN_ASYNC | 103.1 s | 75/96 ok, media 38.12, mediana 37.78 |
| cxv23_rlos3 | origin/grok/t/cxv23_los3 (46159f9) | ROUTE_PLAN_ASYNC + ROUTE_BODY_LOS_MM=1400 | 102.6 s | 71/96 ok, media 38.07, mediana 37.73 |

## pytest
No se corrió la suite pure_pursuit/twin/tests: la condición era "si ejecutas la pareja", y la pareja no se ejecutó. Si se quiere el conteo igualmente, pedirlo explícitamente.

## Qué no se hizo
No se crearon ni reiniciaron bancos, no se lanzaron otras parejas, no se cambiaron parámetros ni se mató ningún proceso.
