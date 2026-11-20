# cxv1_r2_estado — inspección (SOLO INSPECCION, no se lanzó ni duplicó ningún banco)

Consulta de codex-v1, revisada el sáb 14 nov 2026 a las 05:05 PST (box). Esta consulta NO es otro banco.

## Resumen
| tag | run | estado | commit | inicio → fin (PST) | reloj | ok/96 | media | mediana | mejor |
|---|---|---|---|---|---|---|---|---|---|
| cxv1_r2_lock_b_1114a | b2d469ae-e4da-41d0-9322-dc559dc37b34 | TERMINADA, VÁLIDA | c1a3358 | 00:53:02 → ~00:54:45 | 102.7 s | 67 (29 fallas) | 38.27 | 37.87 | 33.65 |
| cxv1_r2_lock_c_1114a | 15ec9b8b-f32b-4150-ae92-86b4329a5a4f | TERMINADA, VÁLIDA (reloj algo alto) | a44edc5 | 00:50:59 → 00:52:58 | 115.9 s | 69 (27 fallas) | 37.83 | 37.55 | 33.83 |

Tipo de prueba (ambas): carrera completa desde el cajón, banco NUEVO con latencia (costo.py/uno.sh, 3 vueltas), TWIN_VBAT=12.5, ruido 1-2 × semillas 1-48, P=7.
- b: extra {"ROUTE_PLAN_ASYNC":true}
- c: {"ROUTE_PLAN_ASYNC":true,"ROUTE_LAP1_LOCK_STATIONS":3}

Las dos corrieron en serie (c terminó 00:52:58, b tomó los locks y empezó 00:53:02), cada una bajo `flock -x /workspace/fox/twin.lock` → `flock -x /tmp/fox-grok-bancos.lock` (orden respetado). Sin OOM ni ENOSPC. c: reloj 115.9 s, un poco por encima de ~100-110 s; carga máx. durante la tanda 6.39 (<10), el script la marcó VÁLIDA, pero conviene tomar su latencia con algo de cautela.

Fallas b: n1/s2 s4 s5 s6 s7 s9 s10 s22 s26 s30 s33 s34 s36 s40; n2/s1 s2 s3 s7 s9 s10 s19 s20 s26 s30 s33 s34 s39 s44 s46.
Fallas c: n1/s2 s5 s9 s10 s19 s22 s26 s30 s33 s34 s37; n2/s2 s3 s5 s9 s19 s22 s26 s30 s31 s33 s34 s35 s36 s37 s39 s46.
Los detalles por carrera están en resultados/cxv1_r2_lock_b_1114a.md y resultados/cxv1_r2_lock_c_1114a.md (en cola, ver abajo).

## Procesos / PID
- No hay ningún proceso de banco activo (sin uno.sh, costo.py, python del twin, flock de banco ni coordinador cxv1). Los PID de b y c ya salieron.
- Coordinador antiguo cxv1r1 (cxv1_r1_seq.sh): no está corriendo. Su log cierra con "SECUENCIA TERMINADA" el vie 13 nov a las 20:34:28 PST.

## Locks (/proc/locks)
- /workspace/fox/twin.lock (inodo 121879): sin titular → LIBRE.
- /tmp/fox-grok-bancos.lock (inodo 7353): sin titular → LIBRE.
- Los demás FLOCK/POSIX que aparecen son del escritorio/daemons del box (xvfb, dbus, novnc, orbitd, host-main), ajenos a los bancos; no se tocaron.

## Carga / espacio (05:05 PST)
- load average: 0.76 1.54 2.41 (los últimos bancos del box terminaron ~04:22)
- RAM: 16002 MB total, 5470 MB disponibles, sin swap
- Disco /: 128 G, 68 G usados, 61 G libres (53 %)

## Coordinador antiguo cxv1r1: bancos ya hechos
| tag | reloj | estado |
|---|---|---|
| cxv1_lock_base_1113a_r1 | 99.7 s | VÁLIDA |
| cxv1_lock3_1113a_r1 | 102.8 s | VÁLIDA |
| cxv1_debt_base_1113a_r1 | 104.6 s | VÁLIDA |
| cxv1_debt_1113a_r1 | 2230.0 s | **INVÁLIDA** (reloj muy por encima de ~100-110 s) |
| cxv1_stale_base_1113a_r1 | 108.6 s | VÁLIDA |
| cxv1_stale_1113a_r1 | 105.4 s | VÁLIDA |

Sus resúmenes (resultados/<tag>.md) ya se generaron y están en cola para grok/resultados.

## Bloqueos
- Ninguno en el box: locks libres, nada corriendo, sin OOM.
- Publicación: la Mac de Jesse está offline, así que los push a grok/resultados quedan en cola en /workspace/fox/pend/ (este archivo, cxv1_r2_lock_b/c_1114a y los seis de cxv1 r1). Se suben cuando vuelva la Mac.

