# Optimización CPU R5 del SIL

Base: `4472197`. Rama: `opt-r5-sil`. Solo este worktree; sin cambios al `.ino` ni a la conducción.

## T1 — hecho

CPU user en serie (suma de seis parejas): **27.70 → 24.95 s (−9.93%)**.
Mejora en las seis parejas. Banco: **48/48 idénticas** frente a `4472197`.
Suite: **402 passed, 2 skipped, 1 xfailed** (ocho variantes adicionales).

Se consulta el anillo SIL directamente por cuadro y al registrar semillas. La
búsqueda inversa e interpolación nativa conservan las mismas operaciones y orden
que `_pose_at`, compiladas con `-ffp-contract=off`. No se vacía ni se copia la
historia en cada loop. Los escalares del encoder siguen sincronizándose.
La vista retiene el propietario de la biblioteca y se lee con el firmware suspendido.
Los tests comparan bytes IEEE-754 de historia, poses interpoladas, vehículo,
encoder, callbacks y puntos de colisión, con vaciado inmediato, cada 17 llamadas
y sin vaciado. El anillo mantiene la capacidad original de 20.000 muestras.

Variantes descartadas: aplazar solo el vaciado, 26,81 → 26,60 s (−0,78%, un empate);
vista recorrida desde Python, 26,71 → 29,31 s (+9,73%).

| Seed | Repetición | Base user (s) | T1 user (s) |
|---|---:|---:|---:|
| 5 | 1 | 4.44 | 3.96 |
| 5 | 2 | 4.55 | 4.00 |
| 5 | 3 | 4.50 | 4.06 |
| 13 | 1 | 4.88 | 4.32 |
| 13 | 2 | 4.66 | 4.33 |
| 13 | 3 | 4.67 | 4.28 |

## T2 — descartado

CPU incremental frente a T1: **24.17 → 24.01 s (−0.66%)**.
Mejora en 5/6 parejas; la primera empeora. No alcanza el 1% y no mejora de forma
consistente, por lo que se retiraron todos sus cambios de producción y tests.
No hay commit de T2. Se conservan `t2-descartado.patch`, `test_sil_tof_hold.py`,
`t2-serie.json`, `t2-suite.log` y el banco `results.noindex/t2/` como evidencia local.

La variante retenía lectura y última muestra por sensor dentro del SIL. Usaba
`now_us / 1e6` y el mismo predicado `t - last < period - 1e-9`, con umbral calculado
en Python. Solo invocaba Python al tocar muestra nueva; cambiar callbacks
invalidaba la retención. Los tests comprobaron valores, tiempos exactos, RNG,
periodos por sensor, límites de muestra, tiempo repetido/retroceso, índices
inválidos y reemplazo del callback. Banco: **48/48 idénticas** frente a `4472197`;
suite de la variante: **404 passed, 2 skipped, 1 xfailed**.

| Seed | Repetición | T1 user (s) | T1+T2 user (s) |
|---|---:|---:|---:|
| 5 | 1 | 3.87 | 3.91 |
| 5 | 2 | 3.97 | 3.91 |
| 5 | 3 | 3.87 | 3.86 |
| 13 | 1 | 4.11 | 4.10 |
| 13 | 2 | 4.20 | 4.11 |
| 13 | 3 | 4.15 | 4.12 |

## T3 — descartado

CPU incremental frente a T1: **24.62 → 24.29 s (−1.34%)**.
Solo mejora en 3/6 parejas; dos empatan y una empeora. Aunque la suma supera el
1%, no mejora consistentemente y se retiró todo el cambio. No hay commit de T3.
Se conservan `t3-descartado.patch`, `t3_tests.txt`, `t3-serie.json`, `t3-suite.log`
y `results.noindex/t3/` como evidencia local.

La variante reutilizaba esquinas, aristas, caja del vehículo y geometría del
mundo dentro de cada chequeo de 5 ms. Las consultas sucesivas de `ignore` y el
cálculo de penetración de `ignored_contacts` compartían el mismo cuerpo, sin
cambiar el orden de causas, predicados ni la reconstrucción de World tras derribar
una señal. Las pruebas de referencia cubrieron múltiples listas de `ignore`,
campos mutables y eliminación sucesiva de señales. Banco: **48/48 idénticas**
frente a `4472197`; suite de la variante: **404 passed, 2 skipped, 1 xfailed**.

| Seed | Repetición | T1 user (s) | T1+T3 user (s) |
|---|---:|---:|---:|
| 5 | 1 | 3.94 | 3.83 |
| 5 | 2 | 3.86 | 3.86 |
| 5 | 3 | 3.93 | 4.02 |
| 13 | 1 | 4.31 | 4.14 |
| 13 | 2 | 4.28 | 4.14 |
| 13 | 3 | 4.30 | 4.30 |

## Confirmación final frente a la base

Tras descartar T2 y T3, el código retenido coincide con T1. Una segunda serie
independiente contra `4472197` confirma **26.69 → 24.16 s user
(−9.48%)**, con mejora en las seis parejas y STOP/trace idénticos.

| Seed | Repetición | Base user (s) | Final user (s) |
|---|---:|---:|---:|
| 5 | 1 | 4.30 | 3.91 |
| 5 | 2 | 4.33 | 3.89 |
| 5 | 3 | 4.34 | 3.88 |
| 13 | 1 | 4.60 | 4.15 |
| 13 | 2 | 4.57 | 4.14 |
| 13 | 3 | 4.55 | 4.19 |

## Protocolo y artefactos

Artefactos ignorados por Git en `src/RASPI/cam/runs/r5-sil.noindex/`.
`base/` se extrajo mediante `git archive 4472197 src | tar -x -C <base>`.
`bench.py` ejecuta `eval.sh` con `COSTO=0`, `FOX_PY` apuntando al venv del repo
principal y `LAT_OUT=.../results.noindex` para el banco definitivo (`base-final` / `final`).
Los bancos y series intermedios usaron `LAT_OUT=.../r5-sil.noindex/results`; sus
artefactos se trasladaron después a `results.noindex/` sin alterar su contenido.
Cada banco usa un tag distinto. La serie ejecuta `uno.sh` con `/usr/bin/time -p`,
seeds 5 y 13, ruido 1, tres repeticiones, alternando referencia/nuevo y sin
concurrencia de bancos ni tests. Se conserva stderr original en `*.time` y las
seis parejas en JSON. También se compara cada pareja cronometrada.

La comparación conserva el prefijo STOP y todos los campos excepto
`mean_pi_frame_ms`, `render_s`, `pi_s`, `other_s`, `wall_time_s`, `wall_per_sim_s`.
El JSON normalizado conserva signos de cero y tipos numéricos; `trace.csv` se
compara celda por celda como texto, excluyendo solo `pi_frame_ms`.
Los 0/48 éxitos del resumen del banco son colisiones ya presentes en la base:
se exige reproducirlas exactamente, incluidos tiempos simulados y contactos.

Suite ejecutada desde `src/RASPI/cam`:

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH=. \
/Users/jesse/Documents/GitHub/FoxRobotics/.venv/bin/python -m pytest -q \
pure_pursuit/twin/tests pure_pursuit/native/test_route_dp.py
```

Verificación definitiva tras retirar T2/T3: `base-final` frente a `final`,
**48/48 idénticas**. Suite final en `final-suite.log`:
**402 passed, 2 skipped, 1 xfailed in 88.92s**. Las ocho pruebas adicionales
corresponden a las variantes de historia de T1. Las suites y bancos pueden
correr en paralelo para verificar equivalencia; ninguna serie de CPU se ejecutó
con ellos activos. La copia `t1-ref/` corresponde al código del commit T1 y se
calentó con `t1-ref-warm` antes de medir T2/T3, excluyendo su compilación.

Reproducción desde la raíz del worktree (usar tags nuevos para no reutilizar
resultados):

```sh
P=/Users/jesse/Documents/GitHub/FoxRobotics/.venv/bin/python
B=src/RASPI/cam/runs/r5-sil.noindex/bench.py
"$P" "$B" batch "$PWD/src/RASPI/cam/runs/r5-sil.noindex/base" base-repetida
"$P" "$B" batch "$PWD" final-repetida
"$P" "$B" compare base-repetida final-repetida
# Esperar a que no haya bancos ni tests activos antes de medir:
"$P" "$B" series "$PWD/src/RASPI/cam/runs/r5-sil.noindex/base" "$PWD" serie-repetida
```

## Perfil final

Seed 5, ruido 1, mismo `uno.sh`, `COSTO=0` y primer costo 0; se ejecutó con
`python_profile.sh` como `FOX_PY` después de terminar toda otra verificación.
Artefactos: `final.prof`, `final-profile.txt` y `results.noindex/final-profile/`.
STOP/trace de la carrera perfilada también coinciden con la base.
Tiempo total de cProfile: **5,698 s**. Sus tiempos acumulados no se suman entre
sí ni se usan como sustituto de las mediciones `user` en serie.

| Función | Llamadas | Acumulado (s) | % del perfil |
|---|---:|---:|---:|
| `sync_native` | 21.407 | 0,005731 | 0,10% |
| `drain_history` | 0 | 0 | 0% |
| `_pose_at` (incluye búsqueda nativa) | 1.486 | 0,003361 | 0,06% |
| `ToF.read_mm` | 85.624 | 0,499907 | 8,77% |
| `check_collision` | 6.981 | 0,263337 | 4,62% |
| `world.collision` (incluido en el anterior) | 13.962 | 0,240290 | 4,22% |

El trabajo remanente de T2/T3 sigue superando el 3% acumulado, pero las variantes
ensayadas no satisfacen el criterio de mejora de CPU. Se mantienen descartadas.
No se añadieron puntos al checklist cerrado.

Se conserva un único commit de implementación, correspondiente a T1, con el
informe completo. No se modificaron el `.ino`, los parámetros ni la lógica de
conducción. Sin push ni merge.
