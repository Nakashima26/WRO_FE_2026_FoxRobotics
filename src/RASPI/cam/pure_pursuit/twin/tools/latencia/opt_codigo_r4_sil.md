# CPU del twin: integración directa desde SIL

Base: `911a6b1`. Rama: `opt-r4-sil`. Medición en Mac arm64, 10 de noviembre de 2026.
Se revisó `c2e0546`; esta implementación usa directamente el kernel existente,
mediante su puntero de función, desde el SIL.

## Resultado

CPU `user` en serie: **31,25 → 27,00 s (−13,60%)** sumando las seis parejas.
El nuevo código ganó en las seis comparaciones, con tres repeticiones por semilla.
La equivalencia del banco fue **48/48**, y la de las parejas cronometradas, **6/6**.
Suite: **394 passed, 2 skipped, 1 xfailed**, frente a 390 pruebas aprobadas en la base.

| Semilla | Repetición | Base, user (s) | Nuevo, user (s) |
|---|---:|---:|---:|
| 5 | 1 | 5,05 | 4,34 |
| 5 | 2 | 5,02 | 4,32 |
| 5 | 3 | 5,06 | 4,36 |
| 13 | 1 | 5,37 | 4,65 |
| 13 | 2 | 5,44 | 4,65 |
| 13 | 3 | 5,31 | 4,68 |

Media seed 5: **5,043 → 4,340 s (−13,95%)**.
Media seed 13: **5,373 → 4,660 s (−13,28%)**.
La mejora de cada pareja está entre el 11,86% y el 14,52%.
No se usa el tiempo del lote paralelo para afirmar una mejora.

## Implementación

- `sil_vehicle.cpp` ejecuta `fox_twin_vehicle` por su puntero C sobre
  `NativeVehicle._state` y `_p`, propiedad de Python. No hay copia del vehículo.
- Integra el encoder con las mismas operaciones y orden de `Encoder.update`,
  incluido `pow` con exponente volátil, truncamiento y acumulador fraccional.
  Las lecturas del encoder y de la velocidad de giro usan sus slots compartidos.
- Conserva la resta sucesiva de `remaining`, los subpasos de hasta 1 ms y el
  mismo tiempo constante dentro de cada avance. El callback de colisión ocurre
  tras el primer subpaso que satisface `t - last_coll_check >= 0.005`.
- Guarda `(t,x,y,h)` en un anillo C de 20.000 muestras. Al regresar de `setup()`
  o `loop()`, sincroniza los escalares y vacía el anillo al `deque` Python antes
  de que el simulador lea la historia, interpole la pose o registre una semilla.
  Sonar, ToF y BNO leen directamente el estado actualizado; el gyro usa la
  última velocidad de giro nativa. Se conserva el orden de las lecturas y RNG.
- `FirmwareSIL` retiene los propietarios de los punteros y del callback durante
  toda su vida. Cada instancia mantiene su propio estado y anillo.
- Si no se selecciona `NativeVehicle` (biblioteca ausente o
  `FOX_TWIN_PHYSICS=python`), se mantiene el callback Python original.
- El integrador nuevo se compila por separado con `-O3 -ffp-contract=off`.
  El kernel físico ya usa `-ffp-contract=off`. El firmware conserva sus opciones
  originales de compilación. Aplicar esa opción también al firmware produjo
  divergencias en una versión intermedia, descartada antes de medir la final.

No se modificaron el `.ino` ni las decisiones de conducción.

## Verificación

Desde `src/RASPI/cam`:

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH=. \
/Users/jesse/Documents/GitHub/FoxRobotics/.venv/bin/python -m pytest -q \
  pure_pursuit/twin/tests pure_pursuit/native/test_route_dp.py
```

Resultado final: `394 passed, 2 skipped, 1 xfailed in 76.97s`.
El test nuevo `test_sil_native_advance_exact`, con cuatro variantes, compara bytes
IEEE-754 de todo el arreglo de estado, historia, puntos de chequeo de colisión,
encoder, último desplazamiento, velocidad de giro y estado observado por sensores.
Ejecuta dos instancias del firmware: una con el avance Python de referencia y otra
con el avance C, incluyendo los avances de varios subpasos durante el arranque.
Las variantes cubren parámetros normales, holgura/retardo, motor inverso y frenado/
límites laterales. Los tests existentes verifican además el kernel contra `Vehicle`.

La base se extrajo con `git archive 911a6b1 src | tar -x -C <directorio>.noindex`
desde la raíz del worktree, incluyendo `src` completo. Para cada implementación
se ejecutó desde su correspondiente `src/RASPI/cam`:

```bash
COSTO=0 FOX_PY=/Users/jesse/Documents/GitHub/FoxRobotics/.venv/bin/python \
LAT_OUT=<salida>.noindex bash pure_pursuit/twin/tools/latencia/eval.sh <tag> '{}' 4
```

Se compararon todos los campos de `STOP`, excepto `mean_pi_frame_ms`, `render_s`,
`pi_s`, `other_s`, `wall_time_s` y `wall_per_sim_s`. Se conservaron los tiempos
simulados, las vueltas y los instantes de colisión en la comparación.
Se compararon todas las celdas de `trace.csv`, excepto `pi_frame_ms`.
**Las 48 parejas coinciden exactamente.** El resumen del banco da `0/48 ok`
en ambas implementaciones: las colisiones de la base se reproducen, sin cambios.

Después de terminar el banco y la suite, se ejecutaron sin concurrencia las
semillas 5 y 13, ruido 1, tres veces cada una, alternando base/nuevo. Cada medición
usó una salida distinta para evitar que `uno.sh` reutilizara corridas anteriores:

```bash
FOX_PY=/Users/jesse/Documents/GitHub/FoxRobotics/.venv/bin/python \
LAT_OUT=<salida>.noindex TWIN_COST_FIRST=0 TWIN_PI_EXTRA='{}' \
/usr/bin/time -p bash pure_pursuit/twin/tools/latencia/uno.sh <tag-unico> 0 0 1 <seed>
```

Se tomó exclusivamente `user` de `/usr/bin/time -p`; incluye el proceso Python
y el pequeño wrapper shell. Las seis parejas cronometradas también coinciden en
`STOP` y `trace.csv` con las mismas exclusiones.

## Artefactos locales

En `src/RASPI/cam/runs/` (ignorados por Git):

- `r4-sil-base.noindex/`: copia de referencia de `911a6b1`.
- `r4-sil-results.noindex/base/` y `final/`: los dos bancos de 48 carreras.
- `r4-sil-results.noindex/serie-*-r*/`: las doce carreras medidas en serie.
- `r4-sil-results.noindex/time-*.txt` y `serie.json`: tiempos originales y datos.
- `r4-sil-results.noindex/comparar.py`: comparación reproducible del banco y serie.
- `r4-sil-results.noindex/serie.py`: comandos y orden de las mediciones;
  para repetirlos hay que usar tags nuevos o retirar las salidas anteriores.

Las carpetas `nuevo`, `nuevo2` y `nuevo3` contienen comprobaciones intermedias;
no intervienen en los resultados finales ni en las cifras de CPU anteriores.
