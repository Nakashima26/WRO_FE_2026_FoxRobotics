# Segunda ronda: simulador completo

Base: e5d0b17dfb28af80fa717f8126b6068a41986415. Rama: opt-twin-r2.
Medido el 10 de noviembre de 2026 con el Python de FoxRobotics.

## Perfil completo

Se ejecutó costo.py bajo cProfile desde src/RASPI/cam, semilla 1,
ruido 1, con las variables de uno.sh: TWIN_GEAR=1.25,
FOX_PARK_POS=random, TWIN_NOISE_SEED=1, OMP_NUM_THREADS=1,
OPENBLAS_NUM_THREADS=1, MKL_NUM_THREADS=1, PYTHONUTF8=1,
TWIN_FW_PARAMS=serial2_us_per_char=21.7,serial_us_per_char=10.85,
TWIN_PI_UART_US_PER_BYTE=21.7, TWIN_COST_FRAME=0, TWIN_COST_DP=0,
PYTHONPATH=.; stdin cerrado. Se perfiló hasta la terminación normal de
esa carrera (colisión), sin recortar pasos ni cuadros: 190 cuadros,
2641 llamadas al SIL. Tiempo total de cProfile: 2023,8 ms.

Reparto aproximado, sin sumar dos veces los callbacks del SIL:

| Bloque | ms base | % del perfil completo |
|---|---:|---:|
| Pi: process_frame | 804,9 | 39,8 |
| Render de cámara del twin | 174,4 | 8,6 |
| Física: advance, incluye colisión y encoder | 229,7 | 11,3 |
| Sensores: callbacks ToF, gyro y lectura encoder | 101,9 | 5,0 |
| SIL restante: loop menos física y callbacks de sensores | 50,6 | 2,5 |

El resto incluye arranque/importaciones, planificación de calentamiento,
carga de biblioteca SIL y el bucle/I/O exterior. El loop SIL inclusivo
cuesta 382,2 ms; no debe sumarse entero a física y sensores.
Dentro de física, colisión cuesta 36,3 ms. Dentro de sensores, read_mm
cuesta 80,2 ms. I/O: pop_debug 12,7 ms, pop_serial2_lines 8,3 ms,
_log_trace 4,2 ms y writerows de CSV 1,6 ms (incluyen sus subllamadas).
Estos últimos costes no justifican alterar el formato ni la selección de trazas.

## Cambios

- Kernel exclusivo del render twin: escribe piso/isla directamente en BGR
  con el mismo orden float32, sin FMA. Evita IDs intermedios, merge y LUT.
  Conserva la ruta NumPy si no hay biblioteca nativa.
- Puente SIL: conserva buffers de serie por instancia, devolviendo copias
  como antes; conecta advance directamente al CFUNCTYPE, que ya convierte
  uint64 en int de Python.
- Sin cambios en parámetros, lógica de vueltas, visión/control de la Pi ni
  firmware del ESP. La adición a native/src/twin_floor.c solo la usa el twin.

## Mediciones y equivalencia

48 carreras mediante race48.sh con P=2, FF=FD=0, ruido 1–2, semillas 1–24.
El baseline válido se ejecutó completo con árbol limpio antes de reaplicar
los cambios; se descartó una primera tanda que coincidió con las ediciones.

| Medida | Base | Nueva |
|---|---:|---:|
| CPU user, 48 carreras | 215,46 s | 209,95 s |
| CPU sys, 48 carreras | 5,07 s | 5,06 s |
| Tiempo real, 48 carreras | 118,02 s | 115,09 s |

Ahorro user: 5,51 s (2,56%). Son tandas únicas bajo carga compartida;
no se extrapola un tiempo absoluto a P=8.

Microbenchmark de render: mismas 200 poses aleatorias (rng 11), campo 11,
OpenCV con un hilo; comprobación array_equal entre ambas versiones para
cada pose, seguida de dos tandas alternadas medidas con thread_time.
Base: 0,7391 y 0,7309 ms/llamada. Nueva: 0,5470 y 0,5290 ms/llamada.
Promedios: 0,7350 → 0,5380 ms (−26,8%).

Segundo cProfile de la misma carrera: render 174,4 → 125,0 ms,
SIL inclusivo 382,2 → 338,1 ms, pop_debug 12,7 → 8,3 ms,
pop_serial2_lines 8,3 → 3,0 ms. Los tiempos de bloques sin cambios también
fluctúan; la referencia de ahorro global es el CPU user de las 48 carreras.

Verificación obligatoria:

- PYTHONPATH=. /Users/jesse/Documents/GitHub/FoxRobotics/.venv/bin/python
  -m pytest -q pure_pursuit/twin/tests: 242 passed, 2 skipped, 1 xfailed.
- cmp.py base new: 48/48 idénticas []; compara STOP y todos los valores
  de trace.csv salvo tiempos de cómputo, según el comparador proporcionado.
- git diff --check sin errores. Salidas temporales .noindex eliminadas.
