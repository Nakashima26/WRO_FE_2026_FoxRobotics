# DP de ruta: segunda ronda

Base: `e5d0b17dfb28af80fa717f8126b6068a41986415`, rama `opt-dp-r2`.
Medición local: Apple M5 Pro (arm64), Apple clang 21.0.0, Python 3.14.8.
No se dispone de una Pi 5: estos tiempos corresponden exclusivamente al Mac.

`fox_route_dp_sparse_t` divide las filas b entre pthreads. Crea el equipo una vez
por llamada y lo reutiliza entre estaciones, con una barrera de publicación por
estación. Cada celda conserva el orden ascendente de a, las operaciones de coma
flotante y el primer mínimo, incluidos NaN y costes saturados. No se introduce
poda aritmética adicional. El equipo y sus contadores son locales a la llamada;
si falla la creación de un hilo, se usa el equipo que logró crearse.

Los dos búferes de costes alternan como entrada/salida transpuestas, eliminando
la copia/transposición intermedia de cada estación. Al salir se restauran los
formatos originales de Vc, Vtmp y Vt. No cambia la ABI ni se reserva memoria en el
heap. En Windows se conserva la ejecución serial.

`FOX_DP_THREADS=1..4` selecciona el número total de hilos, incluido el llamante.
El valor por defecto o inválido es 1, por consumo de CPU y ausencia de medición
en la Pi. Se limita también a m; sin estaciones intermedias no crea hilos.
La espera de barrera usa atómicos y cede CPU periódicamente; los modos paralelos
consumen más CPU, como refleja la tabla.

La compilación usa `-O3`, `-fno-fast-math`, `-ffp-contract=off` y `-pthread` en
POSIX. `FOX_NATIVE_CPU=native` activa `-mcpu=native` en ARM (Pi incluida) y
`-march=native` en x86. Es opcional, para compilaciones locales; las opciones
forman parte del hash del binario y no se debe trasladar ese binario a otra CPU.

## Medición pareada del plan completo

Cuatro casos golden, cachés calientes, 100 planes por caso y lote, 9 lotes.
Se rota el orden de base/1/2/4 para repartir carga externa. Cada cifra es la media
de las medianas por caso. CPU es tiempo de proceso, suma de todos sus hilos.
La base y los cambios se cargan en el mismo proceso. No se activa CPU=native.

| Versión | ms/plan | CPU ms/plan |
| --- | ---: | ---: |
| Base, 1 hilo | 2.967512 | 2.966163 |
| Nueva, 1 hilo (defecto) | 2.839512 | 2.837475 |
| Nueva, 2 hilos | 2.178107 | 3.359958 |
| Nueva, 4 hilos | 1.727088 | 3.901253 |

Frente a la base, 1 hilo reduce tiempo y CPU un 4,3%; 2 hilos reduce tiempo un
26,6% y aumenta CPU un 13,3%; 4 hilos reduce tiempo un 41,8% y aumenta CPU un
31,5%. El tiempo absoluto depende de la carga del equipo; la primera medición,
con las carreras ejecutándose, rondaba 4,2–4,3 ms. La comparación de la tabla es
pareada, con el mismo entorno para todas las variantes.

Desde `src/RASPI/cam`, conservando primero una biblioteca compilada en la base:

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH=. \
/Users/jesse/Documents/GitHub/FoxRobotics/.venv/bin/python \
-m pure_pursuit.native.bench_route_dp \
--baseline pure_pursuit/native/_build/libfox_native_c247835d0c2ce19c.dylib \
--repeats 100 --batches 9
```

El benchmark comprueba también las rutas y reportes contra la base en cada modo.
En la Pi se puede repetir con `FOX_NATIVE_CPU=native`, compilando la biblioteca
base en la propia Pi y pasando su ruta. Mantener `OMP_NUM_THREADS=1` evita que la
geometría y los kernels alternativos OpenMP usen un equipo distinto.

## Verificación

- Base sin cambios: 48 carreras con `race48.sh`, tag `base`, P=2.
- Cambios: 48 carreras con tag `new`, `FOX_DP_THREADS=4`, P=2.
- `cmp.py base new`: **48/48 idénticas []** (STOP y trazas, excluyendo tiempos).
- Suite obligatoria `pure_pursuit/twin/tests`: **242 passed, 2 skipped, 1 xfailed**.
- `pure_pursuit/native/test_route_dp.py`: **84 passed**. Compara bytes de Vc,
  Vtmp y back con el kernel sparse de referencia; cubre m=1..256, 1/2/4 hilos,
  saturación, empates, NaN, pasos pares/impares, N=2/3 y entorno inválido.
- Con `FOX_NATIVE_CPU=native`, pruebas del kernel y golden de rutas: **87 passed**
  en este Mac. La compilación específica de CPU en la Pi queda sin medir.
- Compilación de `route_dp.c` con `-Wall -Wextra -Werror`: sin avisos.

Los resultados de carreras se guardaron en `/tmp/fox-dpr2.noindex` y se eliminaron
tras la comparación. No se cambian parámetros de carrera ni lógica de vueltas.
