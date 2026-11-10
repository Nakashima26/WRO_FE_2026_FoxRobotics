# Optimización conjunta y siguiente iteración con Codex

Medición del 10 de noviembre de 2026, Mac M5 Pro. Estos resultados son locales;
no se ha medido la Raspberry Pi real.

Base de esta comparación: `e5d0b17`, que **ya contiene las optimizaciones
anteriores**. No son porcentajes acumulados desde el código original.

## Carrera completa: cambios juntos

48 casos por tanda: semillas 1–24 × ruido 1–2, `uno.sh` con FF=FD=0,
cuatro procesos, `FOX_DP_THREADS=1`, OpenCV/BLAS/OpenMP con un hilo.
`TWIN_GEAR=1.25`, estacionamiento aleatorio y UART modelado según `uno.sh`.
Se calentaron las bibliotecas antes de cronometrar; cada carrera sigue siendo
un proceso nuevo, con sus importaciones y sus cachés de planificación vacías.
Las versiones de referencia se extrajeron con `git archive`; las fuentes se
mantuvieron fijas durante toda la comparación.

Dos tandas por versión, orden base→R2→R3 y luego R3→R2→base.
CPU user es la suma del tiempo de usuario de los procesos hijos, medido con
`resource.getrusage(RUSAGE_CHILDREN)`; tiempo real es `perf_counter` del lote.
Las cifras principales son medias de las dos tandas.

| Versión | CPU user (s/48) | Tiempo real (s/48) | Menos CPU vs base |
| --- | ---: | ---: | ---: |
| Base `e5d0b17` | 225.700 | 63.473 | — |
| DP + twin `d6dd439` | 211.287 | 59.455 | 6.39% |
| DP + twin + máscaras nativas | 204.300 | 57.656 | 9.48% |

La pasada nueva de visión ahorra otro **3.31% de CPU** sobre R2. Todos los cambios
juntos reducen el tiempo real **9.17%** sobre la base. Para el optimizador genético,
esto mide el coste de simular los casos; el ahorro del trabajo completo depende
también del paralelismo y del resto del optimizador. Dos tandas permiten observar
consistencia, pero no constituyen un intervalo estadístico de confianza.

Datos individuales (CPU user / CPU sys / tiempo real, segundos):

| Tanda | Base | R2 | R3 |
| --- | --- | --- | --- |
| 1 | 227.085 / 5.829 / 63.986 | 212.471 / 5.306 / 59.874 | 204.809 / 5.093 / 57.791 |
| 2 | 224.314 / 5.338 / 62.960 | 210.103 / 4.958 / 59.036 | 203.792 / 4.982 / 57.521 |

No se suman los porcentajes individuales de DP y render: esta tabla mide los
cambios integrados en la carrera completa. Se mantiene un hilo de DP por defecto;
los modos de 2 y 4 hilos siguen siendo opcionales y consumen más CPU.

## Siguiente iteración: máscaras HSV en una pasada

`vision.hsv_masks` aplica la misma LUT de bits mediante `fox_hsv_masks` y escribe
directamente las máscaras por color. Evita la imagen LUT de tres canales, su
separación y las pasadas adicionales de AND/comparación. El caso habitual de dos
colores evita también el bucle interior por color. No cambia rangos, filtros,
contornos ni detecciones. La tabla se sigue reconstruyendo si cambian los rangos.

Si la biblioteca nativa no está disponible, conserva la implementación OpenCV.
Con `VISION_LUT=False` o más de ocho rangos conserva `cv2.inRange`.
Se aceptan vistas no contiguas copiándolas una vez; las máscaras de un cuadro
no se sobrescriben al procesar otro. El kernel nativo libera el GIL.

Microbenchmark en 99 cuadros de 640×480 del twin: campo 11, poses aleatorias con
rng=11, x/y en ±1250 mm y rumbo 0–360°. Nueve tandas alternando el orden de las
dos implementaciones, mediana de CPU del hilo por cuadro, OpenCV con un hilo.

| Operación | LUT OpenCV anterior | LUT nativa | Menos CPU |
| --- | ---: | ---: | ---: |
| Máscaras | 0.3143 ms | 0.2018 ms | 35.8% |
| `detect_on` completo | 0.5964 ms | 0.4839 ms | 18.9% |

La mejora de `detect_on` afecta al código que corre en la Pi, pero su magnitud en
ese hardware queda por medir. Las cifras no se multiplican por un factor Mac→Pi.

## Verificación

- Se compararon las seis tandas contra la primera base: **48/48 idénticas** en
  cada una, tanto STOP como todas las columnas de `trace.csv`, excluyendo tiempos
  de cómputo. Se usó el mismo criterio de comparación de las rondas anteriores.
- Las pruebas de visión ejecutan tanto el backend nativo como el de OpenCV;
  comparan con `cv2.inRange` en las 256³ combinaciones HSV, incluyendo rangos
  actuales/Pi4/Pi5, colores adicionales, rangos modificados, más de ocho rangos,
  colores vacíos, imágenes no contiguas y cuadros del twin.
- Compilación del nuevo C con `-Wall -Wextra -Werror`: sin avisos.
- Comando de la suite desde `src/RASPI/cam`:
  `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH=. python -m pytest -q pure_pursuit/twin/tests pure_pursuit/native/test_route_dp.py`.
- Resultado de la suite: **340 passed, 2 skipped, 1 xfailed**.
