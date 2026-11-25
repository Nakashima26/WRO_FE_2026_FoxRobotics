# pk2 · detección de las maderas del cajón

Base 0ee69022. La reproducción usa la pose y los ACK registrados en los JSONL de la Pi para las corridas 257, 258, 260, 261, 263, 264 y 265. Las instantáneas JSONL, aproximadamente a 10 Hz, pueden saltarse una madera que el ESP sí leyó. Los TSV a tasa completa permiten situar los flancos de 264 y 265.

## Por qué decía passes=0

Reproduje ParkRefine de 0ee69022 con pose fijada, ACK S/C/R, tc 3–11 y odometría nueva. En las siete corridas, estimate() devolvió None antes de llegar a la regla de 70 mm contra el prior.

| Corrida | ACK únicos | h/rumbo/w | ToF 20–470 mm | Profundidad 160–240 mm | Retornos por pasada, w en mm |
| --- | ---: | ---: | ---: | ---: | --- |
| 257 CCW | 214 | 16 | 10 | 0 | — |
| 258 CCW | 225 | 18 | 12 | 1 | 1328 |
| 260 CCW | 210 | 17 | 12 | 0 | — |
| 261 CCW | 217 | 0 | 0 | 0 | — |
| 263 CCW | 220 | 1 | 0 | 0 | — |
| 264 CW | 166 | 21 | 12 | 3 | 995 y 1356, luego 958 |
| 265 CCW | 203 | 13 | 3 | 1 | 1343 |

- En 261 y 263 el carro pasó por el interior. Hay retornos de madera con h por encima de 700 mm y ToF lateral de 486–613 mm. Otros retornos proyectan al costado de la madera fuera de ±40 mm de la punta. Los ecos de pared dominan entre los retornos aceptados por rango.
- En 264 los dos puntos de la primera pasada están separados 361 mm. La separación nominal de centros es 284 mm (264 + 20), con tolerancia de 35 mm. Con el cajón real de 291 mm la separación teórica pasa a 311 mm y todavía deja 50 mm de residuo. La segunda pasada aporta una sola madera.
- En 265 sobrevivió un solo retorno hasta tc11. El corte por tc y estado perdió ecos cortos de la recta final. El de 14 mm también queda por debajo del mínimo original.
- La banda de 70 mm habría ocultado una corrección pequeña si hubiera habido pareja. El límite interno de ±245 mm y los límites nominales de w también restringían correcciones extremas. Los siete ceros se producen antes de aplicar esas restricciones al resultado.

## Resultado offline

| Corrida | pka nominal | w observado en tc12 | pka geométrico en tc12 | Pasadas compatibles |
| --- | ---: | ---: | ---: | ---: |
| 257 CCW | +500 | 1089 | +411 | 1 |
| 258 CCW | +500 | 1094 | +406 | 1 |
| 260 CCW | +500 | 1044 | +456 | 1 |
| 261 CCW | +500 | 1021 | +479 | 3 |
| 263 CCW | +500 | 1093 | +407 | 3 |
| 264 CW | −196 | 985 | −263 | 1, con corroboración de la primera madera en otra pasada |
| 265 CCW | +500 | 1052 | +448 | 2 |

`pal` mide el eje trasero y `pka` la cara lejana. En 264 el primer eco tL=82 a pal=−576 precedió el arranque de pkf0 en −500. Siguieron tL inválido en −539 y retornos de 149/179 mm en −386/−343. El punto medio entre −576 y −386 queda 285 mm antes del prior. Proyectar el segundo eco usando el montaje longitudinal de 110,6 mm sitúa la cara cerca de −263. En 265, tR=14 en pal=346 aparece 154 mm antes del prior comparando lecturas crudas. Al proyectarlo desde el sensor, la cara estimada queda en +448. El ESP ya había anclado en pkd=346 y pasó a pkf1 y 40. La captura termina durante el plan.

En 265 el mapa confirmó W/T2 como verde durante toda la captura. El plan #14 lo lista rojo porque ROUTE_FINAL_DROP_FAR=200 fuerza el lado de paso de las señales situadas más allá del cajón al preparar la recta final. En CCW el color que el planificador usa para ese lado es rojo. Hay evidencia del cambio de lado en el plan y el mapa sigue reportando verde.

Reproducción desde la raíz del worktree, con el Python de la instalación de carrera.

```sh
PYTHONPATH=src/RASPI/cam /Users/jesse/Documents/GitHub/FoxRobotics/.venv/bin/python codex_tareas/pk2_replay.py /Users/jesse/fox-coord.noindex/real1125/tele/orillas*_tele.jsonl
```

## Implementación

La Pi conserva la corrección conservadora durante la carrera y recoge el par observado con un rango de proyección mayor. El refinamiento acepta una pareja cercana o parejas corroboradas en más de una pasada. Proyecta ambas maderas para estimar la cara lejana. La pasada de tc12 puede completar la observación en pkf0 con la pose extrapolada a la odometría del ACK. Cuando el ESP indica una madera temprana, la Pi le envía la corrección respaldada por un eco del paso final. El volante sigue bajo control del ESP durante pkf0.

El ESP guarda la primera bajada exterior después de cerrar la esquina 12 y conserva su odometría durante el armado de pkf0. Si la entrada por prior sigue esperando tras avanzar 105 mm desde el eco, abre el escaneo con ese dato. Una bajada durante el armado también invalida la confianza en el prior si aparece más de 120 mm antes de la primera madera esperada. Una vez detectada la discrepancia, el escaneo requiere corroboración para usar el prior para acelerar o cerrar la ventana de detección. Las dos variantes del firmware llevan la misma modificación y se extraen con CRLF.

## Banco

Configuración pareada GROK_FF=4.28, GROK_FD=5.8, TWIN_COST_FIJO=1, TWIN_TOF_SHORT_K=0.5 y calibraciones de cámara y motor de cal1. Base n1–6 × s1–48, réplica n7–8 × s1–48. El estrés adelanta +200 mm solo el pka nominal de la Pi en s20 CW y s21 CCW, con n1 y n2. La geometría física y las señales son idénticas por par.

Grok recibió los trabajos y los puso en cola bajo twin.lock. La aceptación de 288 + 96 queda pendiente de los archivos de resultados. La prueba local n1 × s1–48 de la variante que aplicaba las nuevas ventanas a todos los escaneos dio 46/48 vs 48/48 en base, medias 36,959326 s y 36,370292 s. Esa versión quedó descartada.
