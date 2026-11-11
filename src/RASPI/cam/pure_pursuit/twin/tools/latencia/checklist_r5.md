# Checklist de optimización R5 (completa)

Perfil: carrera seed 5 en `4472197`, cProfile, 6,25 s; sin perfil son 4,3 s user.
El % es la parte de la carrera perfilada que consume cada punto. La ganancia real
será menor que ese %.

Reglas:
- Cada punto se cierra como **hecho**, si queda idéntico (48/48), pasa la suite y
  mejora en las 6 parejas en serie, o como **descartado**, si sale <1% o no es
  idéntico.
- La lista no crece. Al final se perfila de nuevo y solo se abre otra ronda si
  aparece un punto de **≥3%**.

## Twin (solo simulador; acelera el optimizador)
- [x] T1 (Codex) `sync_native` / `drain_history`: la historia se vacía en cada `loop` (21k) creando arreglos ctypes y tuplas. 5,8% → **hecho** (`24c4b07`): historia SIL sin copias, −9,9% CPU, mejora en 6/6.
- [x] T2 (Codex) ToF: 85k callbacks C→Python, la mayoría devuelven la lectura retenida. Mover la retención al SIL. 9% → **descartado**: −0,66%, mejora en 5/6.
- [x] T3 (Codex) chequeo de colisión cada 5 ms (`collision`, 14k llamadas en Python). 4,3% → **descartado**: −1,34%, mejora en 3/6.
- [x] T4 (Claude) render: `_fill_face` de quads parcialmente detrás del plano cercano. 2,4% → **hecho**: `_clip_near` con floats de Python; imagen idéntica en 1200 cuadros; render 0,466→0,441 ms (~0,8% de la carrera, bajo el umbral pero se queda: 4 líneas, idéntico).

## Pi (código del carro; falta medir en la Pi)
- [x] P1 (Claude) `cvtColor` BGR→HSV antes de `hsv_masks`: fusionarlo en la LUT nativa. 4,2% → **descartado**: el HSV se usa después (`process_color` mediana de S y chequeo de naranja antes del BEV en runtime_nuevo:1580); fusionarlo no evita la conversión.
- [x] P2 (Claude) `corner_lines.detect_lines`: 0,9 ms por llamada. 3,8% → **descartado**: repartido en `_components` 1,4%, `_curve_from` 1,0%, `_fit_line_near` 1,2%, todo OpenCV/numpy; ninguno ≥1,5%.
- [x] P3 (Claude) actualización del mapa digital tras ack (`_update_map_after_ack`/`up`). 4-7% → **descartado**: repartido en `_build_line` (`_put` 0,7%, `_cans_on` 0,9%, `_ease` 0,4%), `track_map.update` 1,1%, `parse_ack` 0,6%; ninguno ≥1,5%.
- [x] P4 (Claude) `localizer.update` (raycast). 3,2% → **descartado**: `raycast` ya vectorizado, 1,4%; el resto es overhead numpy de arreglos de ~10 elementos.

## Fuera de la lista (<1,5%)
gyro callbacks 1,2%, prints a pi.log 1,1%, importaciones 0,13 s, `hsv_masks` (ya nativo).

## Resultado (en serie, seeds 5 y 13, 3 repeticiones; `/usr/bin/time` user)
| Versión | s5 | s13 |
| --- | ---: | ---: |
| `32c6b77` (inicio del día) | 5,13 | 5,36 |
| `4472197` (R4: ToF + render + SIL) | 4,40 | 4,61 |
| `0acbca1` (R5: T1 + T4) | 4,00 | 4,14 |

R5 baja ~10% respecto a R4; en total, ~22% respecto a `32c6b77`. 48/48 carreras
idénticas. Suite: 402 passed, 2 skipped, 1 xfailed.

## Perfil final y regla del ≥3%
Puntos ≥3% en el perfil final: `render_camera` 7,9% (el piso ya es kernel C),
`cvtColor` 5,8% (P1, descartado), `hsv_masks` 5,2% (ya nativo), `read_mm` 3,3%
(T2, descartado), `fw.loop` 3,3% (el propio firmware en C). Ninguno es un punto nuevo
sin evaluar, así que **no se abre otra ronda**. Para bajar más haría falta un cambio
estructural, como correr varias carreras por proceso en el optimizador, no más
microoptimización.
