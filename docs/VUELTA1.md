# Vuelta 1 de obstáculos: menos frenos (twin)

Estado: twin, rama `claude/vuelta1-6f4816`, **nada probado en el carro**. Todo apagado por defecto.
Modelo de motor del twin lineal y optimista: cifras provisionales hasta calibrar (`docs/PLAN_TIEMPO.md`, Fase 0).

## Dónde se iba el tiempo (seed 1, ruido 1, base)

- Vuelta 1 = 31.1 s:
  - ~18 s a PWM 100 (freno `ROUTE_SLOW_PWM`).
  - ~3.5 s detenido (parada cuando la cámara ve un asiento sin resolver a <= 750 mm).
  - ~4.4 s en la salida del cajón (estado I del firmware, `INICIO_PWM=95`).
- Con todo a 155, la vuelta 1 baja a ~23 s. Lo que queda es la salida (~4.4 s) y las paradas (~3 s).
- `ROUTE_SLOW_PWM` servía para 4 frenos distintos:
  1. asiento sin ver cerca;
  2. error lateral (CTE) > 40 mm;
  3. mientras sube la ruta al ESP;
  4. antes de fijar el avance del cajón.

## Cambio: topes separados (commit `dadd13b`)

`config.py`: `ROUTE_UNSEEN_PWM`, `ROUTE_CTE_PWM`, `ROUTE_UPLOAD_PWM`. `None` = `ROUTE_SLOW_PWM` (comportamiento idéntico).
"Sin fijar el avance" sigue con `ROUTE_SLOW_PWM`.

## Resultado recomendado: U155

```json
{"pi":{"ROUTE_LAP1_PWM":155,"ROUTE_UNSEEN_PWM":155}}
```

CTE, subida y sin fijar siguen en 100.

Banco: 42 seeds (1-41 y 3170839) x 5 ruidos, preset `ruta`, `FOX_PARK_POS=random`, `--solo-cajon`.
Código congelado en worktree aparte. Base y candidato con el mismo ruido.
Limpia = `race_finished` sin `collisions` ni `sign_contacts`.

## Tabla de resultados

210 pares cada uno salvo nota.

| Candidato | Δ vuelta 1 med | Δ total med | Vuelta 1 med (base → cand) | Fallas base/cand | Solo-base / solo-cand | Regla |
|---|---|---|---|---|---|---|
| LAP1=155 (ruido 1, 42 pares) | -0.90 | -0.85 | 27.58 → 26.66 | 2/2 | 0/0 | ahorro bajo (ya conocido: -0.95) |
| **U155**: LAP1=155 + UNSEEN=155 | -4.48 | -4.80 | 27.67 → 23.27 | 14/16 | 1/3 | **CUMPLE** (3 <= 1+2; ninguna seed nueva falla en >= 2 ruidos) |
| A155: LAP1=155 + SLOW=155 (todos los frenos) | -6.07 | -6.48 | 27.64 → 22.04 | 14/17 | 7/10 | NO (10 > 7+2) |
| UC155: U155 + CTE=155 + UPLOAD=155 (seeds 4, 16, 19, 23, 32, 36 x 5 ruidos, 30 pares) | -6.53 | -7.25 | — | 6/9 | 4/7 | NO |
| A155 + ROI solo vacíos (vs A155) | -0.39 | -0.45 | 22.06 → 21.64 | 17/19 | 4/6 | NO (ahorro < 0.5; s36 nueva en 2 ruidos) |
| A155 + ROI con color (ruidos 1-2, 86 pares, vs A155) | -0.18 | -0.65 | 22.09 → 21.71 | 4/8 | 3/7 | NO, descartado |

### Lectura de fallas

- **Solo U155**: s16_n1 y s16_n5, choque con roja N/T3 a ~9 s (misma causa que la base en s16_n3);
  s5_n5 contra la isla a 62.1 s, en TERMINANDO tras completar las 3 vueltas (riesgo ya conocido, `GROK_HANDOFF.md` §4).
  s16 es la seed que Grok identificó: la ruta cambia de lado al confirmar rojo y el localizador descarta el ToF frontal.
- **Solo A155**:
  - s16 y s23 a ~9 s;
  - s32 contra W/T2 a ~7.8 s;
  - s19 y s36 en la vuelta 2 (SLOW también sube los frenos CTE/subida en las vueltas 2-3);
  - s4_n4 contra la isla en TERMINANDO, después de la meta.
- UC155 muestra que subir CTE y subida a 155 es lo que choca. Separar los topes es lo que hace aceptable U155.
- Quitar la parada (`ROUTE_STOP_MM=0`) choca en s16: la parada protege cuando una roja se confirma cerca.

## Probado y descartado: "ver lejos usando el mapa" (`ROUTE_SEAT_ROI`, `seat_roi.py`)

Idea: proyectar cada asiento conocido (`SEATS` + `section_to_world`) con la pose del localizador y la homografía
del BEV (`H_inv`) a la imagen de perspectiva, y clasificar rojo/verde/vacío ahí, en lugar de esperar a
`ROUTE_VIEW_MAX_MM=550`.

- **Por asiento** (ventana de 50 mm): con 20 mm de error lateral o 3-6° de rumbo (girando, la pose llega con retraso)
  se pierde la señal y dice vacío.
- **Por columna** (los dos asientos de mismo w nunca tienen señal a la vez; ventana ampliada a 80 mm): nunca confunde rojo con verde.
- **Límite físico**: cámara a 97 mm, paredes de la isla de 100 mm. La isla tapa la recta siguiente entera;
  solo se ve lejos dentro de la recta en que se va.
- Medido (seed 1; gates |yaw| <= 1°/frame, |rumbo| <= 40°, ventana entera en imagen, occ=0):

  | Distancia | Peligrosos / frames |
  |---|---|
  | 0-500 mm | 1 de 435 |
  | 500-1000 mm | 0 de 1053 |
  | 1000-1500 mm | 1 de 293 |
  | 1500-2000 mm | 2 de 45 |

  Son frames sueltos; la decisión exige 3 seguidos. No hay datos útiles > 2 m.
- **Usar el color sin frenar**: la clasificación acertaba, pero sin saber la fila (T2 vs T4) la ruta se corregía tarde
  a 155 y chocaba (s20, s36).
- **Usar solo los vacíos**: -0.4 s, no alcanza la regla.
- Con SLOW/UNSEEN a 155, el freno por no visto ya casi no cuesta. Lo que queda son las paradas en asientos con señal,
  que necesitan la fila.
- **ADVERTENCIA**: la detección a distancia está medida solo en el render sintético del twin (sin ruido de cámara,
  sin blur, colores planos). Hay que validarla en el carro antes de usarla para nada.
- Nota: `Vision.process_frame` espeja el frame (`cv2.flip`); la ROI usa el frame espejado sin anotaciones.

### Banderas (todas apagadas)

`ROUTE_SEAT_ROI`, `SEAT_ROI_COLOR`, `SEAT_ROI_LOG`, `SEAT_ROI_DUMP_DIR`, `SEAT_ROI_USE_MAX_MM=1800`,
`SEAT_ROI_FRAMES=3`, `SEAT_ROI_MAX_YAW=1.0`, `SEAT_ROI_MAX_BEARING=40`, `SEAT_COL_MARGIN_MM=80`, `SEAT_COL_MIN_FRAC=0.3`.

Medición: `twin/tools/seat_roi_eval.py --col <dirs>`.

## Siguiente (no hecho)

1. **Fila por ROI**: dentro de la ventana de columna, el centroide horizontal del color separa T2/T4
   (200 mm, ~11° a 1 m). Con color + fila a 1-1.5 m se quitarían las paradas en asientos con señal (~3 s en seed 1).
2. **Salida del cajón**: `INICIO_PWM=95` (`PurePursuit.ino:762`) se puede barrer por
   `{"fw":{"INICIO_PWM":"..."}}` sin código. ~4.4 s de la vuelta 1 están ahí (salida por dentro, 90°, 620 mm).
3. **CTE/subida a 155 solo en la vuelta 1**: los choques de A155 en la vuelta 2 venían de ahí.
4. **Combinar con el perfil por curvatura** (preset `ruta_perfil`) y medir juntos.
5. **En el carro**: probar U155 con velocidad escalonada después de la calibración de motor (Fase 0).
