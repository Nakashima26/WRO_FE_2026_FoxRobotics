# Handoff 2026-11-27 (noche)

## Qué hay en main

`main` = lo que corre en el carro hoy. Es la rama `real-1127-park-sinu` (linaje real-271 / `base-jala-1126`) más un merge `-s ours` del main viejo. De ese main se trajo solo el arreglo del dashboard (006a26bb); lo demás ya estaba portado o se dejó fuera porque falló en el carro (colapso 11-25 = RUTA_WALL_PANIC false).

- **ESP:** PurePursuitCarrera de `a14ced4a`, flasheado 11-27 desde la Mac (`arduino-cli upload -p /dev/cu.usbserial-0001 --fqbn esp32:esp32:esp32 src/ESP32/PurePursuitCarrera`). Los dos .ino van iguales (CRLF).
- **Pi:** `carrera.json` y `bev_calib.npz` son los de la Pi (tilt 22.5 corregido −0.8°, escala 0.95; SURPRISE_RULE, FIX_POSE_MARGIN 500, VISION_GREEN_V_MIN_MED 45). El resto de `src/RASPI` en la Pi = este commit (md5 verificado).
  - La Pi no se actualiza con `git pull`: tiene cambios locales. Se despliega por `scp` de cada archivo y luego `sudo systemctl stop wro-runtime && sleep 4 && sudo systemctl start wro-runtime`.

## Cambios de hoy (11-27)

| Qué | Dónde | Banco / real |
|---|---|---|
| Rojos en esquinas seguidas: la lata de la recta siguiente vista de frente vota débil (`DIGITAL_MAP_NEXT_ROW_WEAK_DEG=60`) | Pi digital_map.py | 96: 6 vs 11 fallas, +0.39 s; acomodo real 4/4 |
| Asiento de señal especial lejos del asiento → se pega al asiento confirmado ≤350 mm | Pi surprise.py | test unitario |
| `ROUTE_PARK_NEAR_PWM=140`: tope cerca del cajón al salir de la última esquina (402/403 rozaban la 1ª madera) | Pi config | falta validar en real |
| Zigzag adelante del parking: manda el ToF frontal (esquina de la nariz = tF·cos − 0.7 − 6.5·sin − v·LAG), sin corte HOLGURA/COAST por pose mientras nariz > 12 mm y pose > 4 mm | ESP fase 40 | 96: 6 vs 6, vueltas iguales, estacionar −0.64 s. orillas403: tF 11 cm libres y la pose cortaba cada tramo a ~100 ms |
| Gemelo: acomodo fijo de corrida real (`wro_field.layout_field`, TWIN_PARK_GAP_MM…), `FOX_PI5_CAMARA=1`, ghost_real.py | twin | — |

## Probado y rechazado hoy

- `PARK_PL_PWM_REV` 84→100: 8 vs 6 fallas (pared exterior n2/s4, cajón n2/s28).
- Port literal de 9c3eed39 (park5, alargar zigzag con FRENTE libre solo si |th|>22°): idéntico a la base, nunca se activa (en real va a 7-20°).
- ToF frontal sin piso de pose: 7 vs 6 (n1/s5 queda 9 mm fuera del cajón).

Todo está en `~/fox-coord.noindex/probado.md`.

## Pendiente

1. **Validar en el carro** el zigzag con ToF frontal y PARK_NEAR 140. Si roza la madera de enfrente con el carro chueco, subir el piso de pose (4 mm) o pedir |th| < 15°.
2. En orillas402 el ToF frontal no leyó nada en todo el parking (tF = −1, dF = 200). Ahí el cambio no aplica y los tramos siguen cortos. Revisar el ángulo y el alcance del ToF frontal dentro del cajón.
3. PWMs bajos del parking: las metas de velocidad (~83 mm/s en reversa) y las pausas del servo por tramo (hasta ~0.6 s) siguen igual. Subir la reversa se rechazó; falta atacar las pausas.
4. Solo en el gemelo: el arranque fija el cajón a w0=1000 con un solo voto "linea" malo (lay_user n2/n4).
5. **Bases del banco:** este commit trae el carrera.json de la Pi y cambios al gemelo, que mueven un poco los tiempos (n1/s5 43.42 → 43.92). Hay que correr una base nueva antes de comparar (`banco/local.sh <wt> base '{}' 1,2 12`). `pkv_base` ya no sirve.
