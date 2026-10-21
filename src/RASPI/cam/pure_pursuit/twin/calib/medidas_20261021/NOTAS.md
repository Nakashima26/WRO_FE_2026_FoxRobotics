# Calibración 2026-10-21 (Erick, CalibFox `ce7b4cb4` por Bluetooth)

Batería **12.5 V** al empezar y **12.4 V** después del paso 6. CalibFox con
`CPMM 5.45` (paso 4 del 2026-10-19), `MAXPWM 255` y servo centro **77**.
Velocidades de los CSV ya en mm/s reales (cpmm 5.45). Cada paso está en su
carpeta (`calib_capture` reescribe `ramp_*.csv` y `session.json` si se corre
dos veces en la misma).

## Antes de empezar

- **Driver TB6612 dañado y cambiado.** Con reversa: GPIO19 HIGH en la entrada
  del driver, la entrada del 18 en 0 V, PWMA 1.3 V (PWM 100), y aun así AO1/AO2
  en 0 V; adelante bien. Era una mitad del puente H. Con el driver nuevo la
  reversa va a −312 mm/s al aire, igual que adelante. (`p0_prueba_reversa_driver_danado.txt`
  es la prueba con el driver viejo: 468 cuentas adelante, 0 atrás.) Lo del
  2026-10-19 ("la reversa no se corrió") era esto.
- **BNO085 (paso 0b, a mano):** 3 vueltas a la izquierda +1078.3°, 3 a la
  derecha −1082.1° → promedio 1080.2°, **escala 0.9998**: no necesita corrección.
  Izquierda = positivo (como `yaw_deg`).

## Paso 5 — velocidad contra PWM

| Carpeta | Qué | Resultado |
|---|---|---|
| `p5_lento` | `ramp` 20→130 paso 10, 800 ms, ×3 | adelante PWM 80 ≈ 160, 100 ≈ 235, 120 ≈ 315 mm/s; reversa ~20% más lenta |
| `p5_rapido` | `ramp` 150→255 paso 35, 500 ms, ×3 | adelante 150/185/220/255 → ~387/568/713/**861**; reversa ~293/489/644/802 |
| `p5_zona_muerta` | `RAMP 30 80 1 250` ×3 (la guía usa 5→50: este carro no arranca antes de ~60) | arranca adelante en **PWM 60–72**, atrás en **67–77** |
| `p5_escalones` | `step` 90/160/230, 2.5 s, ×3 | v final 204/482/755 adelante, 137/410/697 atrás; τ ≈ 0.2 s |

- **Juego en la transmisión ≈ 40–50 cuentas (~8 mm):** al arrancar tras una
  reversa, el encoder salta ~42 cuentas a PWM 30 sin que el carro se mueva
  (`dead_fwd_r02/r03`).
- La reversa no regresa lo mismo que avanza (más lenta y arranca más tarde).

## Paso 6 — rodado y frenado (`p6_frenado/<coast|brake>/p<pwm>`)

`COAST p 1500 2000 [brake]`, 3 por combinación. Entre corridas el carro se
regresaba en reversa por distancia (encoder + rumbo del BNO), quedando a ±4 cm.

| | total | rueda tras el corte |
|---|---|---|
| coast 120 | 435–447 mm | **~10 mm** |
| coast 230 | 1119–1140 mm | **~114 mm** |
| brake 120 | 417–421 mm | **~5 mm** |
| brake 230 | 1035–1084 mm | **~20 mm** (la llanta derrapó en el piso al frenar; válidas) |

La caja 50:1 × 1.25 frena casi sola a baja velocidad. **Las llantas patinan
~1 cm en cada arranque** desde parado (el regreso en 10 tramos quedó 11 cm corto).

## Paso 7 — radios **con encoder + BNO** (`steer --auto`, sin cinta)

PWM **90** (a 70 casi no arranca), 6 s, ×2, servos simétricos alrededor de 77.
`p7_radios/steer.csv` (y `steer_params.patch`, `steer_ino_ppservogain.patch`
que generó `calib_fit_steer`):

| servo | 5 | 20 | 40 | 55 | 65 | 89 | 99 | 114 | 134 | 149 |
|---|---|---|---|---|---|---|---|---|---|---|
| R (mm) | 129 | 173 | 276 | 519 | 1092 | 1076 | 553 | 321 | 201 | 171 |

`calib_fit_steer`: ganancia **der 0.600 / izq 0.474** °rueda/°servo (CAD 0.627),
asimetría 24%, rueda máx 41.2°, R mín 129 mm. El 89 es el menos preciso
(1029/1128): cerca del centro el juego pesa mucho.

**Extremos** (`p7_extremos`, PWM 90, 4 s): derecha 2/8/12/16 → 123/134/146/154 mm
(sigue bajando hasta 2); izquierda 140/144/150/154 → 188/164/**154**/**254** mm:
en 154 la dirección se traba. **Rango útil: 2 … 150.**

**Centro** (`p7_centro`, PWM 90, 6 s, ~1 m): 75/77/79 → +3.4/+2.2/+2.2° por
metro (leve a la izquierda, casi recto). **Juego de ±2 alrededor de 77**:
correcciones de 1–2 unidades no mueven las llantas. Centro = **77**.

## Paso 8 — pérdida en curva y límite de agarre

`p8_curva`: `lock` PWM 160, 2.5 s, ×3: servo 15 → 430 mm/s (−11% vs recto),
R 158; servo 40 → 474 mm/s (−2%), R 285. El radio casi no cambia con la velocidad.

`p8_curva_rapida` (1.5 s, ×2) — **límite de agarre**:

| servo | PWM | v (mm/s) | R (mm) | R lento | a_lat (mm/s²) |
|---|---|---|---|---|---|
| 40 | 200 | 628 | 289 | 276 | ~1360 |
| 55 | 200 | 642 | 457 | 519 | ~900 (corto, poco confiable) |
| 40 | 255 | 829 | 291 | 276 | **~2360** |
| 55 | 255 | 854* | ~470 | 519 | ~1540 (*r01 marcó 979 > máx en recta: patinó) |
| 20 | 255 | 770 | **207** | 172 | **~2860** |
| 134 | 255 | 773 | **250** | 201 | **~2400** |
| 5 | 255 | 720 | **198** | 129 | **~2610** |

Con 40 todavía obedece; con 20, 134 y 5 la curva se abre 20–54% y la
aceleración lateral se topa en **~2.4–2.9 m/s²** (μ lateral efectivo ≈ 0.25–0.29).
**El `A_LAT` del optimizador (~2770) está justo en el límite**; con el margen de
la guía (×0.8) conviene **≤ ~2300 mm/s²**, y a la izquierda parece un poco menos.

## Pendiente

Paso 9 (agarre con tabla inclinada: confirmaría el μ de arriba sin arriesgar
el carro), paso 10 (ToF, mejor con VueltaCalibracion: el TOF de CalibFox viene
apagado y con XSHUT de otro carro) y la Parte A (cámara).
