# Motor N30 30:1 + encoder 7 CPR (2026-11-17, Erick, `src/ESP32/N30Test`)

Se cambió el motor de tracción del N20 (50:1, encoder 12 CPR) a un **N30 30:1 con
encoder de 7 pulsos por canal** (28 cuentas x4 por vuelta del motor). Mismos
pines (A→34, B→36, PCNT x4 con filtro de 1 µs), mismo driver (TB6612, PWMA 23,
AIN1 18, AIN2 19), mismos engranes 1.25.

Medido con `N30Test` por USB y luego por Bluetooth (COM6, nombre "CalibFox"),
servo fijo en 75, freno en corto al final de cada corrida.

## Encoder

| | N20 (antes) | N30 (ahora) |
|---|---|---|
| Cuentas/mm | 5.45 | **7.523** (7523 cuentas empujando 1000 mm; otra medida ~7.56) |
| Teórico | 12 × 50 × 1.25 = 751/vuelta → 5.60 | 28 × 30 × 1.25 = 1050/vuelta → 7.83 |
| Sentido | adelante = negativo | adelante = negativo (`ENC_SIGNO` −1 sin cambio) |

Medido/teórico 0.96 (N20 0.97): la llanta rueda como de ~44 mm, no 42.7.
A/B limpios (pasan por 00/01/11/10), sin cuentas en reposo.

## Velocidad en piso, adelante (`f<pwm>[,<mm>]`)

v = promedio en la segunda mitad del recorrido; v2 = último cuarto.

| PWM | v (mm/s) | Recorrido | Nota |
|---|---|---|---|
| 60 | 265 | 0.73 m | cortó por tiempo (3 s) |
| 100 | 593 | 1.2 m | |
| 140 | 809 | 1.2 m | |
| 180 | 1077 | 1.2 m | probablemente aún acelerando (bajo) |
| 220 | 1220 | 1.2 m | aún acelerando, descartado |
| 220 | **1366** | 2.5 m | v2 1383: estable |
| 255 | **1556** | 2.5 m | v2 1592: casi estable → tope ~1.6 m/s |

Ajuste lineal (60…255, sin el 220 corto): **k 6.64 mm/s por PWM, pwm0 16.6**,
residuos ±40 mm/s. Solo 100…255: k 6.46, pwm0 11.2. El pwm0 es de la recta; el
arranque real con carga es más alto (a 60 la recta da 404 y medido 265).

## En el aire (`k`/`K`, sin carga, referencia)

Adelante k 7.56, pwm0 5.3 (50 → 341, 250 → 1836 mm/s). Reversa k 7.14, pwm0 2.9
(250 → 1738). La reversa en piso NO se midió: se usa 5 % menos que adelante.

## Lo que cambió

| Dónde | Antes (N20) | Ahora (N30) |
|---|---|---|
| `params.py` `EncoderParams.counts_per_mm` | 5.45 | **7.523** |
| `params.py` `MotorParams.k_mm_s_per_pwm` / `pwm0` | 3.82 / 45.5 | **6.64 / 16.6** |
| `params.py` `k_rev_mm_s_per_pwm` / `pwm0_rev` | 3.61 / 61.0 | **6.27 / 16.6** (supuesto) |
| `PurePursuitCarrera.ino` `ENC_CUENTAS_POR_MM` | 5.45 | **7.523** |
| `PurePursuitCarrera.ino` `MOTOR_K_*`, `MOTOR_PWM0_*` | — | nuevos (6.64/16.6, 6.27/16.6) |

En el `.ino`, `LAZO_K_*`/`LAZO_PWM0_*` (3.82/45.5, 3.61/61) se quedan como
**escala de mando**: el PWM que piden los estados sigue significando la misma
velocidad que con el N20. El lazo usa `MOTOR_K_*` para el feedforward, escala
KP/KI por `LAZO_K_FWD / MOTOR_K` (misma ganancia en mm/s) y, cuando no actúa
(cambio de sentido, contramarcha, debajo del arranque), manda el PWM
equivalente del N30 (`pwmMotorEquiv`) en vez del PWM crudo. `LAZO_V_MAX` sigue
en 700: con la escala de mando el tope pedible es 3.82·(255−45.5) ≈ 800 mm/s.

**Sin medir con el N30** (siguen los del N20 en `params.py`): `tau_drive_s`,
`tau_coast_s`, `coast_decel_mm_s2`, freno (`brake_*`), `a_max_*`, reversa.
