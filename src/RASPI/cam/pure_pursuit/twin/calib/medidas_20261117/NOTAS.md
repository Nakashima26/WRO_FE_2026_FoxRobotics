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
(250 → 1738). En piso la reversa sale bastante más lenta (ver segunda sesión).

## Segunda sesión (misma tarde, pila a 11.88 V; las de arriba fueron con la pila a tope)

N30Test ampliado: cada corrida imprime aceleración (ms a 63 % / 90 % de v, con
velocidad de 50 ms) y la parada (FRENO en corto o SUELTO con `,c`), del encoder.

**Arranque en piso** (rampa `a`/`A`, +1 PWM cada 40 ms): adelante PWM **11**, reversa **10**.

**Adelante a 11.88 V** (≈15 % abajo de la pila a tope):

| PWM | v (mm/s) | 63 % / 90 % | Parada |
|---|---|---|---|
| 100 | 503 | 310 / 630 ms | FRENO desde 500: 26 mm en 405 ms (~4.8 m/s², sin patinar) |
| 150 | 798 | 320 / 600 ms | SUELTO desde 803: 329 mm en 885 ms (~1.0 m/s²) |
| 180 | 1062 | 310 / 660 ms | FRENO desde 1066: encoder 64 mm, la llanta se bloquea y patina |
| 255 | 1501 | 290 / 620 ms | FRENO desde 1502: encoder 88 mm + **~15 cm patinando (cinta)** ≈ 240 mm, ~4.7 m/s² |
| 255 | 1407 (2 m) | 270 / 580 ms | SUELTO desde 1409: 944 mm en 1.5 s (~1.05 m/s²) |

Ajuste 100…255: **k 6.51, pwm0 22.9**. Tope a 11.9 V ~1.5 m/s.

**Reversa** (arrancando de golpe la llanta patina al principio; a velocidad
constante va más cargada que adelante, sospecha: ruedas delanteras arrastrando
con el juego del servo):

| PWM | v (mm/s) | Parada (FRENO, encoder) |
|---|---|---|
| 60 | ~70–117 (no llegó a 0.6 m en 5 s) | — |
| 100 | 370 | desde 351: 13 mm |
| 140 | 624 | desde 630: 31 mm |
| 180 | 802 (v2 847, aún acelerando) | desde 864: 56 mm |

Ajuste 100/140/180(~850): **k 6.0, pwm0 37.6**.

## Lo que cambió

| Dónde | Antes (N20) | Ahora (N30) |
|---|---|---|
| `params.py` `EncoderParams.counts_per_mm` | 5.45 | **7.523** |
| `params.py` `k_mm_s_per_pwm` / `pwm0` | 3.82 / 45.5 | **6.51 / 22.9** (11.88 V) |
| `params.py` `k_rev_mm_s_per_pwm` / `pwm0_rev` | 3.61 / 61.0 | **6.0 / 37.6** |
| `params.py` `tau_drive_s` | 0.173 | **0.28** |
| `params.py` `coast_decel_mm_s2` | 2700 | **1000** |
| `params.py` `brake_decel_mm_s2` / `brake_lock_speed_mm_s` | 12000 / 550 | **5000 / 600** |
| `params.py` `brake_slide_decel_mm_s2` / `brake_wheel_tau_s` | 5500 / 0.02 | **4700 / 0.06** |
| `PurePursuitCarrera.ino` `ENC_CUENTAS_POR_MM` | 5.45 | **7.523** |
| `PurePursuitCarrera.ino` `MOTOR_K_*`, `MOTOR_PWM0_*` | — | 6.51/22.9 adelante, 6.0/37.6 reversa |
| `PurePursuitCarrera.ino` `LAZO_V_255` / `LAZO_V_MAX` | — / 700 | **1575** (255 = máx. con pila a tope) / **0** (sin tope) |

En el `.ino`, `LAZO_K_*`/`LAZO_PWM0_*` (3.82/45.5, 3.61/61) se quedan como
**escala de mando**: el PWM que piden los estados sigue significando la misma
velocidad que con el N20. El lazo usa `MOTOR_K_*` para el feedforward, escala
KP/KI por `LAZO_K_FWD / MOTOR_K` (misma ganancia en mm/s) y, cuando no actúa
(cambio de sentido, contramarcha, debajo del arranque), manda el PWM
equivalente del N30 (`pwmMotorEquiv`) en vez del PWM crudo. `LAZO_V_MAX` pasó
de 700 a 0 (sin tope).

`vMando()` (commit a3647f69 + esta sesión): hasta PWM 155 el mando significa lo
mismo que con el N20 (155 -> 418 mm/s); de 155 a 255 sube recto hasta
`LAZO_V_255` (1575 en e994bba0; **regresado a 800** tras orillas65, ver docs/notes/orillas65_n30). `MOTOR_MAX` sigue en
155: la carrera no cambia hasta que se suba. El perfil de la Pi
(`ROUTE_PROFILE_K_MM_S_PER_PWM` 3.5 / `PWM0` 25) no usa esta escala.

**Sin medir con el N30**: `a_max_*`, `scrub_coeff`, giros en reversa. El
deslizamiento del freno desde 1066 no se midió con cinta.

**Para carrera rápida:** a 1.3–1.5 m/s el freno en corto bloquea la llanta
(~24 cm para parar, ~15 cm que el encoder no ve) y suelto rueda ~1 m. Frenar
antes de curvas con el lazo (rampa de velocidad), no con freno en corto.
