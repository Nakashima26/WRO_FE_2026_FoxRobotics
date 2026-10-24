# Calibración 2026-10-24 (Erick, CalibFox por Bluetooth + SeguirRuta)

Batería **12.1 V** al empezar y **11.93 V** al final. CalibFox con `CPMM 5.45` y
`MAXPWM 255`, manejado con `scripts/medir_bt.py` de la rama `parking`. Los CSV
están en `logs_tof/` de esa rama (`<prueba>_<volt>V_<fecha>_<hora>.csv`). La
tabla completa en inglés está en `docs/route-following-bench.md` §7.1–7.2 de `parking`.

## Lo que cambió en el gemelo (`params.py`, `vehicle.py`, `sim.py`)

| | Antes | Ahora | De dónde |
|---|---|---|---|
| Centro del servo | 78 (mitad de 1..156) | **74.5** | 75 ±0.3 en rectas; 74.5 del ajuste a 5 vueltas (3 antihorario, 2 horario) |
| Juego servo→ruedas | — (zona muerta 2) | **±1.5 unidades** (`backlash_deg`), zona muerta 0.5 | desde 55 la rueda queda −1.1…−1.45°, desde 95 +0.9…+1.1° |
| Retraso de las ruedas | — | **0.10 s** (`lag_tau_s`) | ajuste a las 3 vueltas de referencia |
| Dirección | lineal 0.552 der / 0.588 izq | tabla por lado (`table_right/left`) | arcos a PWM 150: der 60/45/1 → 8.7/18.2/42.5°, izq 90/105 → 7.3/15.0°; la izq ×1.04 por el ajuste a las vueltas |
| Velocidad | k 4.02, pwm0 42.1 (12.5 V) | **k 3.82, pwm0 45.5** (12.0 V) | 150 → 394–404, 255 → 789–826 mm/s |
| Rodar suelto | exponencial τ 0.091 s | **2.7 m/s² parejo** (`coast_decel_mm_s2`) | desde 255 rueda 113–123 mm, desde 150 31 mm |
| Reversa | k igual que adelante, sin patinar | **k_rev 3.61, pwm0_rev 61**; arriba de 1.2 m/s² las llantas patinan (encoder cuenta de más); giros 10 % más abiertos | −255 de golpe: encoder 661, cinta 550; con la rampa de 300 ms no patina; a tope en reversa R 138/162 |
| Freno en corto | igual que manejar (τ 0.173 s) | **12 m/s²**; arriba de 550 mm/s la llanta se bloquea: **5.5 m/s²** y el encoder deja de contar | desde 150 para en 6–7 mm; desde 255 el encoder ve 13–23 mm y el carro patina 24–51 mm más |

Agarre lateral: el tope medido (servo 1: 230–237 °/s, ~2.5–2.8 m/s² a 210–255)
coincide con el `a_lat_right_mm_s2` = 2750 que ya estaba; no se cambió.

## Comprobación

`tools/calib/replay_seguir_ruta.py` repite los comandos (servo, PWM) de las
vueltas reales en el modelo del carro, por ventanas de 0.5 s que arrancan con la
velocidad real:

| | Error de giro por 0.5 s (rms) | Distancia (real 22.25–22.37 m) |
|---|---|---|
| Antes | 10.0–10.6° | 23.3–23.7 m (+4.5 %) |
| Ahora | **1.2–1.6°** | 22.24–22.58 m |

## Pendiente / ojo

- `PurePursuit.ino` todavía tiene `centroServo = 78` y `RUEDA_POR_TICK_DER/IZQ` 0.552/0.588:
  en el carro real (y ahora en el gemelo) eso tira ~2° a la izquierda en recta.
- ToF frontal (Long, ROI 8, pared negra): bien hasta ~80 cm solo si está nivelado. Inclinado
  hacia arriba el cono pasa por encima de la pared de 10 cm (a 80 cm leyó 1650–1856). El
  modelo 2D de `sensors.py` no tiene la altura de la pared: eso no está en el gemelo.
- Las pruebas de una sola línea (escalones, freno, suelto, arcos, reversa) repetidas en el gemelo quedan a ~5 % del carro.

## SeguirRuta en el gemelo (lazo cerrado)

`tools/calib/seguir_ruta_sil/sr_sil.py` compila el `SeguirRuta.ino` de `parking` en la PC
(zig) con imitaciones del hardware y el `Vehicle` de arriba como física, y le manda lo
mismo que mandó `ruta_bt.py` en cada vuelta real (ruta, salida y todos los `# par` del log).
Sensores ideales y sin ToF. 3 vueltas a 255/230, lookahead 275:

| Vuelta | Tiempo carro / gemelo (s) | Encoder (mm) | Fin odometría (mm) | Lateral máx / rms (mm) | Servo medio |
|---|---|---|---|---|---|
| 125121 antihorario | 29.92 / 29.50 | 22417 / 22380 | 1504,506 / 1496,503 | 17/5 / 19/8 | 83.0 / 83.9 |
| 125707 antihorario | 29.09 / 29.50 | 22407 / 22380 | 1503,509 / 1496,503 | 16/5 / 19/8 | 82.9 / 83.9 |
| 130029 antihorario | 29.20 / 29.50 | 22405 / 22380 | 1498,506 / 1496,503 | 16/4 / 19/8 | 83.1 / 83.9 |
| 132712 horario | 29.09 / 29.24 | 22315 / 22282 | 491,998 / 490,997 | 92/18 / 92/15 | 67.2 / 66.3 |
| 133331 horario | 29.25 / 29.24 | 22319 / 22272 | 490,997 / 490,997 | 16/4 / 17/7 | 67.0 / 66.1 |

- El tiempo del gemelo cae dentro de lo que varía el carro entre vueltas (29.1–29.9 s).
- El gemelo pide ~1 unidad más de servo en las curvas (los dos lados) y baila un poco más
  en las rectas (rms 7-8 contra 4-5 mm): sus ruedas giran ~10 % menos que las del carro
  a 800 mm/s en curvas de R 300, aunque en los arcos a 150 dio lo contrario a la izquierda.
- 132712: en el carro `salidaPared` vio una pared fantasma y corrió la salida 9 cm; el
  gemelo arranca con la misma creencia y su carro "de verdad" termina a 93 mm de su
  odometría, como lo que viste ("acabó más a la izquierda").

### Ruta de prueba sin paredes (`scripts/rutas/prueba_gemelo.csv` de parking, 2 vueltas, 11.88 V)

Sale y llega a (1000, 500); curvas izq R400/R300 y una der R300. Sin ToF en los dos.

| Corrida | Tiempo carro / gemelo (s) | Encoder (mm) | Lateral máx / rms (mm) | Servo medio | |Δservo| curva |
|---|---|---|---|---|---|
| 145031 a 150/150 | 35.77 / 34.72 | 13624 / 13636 | 34/8 / 33/12 | 83.3 / 85.1 | 1.18 / 0.59 |
| 145320 a 255/230 | 18.56 / 18.56 | 13714 / 13719 | 23/5 / 26/9 | 84.4 / 85.5 | 2.22 / 1.13 |
| 145447 a 255/230 | 18.53 / 18.56 | 13717 / 13719 | 21/5 / 26/9 | 84.3 / 85.5 | 2.22 / 1.13 |
| 145534 a 255/230 | 18.70 / 18.56 | 13709 / 13719 | 23/6 / 26/9 | 84.3 / 85.5 | 2.33 / 1.13 |

- A 255 el tiempo del gemelo cae dentro de las 3 del carro (18.53–18.70). A 150 el carro fue 3 %
  más lento (11.88 V; el gemelo con motor −3 % da 35.72 s).
- El carro termina donde arrancó (el usuario a ojo; la cinta no se midió).
- Lo que falta en el gemelo: ~1 unidad menos de servo en curvas y el ruido de los sensores (el
  servo del carro vibra el doble; el del gemelo, con encoder y BNO perfectos, oscila más lento
  y su lateral rms sale 9 contra 5 mm).
