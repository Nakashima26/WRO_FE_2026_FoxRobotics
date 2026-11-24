# g1b — seguimiento de g1: RMIN70 solo + diagnóstico n2/s34, n2/s39 con PAUSA corta — FIN

- **Tipo de prueba: U-turn + parking** (carrera completa de 3 vueltas + estacionamiento, con o sin U según la semilla; harness `latencia/uno.sh`, cen_seeds). Mismo tipo y banco que g1; solo se compara contra tandas de este tipo.
- **Banco** (idéntico a g1): origin/main **2813d20**, TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, cámara desde calib + lens_calib.npz, TWIN_COST_FIJO=1, COST_FIRST=1.95, FF 2.8 / FD 3.8, xargs -P 7, ruidos n1,n2 × semillas s1..48 = 96 corridas. Cada tanda corrió bajo `flock /workspace/fox/twin.lock`, una a la vez, con procesos ajenos=0 y sin Killed/MemoryError.
- **RMIN70** = `MOTOR_REV_RAMPA_MIN` 55 -> 70 (const del firmware en PurePursuitCarrera.ino + la entrada `fw` en carrera_bench.json). Es el mismo diff que dejó preparado g1 (`g1t_rm70`, que no llegó a correr). Worktrees nuevos: `g1b_wt` (base limpia) y `g1b_rm_wt` (RMIN70), los dos en 2813d20.

## Discrepancia de la base (95/96 vs 93/96)
- `g1_work/g1_base_resumen.txt` (93/96, media 42.20, fallas n1/s36 n1/s43 n2/s20) y `g1_base_rep` (igual) son de **a943835 con parches** (dirty=1), no de main. g1.md ya lo aclaraba: esas tandas no son comparables.
- La base real de main 2813d20 es `g1m_base` / `g1m_base_rep`: **95/96, media 42.22**, falla n1/s36.
- La volví a correr en g1b (`g1b_base`, worktree limpio): **95/96, media 42.22, mediana 41.94, mejor 36.60, falla n1/s36**. Coincide corrida por corrida con g1m_base (0 diferencias), o sea que el banco es determinista y es el mismo.

## Resultado: base vs RMIN70 (96 corridas)
| tanda | ok | media (s) | mediana | mejor | fallas | wall (s) | pi_frame_ms medio |
|---|---|---|---|---|---|---|---|
| g1b_base (2813d20) | 95/96 | 42.22 | 41.94 | 36.60 | n1/s36 | 134.9 | 3.57 |
| g1b_rm70 | 94/96 | 42.05 | 41.80 | 36.05 | n1/s36 **n2/s34** | 133.6 | 3.57 |

- Diferencia pareada en las 94 que pasan en las dos: **−0.20 s** (mediana −0.20); mejora en 71/94. RMIN70 mueve el tiempo de todas las corridas (la rampa de reversa actúa en cada reversa), así que las 96 "cambian".
- La falla nueva, **n2/s34**, es un caso al límite: termina estacionado, sin choque, pero queda **5.0 mm** fuera del cajón (el umbral es ≤ 5.0 mm; en la base ya quedaba en **4.9 mm**). El que sobresale es el lado hacia la pista (y<1300 en el cajón N): el plan cierra con w≈79 y rumbo 271.9° en vez de 271.3°.

## Réplica n3–n8 (semillas que cambian de estado o quedan al límite)
Semillas s11, s13, s34, s36, s39, s40 (s34 cambió de estado; s36 es la falla base; s11, s13 y s40 tuvieron fuera > 0 mm en alguna de las dos; s39 se pidió en el diagnóstico) × ruidos n3..n8 = 36 corridas por tanda:
| tanda | ok | media | mediana | mejor | fallas | wall (s) | pi_frame_ms |
|---|---|---|---|---|---|---|---|
| g1b_base_r38 | 35/36 | 42.13 | 41.41 | 38.73 | n5/s36 | 50.4 | 3.66 |
| g1b_rm70_r38 | 35/36 | 41.99 | 41.27 | 38.84 | n5/s36 | 51.4 | 3.73 |

- s34 pasa en n3..n8 en las dos variantes (fuera 0 mm), así que la falla de n2/s34 es de ese ruido en particular, no de la semilla. s36 falla igual en las dos (choque con la señal Green S/T1 en la vuelta 1, t≈7 s; no tiene que ver con el parking). Pareado: −0.14 s (mejora en 23/35).
- Total, 132 corridas: base 130/132, media 42.20; RMIN70 129/132, media 42.03; pareado −0.18 s.
- CSV por corrida: `resultados/g1b_corridas.csv` (132 filas).

## Diagnóstico de n2/s34 y n2/s39 con PAUSA corta (logs de g1t_p150 = PAUSA 150 y g1t_c2 = PAUSA 100 + ARC1 150 + RMIN70)
**Qué pasa en el gemelo.** `PARK_PL_PAUSA_MS` es el tiempo que el carro se queda parado con el volante girando antes de cada tramo del plan. En el gemelo, el servo va a 600 °/s (`steering.slew_deg_per_s`, un valor **supuesto** del datasheet del SG90, no medido) y las ruedas lo siguen con un retraso de primer orden de τ=0.10 s (este sí está medido). Un cambio de 75 a 156 tarda unos 135 ms solo de servo, más el retraso de las ruedas; de tope a tope (1↔156) son unos 260 ms. Con 220 ms las ruedas llegan casi al tope. Con 150 o 100 ms el tramo arranca con las ruedas todavía girando: el arco sale más abierto y avanza más mm por cada grado de rumbo. En el trace se ve que en c2 el motor arranca unos 90 ms después del cambio de servo (37.433 -> 37.524), contra unos 210 ms en la base.

**n2/s34** (CCW, cajón N, **parking sin U**: "PARK sin U", fases 1 -> 2 -> 40 PLAN, 3 tramos):
| | plan1 fin th | plan2 fin (HOLGURA) th / mm | plan3 fin | pose final | fuera |
|---|---|---|---|---|---|
| base (220) | 44.5 | 21.7 / 44 | TH, th 2.8, w=78 | hdg 271.3 | 4.9 mm ok |
| p150 | 44.7 (más "encoder avanza pero el BNO no gira (patina?)") | 22.9 / 42 | **FRENTE** (dF 6.0), th 5.1, w=81 | hdg 269.4 | 7.8 mm FALLA |
| c2 (100) | 45.3 | 21.9 / 45 | TH, th 2.9, w=83 | X −295.8 | 6.6 mm FALLA |
| RMIN70 solo | 45.2 | 21.6 / 45 | TH, th 2.7, w=79 | hdg 271.9 | 5.0 mm FALLA |

No hay choque ni contacto en ninguna. Es un caso estructuralmente al límite: el plan siempre deja el costado del carro hacia la pista (borde y=1300), con el centro en Y≈1360 cuando el cajón tiene 200 mm de profundidad. Cualquier cambio de unos pocos mm o de 1–2° en el último tramo lo saca del umbral de 5 mm. La pausa corta cambia el th con el que cierran los tramos y la w final (78 -> 81–83). Falta en el plan una corrección lateral, o un objetivo de profundidad con margen, al final del tramo 3.

**n2/s39** (CW, cajón W, **con U**: "PARK U por pose", fase 27 ENDEREZA, luego PLAN de 4 tramos):
- Base: plan2 HOLGURA th 37.6 (27 mm), plan3 HOLGURA, plan4 REVERSA INT hasta TH 2.7, centrar adelante 16 mm. OK.
- p150: plan2 cierra con th 32.8 tras 35 mm (arco más abierto), plan3 HOLGURA y plan4 termina por "huella dentro". OK, a 0 mm.
- c2: también aparece "patina?" en el plan 1. Plan2 cierra con th 31.0 tras 39 mm, plan3 se corta por **FRENTE** (ToF frontal dF 6.15 cm, distB 6.6) y en el plan 4 (REVERSA INT, th 7.7 + atasco +20) **choca con el cajón magenta** a t=39.30 s (STOP collision). Solo falla en c2: con PAUSA 150 sola pasa, y con RMIN70 solo también pasa (40.11 s). La falla sale de **combinar** la pausa corta (arco abierto y el carro más atrás) con una reversa más fuerte (RMIN70 y ARC1 150) en el último tramo, cuando queda poco espacio atrás.

**Conclusión del diagnóstico:** no es un bug de estado; la culpa es la geometría del plan y el tiempo de asentamiento del volante. Bajar la pausa solo es seguro si el tramo espera a que las ruedas lleguen (servo en posición). Alternativa: pausa variable según el tamaño del giro, por ejemplo unos 100 ms para cambios chicos y ≥220 ms para tope a tope. Ojo: en el gemelo, el efecto depende de `slew_deg_per_s`=600, que no está medido. Conviene medir el slew real del servo antes de ajustar PAUSA.

## Validez
- Todas las tandas con ajenos=0, sin Killed/MemoryError, memoria máxima ~4.5 GB y carga máxima 6.5–7.1 (son los 7 workers propios).
- Wall: 134.9 y 133.6 s por 96 corridas. Está por encima de la referencia general de ~100–110 s, pero es lo habitual en este banco U+park (g1m_base 140.1, g1t_p150 135.7, g1t_c2 138.0). El mean_pi_frame_ms (3.57) es igual o menor que el de g1m_base (3.71), y g1b_base reproduce g1m_base corrida por corrida, así que no hubo contención. **Las 4 tandas son válidas.**

## Decisión
**RMIN70 rechazado: no se commitea nada.** No cumple el criterio: tiene 2 fallas contra 1 de la base (n2/s34), aunque la media baja 0.17 s (pareado −0.20). La falla nueva queda justo en el umbral (5.0 vs 4.9 mm) y no se repite en n3–n8, así que el riesgo real parece bajo. Aun así, la regla es dura. La rama grok/t/g1b no se creó y main no se tocó.
Sugerencia: si se arregla el margen lateral del final del plan (s34 sale con 4.9 mm ya en main), vale la pena volver a probar RMIN70, porque es una mejora de ~0.2 s bastante pareja (gana en 71 de 94 corridas).

Todo es del gemelo; nada se midió en el carro.

FIN
