# Dirección medida 2026-10-22 (CalibFox `steer --auto`, encoder 5.45 cuentas/mm + yaw del BNO)

Radio = avance del centro del eje trasero / giro, sin el primer segundo. Rango del servo 1..156
(en 160 el servo se queda forzando contra el tope; en 156 no).

| carpeta | qué es |
|---|---|
| `rango_tapete_p150/` | **la buena**: tapete, PWM 150, 156→1, 3 repeticiones (±1 mm) |
| `rango_piso_p090/` | piso, PWM 90, 1→156; patina, radios de la izquierda inflados (no usar para el twin) |
| `extremos_160_1/` | tapete, PWM 150, alternando 160 y 1 (histéresis); solo quedó el último CSV de cada uno, los 6 radios están en `steer.csv` |
| `s160_espera3s/` | tapete, 1→160 esperando 3 s antes de arrancar: 131/132/132 mm (no es tiempo de llegada) |
| `s160_llantas_a_mano/` | servo 160 con las llantas acomodadas a mano: 120 mm (el juego está en la llanta de afuera) |

Resumen en el tapete (PWM 150), bici = atan(113 / R):

| servo | 1 | 4 | 8 | 16 | 140 | 144 | 148 | 152 | 156 |
|---|---|---|---|---|---|---|---|---|---|
| R (mm) | 123 | 130 | 142 | 166 | 153 | 140 | 136 | 134 | 136 |
| bici (°) | 42.6 | 40.9 | 38.5 | 34.3 | 36.4 | 38.8 | 39.8 | 40.1 | 39.8 |

Con centro 78: derecha lineal 0.552 °/°servo; izquierda 0.588 °/°servo hasta 144 y de 148 en adelante
plano en ~39.9° (pasando 144 solo gira la llanta de adentro). Histéresis: servo 1 llegando desde 160
da 126-127 mm contra 123 llegando desde 4. Medidas del carro: 176 × 125 mm.
