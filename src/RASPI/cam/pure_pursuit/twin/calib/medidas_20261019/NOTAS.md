# Calibración 2026-10-19 (Erick, CalibFox por Bluetooth)

CalibFox v1 de esta rama (`ae8e86ba`), por Bluetooth desde Windows (COM6).
Batería **12.20 V** al empezar. `runs/` está en `.gitignore`, por eso los datos
van aquí: para seguir, cópialos a tu `runs/calib/<fecha>` o pásale esta carpeta
directo a `calib_fit_*`.

## Paso 4 — encoder: **5.45 cuentas/mm** (CalibFox trae 5.60)

| Prueba | Cuentas | Distancia | cuentas/mm |
|---|---|---|---|
| A mano 1 | 5435 (al volver −19) | 1000 mm | 5.435 |
| A mano 2 | 5561 (al volver +100) | 1000 mm | 5.561 |
| A mano 3, con tope | 5509 (al volver +140) | 1000 mm | 5.509 |
| A mano 4, con tope | 5349 | 1000 mm | 5.349 |
| **Motor**, `PWM 120 2500 77` | **4031** | **~740 mm** (cinta) | **5.447** |
| Motor, servo 75 (se fue chueco) | 3999 | no medida | — |

- Empujando a mano varía ±2% aun con tope, y al regresar no vuelve a 0.
  `err=0` siempre (no es ruido). Lo más probable: al empujar, las llantas
  mueven el motor al revés por la caja 50.092 × 1.25 y a ratos se arrastran en
  el tapete en vez de girar. Con el motor jalando (como en carrera) salió 5.447,
  igual al promedio a mano (5.463). **Usar 5.45.**
- La relación sí es 50.092 × 1.25: 12 × 50.092 × 1.25 = 751.4 cuentas por
  vuelta de llanta → 5.60/mm con Ø 42.7. Medido 5.45 → la llanta rueda como
  **Ø ≈ 43.9 mm** (aplastado/desgaste, 2.7%). Con 1.5 o 2 el error sería de
  20% o más.
- `enc_runs.csv` tiene las 5 en el formato de `calib_capture enc`
  (la de motor con `dist_mm` = 740). Corridas de motor completas:
  `enc_motor_r1.txt` (servo 75) y `enc_motor_r2_s77.txt` (servo 77).

## Paso 7a — centro del servo: **77**

Con `SERVO 75` el carro se fue a la derecha. Con 77 fue recto (~0.74 m). Usa
`--center 77`. Ojo: RondaAbierta, VueltaCalibracion y casi todo `parking`
siguen con 75.

## Paso 5 — solo el bloque 1 hacia adelante; la reversa no se hizo

`ramp --reps 3` (PWM 20→130, paso 10, 800 ms), servo 77, `--maxpwm 255`.

- Solo cuentan **`ramp_fwd_r01` y `ramp_fwd_r02`** (v con cpmm 5.6; multiplicar
  por 5.60/5.45 = 1.028): PWM 50 ≈ 44 mm/s, 80 ≈ 160 mm/s, 110 ≈ 260 mm/s.
- **La parte en reversa no se corrió**: el carro no hizo esas corridas. Los
  archivos `ramp_rev_r01/r02` tienen el PWM negativo del comando pero el encoder
  en 0 todo el tramo: **no usarlos**. Sin el regreso, cada ida arrancó donde
  acabó la anterior y `ramp_fwd_r03` ya iba sin espacio (v = 0 a PWM 80):
  **tampoco usarla**. Repetir el paso 5 completo antes de ajustar el motor.

## Pendiente

Paso 5 (rampa 150→255, deadband, step), 6, 7b (radios), 8, 9 y 10, y la
Parte A (cámara). Antes de seguir: cargar 5.45
(`CPMM 5.45`) o cambiar el valor por defecto en CalibFox.

Dato aparte, de VueltaCalibracion (rama `parking`, radio = arco del encoder /
ángulo del BNO, eje trasero): a tope (servo 0/156) salió 183–213 mm con la
escala vieja de 6.71/mm → **≈ 225–260 mm** real; al 80% del tope, con 5.60/mm,
izquierda 214–224 y derecha 187–201 → **≈ 220–230 / 192–207 mm** real. La guía
espera 110–130 mm en los topes; no cuadra y vale la pena medirlo con el paso 7b.

