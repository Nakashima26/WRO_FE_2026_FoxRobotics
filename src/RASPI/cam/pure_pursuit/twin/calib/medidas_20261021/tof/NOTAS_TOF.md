# ToF VL53L1X en el carro v2 — cómo leen de verdad (2026-10-19/21)

Para ajustar `ToFParams` del gemelo y el localizador. Hoy `params.py` supone
`black_wall_max_mm = 250` (viene del README del **VL53L0X** viejo), ROI completo
(`half_fov_deg = 13.5`) y ruido 5 mm + 1 %. Nada de eso es lo que hace el carro.

## Cómo se midió

Pared **negra de la pista**, perpendicular, distancia con cinta **desde la cara
del sensor**. VueltaCalibracion en espera; `scripts/tof_estatica.py` (rama
`parking`) cambia la config por BT (`c <S|M|L> <ROI> <ms>`) y junta ~3 s por config.
`med` = mediana; `min-max` = rango en ~30-50 lecturas.

## Lateral izquierdo (`lateral_izq_estatica.csv`)

Error = medido − real, en mm (rango min..max entre paréntesis):

| real | **Short ROI 8 50 ms** | Long ROI 8 50 ms (lo que usa RondaAbierta hoy) | Short ROI 16 | Long ROI 16 |
|---|---|---|---|---|
| 100 | −4 (94..97) | −4 (93..98) | +19 | +21 |
| 200 | −5 (192..197) | +2 (199..208) | +17 | +17 |
| 300 | −3 (292..302) | +22 (309..333), **35/42 status 7** | +19 | +22 |
| 450 | 0 (443..464) | **+53 (275..606)**, status 0/2/7 revueltos | +12 | +21 |
| 600 | −8 (572..607), todo status 2 | **+135 (665..814)** | −50 | −29 |
| 800 | −40 (723..798), todo status 2 | **+151 (821..1115)** | −190 | −162 |

- **Short + ROI 8 es la buena para los laterales**: < 1 cm hasta 60 cm, −4 cm a
  80. El status 2 (poca luz) a 60-80 cm es la distancia correcta: **no tirarlo**.
- **Long + ROI 8** (hoy en el firmware de RondaAbierta) se desarma desde 30 cm:
  lee de más y crece con la distancia; a 45 cm salta entre 27 y 60 cm.
- **ROI 16** (cono ~27°) ve el **piso** desde ~50 cm (sensor a ~5-6 cm del piso):
  la lectura sale corta y no lineal. Con ROI 8 el cono es ~13-14° total
  (`half_fov_deg` ≈ 7).
- La pared negra **sí se ve a 80 cm** (y probablemente más; falta 100-120 cm).
  `black_wall_max_mm = 250` está muy corto.
- Pendiente: el lateral derecho (otra unidad, otro montaje), qué marca en Short
  **sin pared** (esquina abierta), y 100-120 cm.

## Frontal (`frontal_estatica_20261019.csv`, Long ROI 8 50 ms)

| real | medido | error | rango | status |
|---|---|---|---|---|
| 300 | 332 | +32 | 317..346 | 7 |
| 600 | 787 | **+187** | 716..884 | 2 |
| 1000 | 1217 | **+217** | 1131..1371 | 2 |
| 1500 | 1690 | **+190** | 1534..1923 | 2 |

De 60 a 150 cm lee **~+20 cm casi constante** (no proporcional), con ruido de
±7 a ±20 cm. RondaAbierta lo corrige restando 200 mm con status 2 y 30 mm con 0/7.
Falta probar el frontal en Short (todo lo que decide con él pasa a < 1 m).

## Lo que se vio corriendo (vueltas de VueltaCalibracion y RondaAbierta)

- Con Long, en una pared negra REAL la mayoría de lecturas laterales salen con
  **status 7 (wrap fail)** con la distancia correcta (±4 cm): el firmware las
  tiraba y "no veía" la pared interior. Ya se aceptan.
- Sin pared (esquina abierta), en Long los laterales dan status 2 a 1.2-2.8 m
  (nunca status 7 corto).
- Un ToF a veces deja de mandar datos (se rearranca por XSHUT). Y el loop del
  ESP32 tiene pausas de 41/82/123 ms en carrera (sospecha: lecturas I2C que
  esperan timeout; en investigación).

## Sugerencia para `ToFParams` (si el firmware pasa los laterales a Short ROI 8)

- `half_fov_deg ≈ 7` (ROI 8), no 13.5.
- `black_wall_max_mm` ≥ 800 en laterales Short (confirmar 100-120 cm).
- Error lateral Short: ≈ 0 ± 5 mm hasta 30 cm, ± 10-20 mm a 45-60, −40 ± 40 a 80.
- Frontal Long: +200 mm de sesgo con status 2 (> ~50 cm), ± 7-20 cm de ruido;
  +30 mm cerca (status 7).
- Mientras el firmware siga en Long para todos, el modelo debe usar la columna
  "Long ROI 8" (sesgo que crece hasta +15 cm y nubes de 30 cm a 45 cm).
- Montaje de los sensores (`tof.mounts`) sigue sin medir.
