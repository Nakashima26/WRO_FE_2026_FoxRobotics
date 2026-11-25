# pk2_prior200_base

Tipo de prueba: carrera completa con estacionamiento (latencia/uno.sh, FF 2.8 / FD 3.8), banco NUEVO con latencia, parche pk2_prior200_base; TWIN_CAMERA_FROM_CALIB=1, TWIN_MOTOR_REAL=1, MOTOR_CURVA_REAL=1, VBAT 11.5, TOF_SHORT_K 0.5, COST_FIJO=1, COST_FIRST 1.95, MOTOR_CURVE_LIVE false, n1-2 x s20-21 (4 carreras), -P 7. Commit 75051700 (origin/grok/t/pk2_prior200_base).

## Resumen (resumen.py)

pk2_prior200_base: 0/4 ok  fallas: n1/s20 n1/s21 n2/s20 n2/s21

## Carreras (última línea STOP)

n1 s20 ERROR COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}
n1 s21 ERROR COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}
n2 s20 ERROR COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}
n2 s21 ERROR COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}

## Costo por carrera

COSTO n1 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}
COSTO n1 s21 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}
COSTO n2 s20 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}
COSTO n2 s21 COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}

## Tiempo de reloj total

1.9 s

## Validez

VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 1.9 s para 4 carreras (referencia ~10-20 s sin carga), carga máx 9.32, mem máx 2239 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms n/d.
