# w_geom_rojo_flash_smoke

Tipo de prueba: carrera completa con estacionamiento geométrico (latencia/uno.sh: 3 vueltas + estacionamiento; PARK_GEOM_EXT=1 + PARK_GEOM_ROJO=1, ROUTE_PARK_GEOM_EXT=1) en el banco NUEVO con latencia (FF 2.8 / FD 3.8, TWIN_COST_FIRST 1.95, TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, cámara desde calibración, lens_calib.npz). Semillas n1 x 16 35 32 (3 carreras), xargs -P 7, commit 337817e (origin/grok/t/w_geom_rojo_flash_smoke). Ese commit está vacío sobre 094f93c.

## Resumen (resumen.py)

w_geom_rojo_flash_smoke: 0/3 ok  fallas: n1/s16 n1/s32 n1/s35

## Carreras (última línea STOP, o ERROR + última línea)

n1 s16 ERROR COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}
n1 s32 ERROR COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}
n1 s35 ERROR COSTO {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}

## Costo por carrera

COSTO n1 s16 {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}
COSTO n1 s32 {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}
COSTO n1 s35 {"FF": 2.8, "FD": 3.8, "cuadros": 1, "media_ms": 0.0, "p50_ms": 0, "p95_ms": 0, "max_ms": 0, "pct_mas_25ms": 0.0, "dp_ms": []}

## Tiempo de reloj total

0.2 s (no es un tiempo real: las 3 carreras abortaron al arrancar)

## Validez

**INVÁLIDA (falla de infraestructura, no del código).** El disco de la caja de Grok estaba al 100 % (128G/128G, 0 bytes libres). En las 3 carreras, costo.py → drive.py → sim.run() → FirmwareSIL (fw.py:82, tempfile.NamedTemporaryFile) lanzó `FileNotFoundError: No usable temporary directory found in ['/tmp', ...]` antes del primer cuadro. El warm-up también falló (0 STOP). No hay ninguna medición, así que no hay nada que comparar con s35_base/r16_wstop/r17_noprepka.
