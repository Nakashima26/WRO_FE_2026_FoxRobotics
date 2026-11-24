# Respuesta: w_geom_rojo_r15_s5

_2026-11-24 00:13 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_geom_rojo_r15_s5 (3fa24ef) tal como la pediste: 12 carreras (n1-4 x s5 s32 s21), TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, cámara desde calibración, TWIN_FW_OVER PARK_GEOM_EXT=1 + PARK_GEOM_ROJO=1, TWIN_PI_EXTRA ROUTE_PARK_GEOM_EXT=1, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7. Corrió sola bajo twin.lock y no faltó ninguna dependencia del venv.

- resumen.py: `w_geom_rojo_r15_s5: 12/12 ok  media 43.16  mediana 43.47  mejor 39.15  fallas: `
- Reloj total: 20.8 s. La doy por **válida**: carga máx 2.72, sin OOM/Killed, media_ms (COSTO) medio de 11.1 ms.
- Tipo de prueba: carrera completa con estacionamiento (mismo tipo y misma configuración que w_geom_rojo_r14).

Comparación con r14 en las 6 carreras que coinciden (n1-2 x s5/s21/s32):
- **n2/s21**, que en r14 chocaba con el cajón durante el estacionamiento (38.7 s), ahora termina bien (40.06 s).
- Las otras 5 carreras siguen ok y bajan entre 0.06 y 1.65 s (por ejemplo, n1/s32 pasa de 44.94 a 43.28).
- n3-n4 también pasan todas (de 39.15 a 46.60 s).

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_geom_rojo_r15_s5.md`.

Siguiente: la muestra es chica. Para confirmar r15 sin regresiones, manda la tanda completa n1-2 x s1-48 con este mismo env y la comparo 1 a 1 contra r14 (89/96, media 42.28). Las 5 fallas tempranas de r14 (n1/s3, n1/s33, n1/s36, n2/s40, n2/s45) no dependen del estacionamiento, así que no espero que r15 las cambie.
