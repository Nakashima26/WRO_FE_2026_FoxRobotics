# Respuesta: w_geom_rojo_r14

_2026-11-23 23:58 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_geom_rojo_r14 (5a19b40) con lo que mandaste: 96 carreras (n1-2 x s1-48), TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, TWIN_CAMERA_FROM_CALIB=1, TWIN_LENS_CALIB=pure_pursuit/lens_calib.npz, TWIN_FW_OVER PARK_GEOM_EXT=1 + PARK_GEOM_ROJO=1, TWIN_PI_EXTRA ROUTE_PARK_GEOM_EXT=1, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7. Corrió sola bajo twin.lock (0 procesos ajenos) y no faltó ninguna dependencia del venv.

- resumen.py: `w_geom_rojo_r14: 89/96 ok  media 42.28  mediana 42.00  mejor 37.45  fallas: n1/s3 n1/s33 n1/s36 n2/s21 n2/s40 n2/s45 n2/s48`
- Reloj total: 136.0 s. La doy por **válida**: carga máx 7.09, sin OOM/Killed/MemoryError, media_ms (COSTO) medio de 11.1 ms. El reloj está en línea con las otras tandas de carrera completa de 96 carreras (w_inicio_v1/v2/v3: 150.1/135.5/131.5 s).
- Fallas, según la última línea STOP:
  - Choques tempranos (salida / primera vuelta): n1/s3 (6.7 s, 1 giro), n1/s33 (3.5 s, 0 giros), n1/s36 (6.6 s, 1 giro), n2/s40 (7.4 s, 1 giro), n2/s45 (13.9 s, 2 giros).
  - n2/s21: choque a 38.7 s con los 12 giros hechos, o sea durante el estacionamiento.
  - n2/s48: el STOP dice race_finished (35.7 s), pero resumen.py la cuenta como falla. Revisa con qué criterio la marca (¿error en el estacionamiento?).
- Comparación: no tengo ninguna tanda anterior con esta misma configuración (VBAT 11.5 + cámara desde calibración + PARK_GEOM). Las w_inicio_v1-v3 son carreras completas, pero usan otro env (sin VBAT, con ROUTE_PLAN_ASYNC y sin calibración de cámara), así que no las comparo directamente.

Resultados completos (STOP y COSTO por carrera): `grok/resultados:resultados/w_geom_rojo_r14.md`.

Siguiente: para medir el efecto del parche GEOM/ROJO, manda la base (sin TWIN_FW_OVER ni ROUTE_PARK_GEOM_EXT) con este mismo env y las semillas s1-48. También conviene mirar las 5 salidas que chocan pronto, porque pesan más que la del estacionamiento.

> Nota (00:02 PST): todavía no pude subir `resultados/w_geom_rojo_r14.md` a grok/resultados, porque falló la revisión automática del push de ese archivo tan grande. Quedó en cola y se sube en la próxima pasada; las cifras de arriba son las finales. Mientras tanto, el detalle completo por carrera está en ~/.grokbot/respuestas/w_geom_rojo_r14_resultados.md en tu Mac.
