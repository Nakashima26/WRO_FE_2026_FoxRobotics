# Respuesta: w_geom_rojo_r16_wstop

_2026-11-24 00:35 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_geom_rojo_r16_wstop (7709c8b) como la pediste: 12 carreras (n1-2 x s5 s32 s35 s36 s16 s3), TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, cámara desde calibración, TWIN_FW_OVER PARK_GEOM_EXT=1 + PARK_GEOM_ROJO=1 + PARK_GEOM_WSTOP=262, TWIN_PI_EXTRA ROUTE_PARK_GEOM_EXT=1, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7. Corrió sola bajo twin.lock y no faltó ninguna dependencia del venv.

- resumen.py: `w_geom_rojo_r16_wstop: 10/12 ok  media 45.58  mediana 45.99  mejor 43.43  fallas: n1/s36 n2/s36`
- Reloj total: 29.5 s. La doy por **válida**: carga máx 2.77, sin OOM/Killed, media_ms (COSTO) de 9.5-12.2 ms en las 10 carreras completas.
- Tipo de prueba: carrera completa con estacionamiento (mismo tipo y configuración que w_geom_rojo_r14 y w_geom_rojo_r15b, más WSTOP=262).

Comparación:
- **Fallan n1/s36 y n2/s36**: el mismo choque temprano con la señal verde de S/T1 a ~6.6 s (1 giro), antes del estacionamiento. No lo atribuyo a WSTOP. Ahora chocan los dos ruidos; en r14 y r15b chocaba solo uno.
- Frente a r15b, en las 6 carreras que terminan en ambas: +2.70 s en total. Frente a r14, en 9 carreras: -0.88 s.
- n1/s3, que en r14 chocaba temprano, ahora termina (47.83 s).
- Tabla por carrera: `grok/resultados:resultados/w_geom_rojo_r16_wstop.md`.

Pregunta: ¿qué esperabas que cambiara con WSTOP=262 (tiempo de estacionamiento o algún caso de fallo concreto)? En esta muestra no veo una mejora clara frente a r15b.

Siguiente: el choque de s36 a ~6.6 s no tiene que ver con el estacionamiento y ya sale en las tres tandas. Si quieres aislarlo, manda un lote solo de s36 con más ruidos (n1-8). Si WSTOP es candidato, manda la tanda completa n1-2 x s1-48 con este env para compararla 1 a 1 contra r14 (89/96, media 42.28).
