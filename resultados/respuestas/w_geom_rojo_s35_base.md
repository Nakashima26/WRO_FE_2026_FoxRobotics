# Respuesta: w_geom_rojo_s35_base

_2026-11-24 00:55 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_geom_rojo_s35_base (2f23448: commit vacío sobre 094f93c) tal como la pediste: 6 carreras (n1-2 x s35 s32 s16), TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, cámara desde calibración, TWIN_FW_OVER PARK_GEOM_EXT=1 + PARK_GEOM_ROJO=1 (sin WSTOP), TWIN_PI_EXTRA ROUTE_PARK_GEOM_EXT=1, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7. Corrió sola bajo twin.lock y no faltó ninguna dependencia del venv.

- resumen.py: `w_geom_rojo_s35_base: 5/6 ok  media 43.85  mediana 43.46  mejor 42.83  fallas: n1/s35`
- Reloj total: 14.5 s. La doy por **válida**: carga máx 1.46, sin OOM/Killed.
- Tipo de prueba: carrera completa con estacionamiento geométrico, el mismo que r14/r15b/r16_wstop.

Hallazgos:
- **Falla n1/s35**: choca con el cajón magenta a t=46.44 s, ya con los 12 giros hechos, o sea en el estacionamiento. Con WSTOP=262 (r16_wstop, misma base) esa carrera terminó en 46.169 s. En r15b y r14 también terminaba. En esta muestra, WSTOP=262 evita este choque de estacionamiento.
- n2/s35 termina (45.782 s, casi igual que en r16_wstop: 45.821).
- En las carreras que terminan en ambas: frente a r16_wstop, 5 carreras, -2.44 s en total; frente a r15b, 3 carreras, -0.73 s; frente a r14, 5 carreras, -3.78 s. Las diferencias son pequeñas y la muestra es corta.
- Tabla por carrera: `grok/resultados:resultados/w_geom_rojo_s35_base.md`.

Siguiente: para confirmar que n1/s35 es un caso real del estacionamiento sin WSTOP y no ruido, manda s35 con más ruidos (n1-8), con y sin PARK_GEOM_WSTOP=262, en la misma base 094f93c.
