# Respuesta: w_inicio_v3

_2026-11-23 23:17 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_inicio_v3 (14accc4) con los parámetros que mandaste: 96 carreras (n1-2 x s1-48), TWIN_MOTOR_REAL=1, ROUTE_PLAN_ASYNC, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7. Corrió sola bajo twin.lock y no faltó ninguna dependencia del venv.

- resumen.py: `w_inicio_v3: 95/96 ok  media 40.57  mediana 40.49  mejor 35.37  fallas: n1/s2`
- Única falla: n1/s2, collision "cajón magenta" en t=37.9 s, con 12 curvas hechas. Fue en el estacionamiento final, el mismo tipo de fallo que tuvieron s26 y s35 en v2.
- Reloj total: 131.5 s. La doy por **válida**: 0 procesos ajenos, carga máx 7.15, sin OOM/Killed, mean_pi_frame_ms medio 3.62 (máx 5.05) y media_ms (COSTO) 9.3. El reloj es parecido al de v2 (135.5 s) y v1 (150.1 s), porque las carreras son largas (~40 s).
- Comparación con el mismo tipo de prueba y el mismo env (carrera completa desde el cajón, banco nuevo con latencia):
  - v1: 90/96, media 40.79, mejor 35.63.
  - v2: 93/96, media 40.47, mejor 35.29.
  - v3: **95/96, media 40.57, mejor 35.37**.
- v3 no tiene choques con señales (v2 tenía n1/s45, señal roja a media vuelta) y solo le queda 1 fallo de cajón (v2 tenía 2). La media es 0.1 s peor que en v2, lo cual cae dentro del ruido. Ojo: entre v2 y v3 hay solo 2 carreras de diferencia sobre 96.

Resultados completos (STOP y COSTO por carrera): `grok/resultados:resultados/w_inicio_v3.md`.

Siguiente: si quieres confirmar la mejora frente a v2, puedo correr semillas 49-96 con v2 y v3. También puedo sacar el fw_debug de n1/s2 (choque con el cajón al estacionar).
