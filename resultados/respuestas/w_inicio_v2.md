# Respuesta: w_inicio_v2

_2026-11-23 23:01 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_inicio_v2 (a4fd005) con los parámetros que mandaste: 96 carreras (n1-2 x s1-48), TWIN_MOTOR_REAL=1, ROUTE_PLAN_ASYNC, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7. Corrió sola bajo twin.lock y no faltó ninguna dependencia del venv.

- resumen.py: `w_inicio_v2: 93/96 ok  media 40.47  mediana 40.17  mejor 35.29  fallas: n1/s26 n1/s45 n2/s35`
- Fallas:
  - n1/s26: collision "cajón magenta" en t=46.7, durante el estacionamiento final.
  - n2/s35: collision "cajón magenta" en t=41.6, durante el estacionamiento final.
  - n1/s45: **señal Red S/X2 en t=14.0 s**, con 2 curvas hechas.
- Reloj total: 135.5 s. La doy por **válida**, igual que v1 (150.1 s): 0 procesos ajenos, carga máx 6.93, sin OOM/Killed, mean_pi_frame_ms medio 3.73 (máx 6.85) y media_ms (COSTO) 9.6. El reloj alto se debe a que las carreras duran más (~40 s).
- Comparación con w_inicio_v1 (mismo tipo de prueba y mismo env: carrera completa desde el cajón, banco nuevo con latencia): v1 dio 90/96, media 40.79 y mejor 35.63; v2 da **93/96, media 40.47 y mejor 35.29**. En v1 había un choque con señal al salir (n1/s36, t=6.7) y en v2 ya no hay ninguno al inicio. Los fallos de cajón bajan de 5 a 2 y queda 1 choque con señal a media vuelta (n1/s45). Ojo: la diferencia es de solo 3 carreras sobre 96, y puede ser ruido.

Resultados completos (STOP y COSTO por carrera): `grok/resultados:resultados/w_inicio_v2.md`.

Siguiente: si quieres separar el efecto del parche, manda la base (mismo env sin parche) y la corro igual. También puedo sacar el fw_debug de n1/s45 (señal roja S/X2).
