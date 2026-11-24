# Respuesta: w_inicio_v1

_2026-11-23 22:51 PST (Tijuana)_

Hecho: tanda corrida en origin/grok/t/w_inicio_v1 (4c33beb), con los parámetros que mandaste (96 carreras, n1-2 x s1-48, TWIN_MOTOR_REAL=1, ROUTE_PLAN_ASYNC, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7). Corrió sola bajo twin.lock. No faltó ninguna dependencia del venv.

- resumen.py: `w_inicio_v1: 90/96 ok  media 40.79  mediana 40.47  mejor 35.63  fallas: n1/s20 n1/s3 n1/s36 n2/s32 n2/s4 n2/s42`
- Fallas: 5 de 6 son **collision "cajón magenta"** al final, durante el estacionamiento (n1/s20 t=35.4, n1/s3 t=43.3, n2/s32 t=41.1, n2/s4 t=42.1, n2/s42 t=37.7). La otra es **n1/s36 → señal Green S/T1 en t=6.7 s**, al salir.
- Reloj total: 150.1 s. Supera el umbral habitual, pero la doy por **válida**: 0 procesos ajenos, carga máx 6.57, sin OOM/Killed, mean_pi_frame_ms medio 3.92 y media_ms (COSTO) 10.0. El reloj sube porque las carreras son más largas (media 40.8 s frente a ~34 s en pfx100g/m0g).
- Tipo de prueba: carrera completa desde el cajón (salida, 3 vueltas, estacionamiento) en el banco nuevo con latencia. No encontré otra tanda de 96 carreras con este mismo env (MOTOR_REAL + plan asíncrono), así que no comparo tiempos.

Resultados completos (STOP y COSTO por carrera): `grok/resultados:resultados/w_inicio_v1.md`.

Siguiente: para medir el efecto del parche, manda la base (mismo env sin el parche w_inicio) y la corro igual. También puedo sacar el fw_debug de n1/s36 (choque con la señal al inicio, que es justo la zona que toca este parche).
