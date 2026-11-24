# Respuesta: w_inicio_v3_rep

_2026-11-23 23:22 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_inicio_v3_rep (ff1c4a2; el árbol es idéntico a w_inicio_v3) con los parámetros que mandaste: 12 carreras (ruido n3-n8 x semillas s2 y s36), TWIN_MOTOR_REAL=1, ROUTE_PLAN_ASYNC, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7. Corrió sola bajo twin.lock y no faltó ninguna dependencia del venv.

- resumen.py: `w_inicio_v3_rep: 12/12 ok  media 41.51  mediana 41.53  mejor 38.49  fallas:`
- Las 12 carreras terminaron con race_finished y sin colisiones.
  - s2: 38.49 a 39.91 s.
  - s36: 43.15 a 44.96 s.
- En v3, n1/s2 chocó con el cajón magenta. Con los ruidos 3 a 8 la semilla 2 no repite el choque, así que parece un fallo raro que depende del ruido y no de la semilla.
- Reloj total: 19.2 s. La doy por **válida**: 0 procesos ajenos, carga máx 1.80, sin OOM/Killed, mean_pi_frame_ms entre 3.27 y 3.53, y media_ms (COSTO) 8.6.
- Tipo de prueba: carrera completa desde el cajón (banco nuevo con latencia). La media no se puede comparar con v1, v2 o v3 porque aquí solo hay 2 semillas.

Resultados completos (STOP y COSTO por carrera): `grok/resultados:resultados/w_inicio_v3_rep.md`.

Siguiente: si quieres más confianza sobre el fallo de cajón, puedo correr s2 con más ruidos (n9-n48). También puedo hacer v3 con semillas 49-96.
