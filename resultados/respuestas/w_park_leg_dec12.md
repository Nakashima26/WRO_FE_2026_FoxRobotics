# Respuesta: w_park_leg_dec12

_2026-11-23 23:13 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_park_leg_dec12 (0080081, parche PARK_SINU_DEC_ANG_DEG 14 -> 12) con los parámetros que mandaste: 26 carreras (n1-2 x s1 2 3 5 8 12 13 16 19 20 26 32 37), TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, ROUTE_PLAN_ASYNC, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7. Corrió sola bajo twin.lock (0 procesos ajenos) y no faltó ninguna dependencia del venv.

- resumen.py: `w_park_leg_dec12: 26/26 ok  media 42.97  mediana 43.17  mejor 37.12  fallas:`
- Reloj total: 42.0 s. La doy por **válida**: carga máx 4.96, sin OOM/Killed/MemoryError y media_ms (COSTO) medio de 9.7 ms.
- Comparación con w_park_leg_recover (mismo tipo y mismo env, 24 semillas sin la s13): recover dio 19/24 y media 42.55 (fallas n1/s3 n1/s5 n2/s3 n2/s5 n2/s19). dec12 en esas mismas 24 da **24/24**, media 43.06 y mejor 37.12. Gana unos 0.5 s de media pero no tiene fallas. Ojo: los dos parches son distintos y la muestra es chica.

Resultados completos (STOP y COSTO por carrera): `grok/resultados:resultados/w_park_leg_dec12.md`.

Siguiente: si quieres confirmar, manda la tanda con más semillas (por ejemplo s1-48) con este mismo parche, y la base sin parche con el mismo env.
