# Respuesta: w_park_u_abort185_s5

_2026-11-24 10:04 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_park_u_abort185_s5 (f7de10b) como la pediste: n1-4 x s5, PARK_UTURN_ABORT_PWM=185 sobre GEOM_EXT + GEOM_ROJO, ROUTE_PARK_GEOM_EXT=1, bajo twin.lock. No faltó ninguna dependencia del venv.

- resumen.py: `w_park_u_abort185_s5: 4/4 ok  media 46.03  mediana 45.93  mejor 45.53  fallas:`
- Reloj total: 13.2 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx 0.58, mem máx 3754 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.36-3.51.
- Tipo de prueba: carrera completa con estacionamiento geométrico y U (mismo tipo que w_park_u_abort185_cen y w_park_u_abort185_s20).

Resultados por carrera (STOP), todas race_finished y sin choques:
- n1/s5: 45.532 s
- n2/s5: 45.899 s
- n3/s5: 45.959 s
- n4/s5: 46.716 s

Lectura: el choque con el cajón magenta de n1/s5 en w_park_u_abort185_cen (t≈45.6 s, ya en el estacionamiento) no se repite. Aquí n1/s5 termina limpio en 45.53 s y las otras 3 repeticiones también terminan bien y dentro del cajón (resumen.py: 4/4). Contando la tanda _cen, s5 con abort185 queda en 5 bien de 6 (n1 1 choque y 1 bien). Los tiempos (45.5-46.7 s) están en el mismo rango que s5 en base_cen (46.43 / 45.99) y uturn185_cen (45.56 / 46.48), así que en s5 abort185 no cuesta tiempo visible. Esto apoya que el choque de n1/s5 fue ruido de corrida, igual que n2/s20. Lo que sigue abierto es n4/s20, que quedó 113.5 mm fuera del cajón en w_park_u_abort185_s20.

Siguiente sugerido: la misma tanda n1-4 x s20 sin PARK_UTURN_ABORT_PWM (base GEOM_EXT + GEOM_ROJO) para ver si n4/s20 queda fuera del cajón también sin el cambio. Si sale bien, abort185 se ve listo para una tanda amplia (p. ej. n1-2 x s1-24 o más repeticiones).

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_park_u_abort185_s5.md`.
