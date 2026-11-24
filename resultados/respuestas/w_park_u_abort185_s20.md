# Respuesta: w_park_u_abort185_s20

_2026-11-24 09:59 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_park_u_abort185_s20 (3ce6497) como la pediste: n1-4 x s20, PARK_UTURN_ABORT_PWM=185 sobre GEOM_EXT + GEOM_ROJO, ROUTE_PARK_GEOM_EXT=1, bajo twin.lock. No faltó ninguna dependencia del venv.

- resumen.py: `w_park_u_abort185_s20: 3/4 ok  media 37.49  mediana 37.80  mejor 36.46  fallas: n4/s20`
- Reloj total: 12.1 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx 5.10, mem máx 3650 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.47-3.83.
- Tipo de prueba: carrera completa con estacionamiento geométrico y U (mismo tipo que w_park_u_abort185).

Resultados por carrera (STOP):
- n1/s20: race_finished 37.802 s, sin choques, fuera del cajón 0 mm.
- n2/s20: race_finished 38.210 s, sin choques, fuera del cajón 0 mm.
- n3/s20: race_finished 36.456 s, sin choques, fuera del cajón 0 mm.
- n4/s20: race_finished 34.806 s, sin choques, pero queda 113.5 mm fuera del cajón (fuera_mm de optimizar_park.py), así que resumen.py la cuenta como falla.

Lectura: el choque con el cajón magenta de n2/s20 en w_park_u_abort185 (t≈22.6 s, a mitad de la carrera) no se repite: aquí n2/s20 termina limpio en 38.21 s, igual que en la repetición de control de la tanda anterior. Eso apoya que fue ruido de corrida. Pero aparece otro modo de falla en s20: n4 termina la carrera sin choques y es la más rápida (34.8 s, ~1.7-3.4 s menos que las demás), pero se detiene a 113.5 mm del cajón. El tiempo corto sugiere que estacionó corto o que la maniobra se cortó. Con 4 carreras no se puede decir si viene de ABORT_PWM=185 o si también pasaría sin el cambio. En s20, con abort185 y contando la tanda anterior: n1 terminó bien 2/2, n2 1 choque + 1 bien (+ control bien), n3 bien, n4 fuera del cajón.

Pregunta: ¿quieres que mire el trace.csv de n4/s20 (/tmp/lat/w_park_u_abort185_s20/n4/s20 en mi máquina) para ver en qué estado del estacionamiento se detuvo? Si me dices qué columna o log del firmware SIL marca el aborto DIRECTO→U, puedo confirmar si se disparó.

Siguiente sugerido: la misma tanda n1-4 x s20 sin PARK_UTURN_ABORT_PWM (base GEOM_EXT + GEOM_ROJO) para ver si n4/s20 también queda fuera del cajón sin el cambio. Después, abort185 en n1-2 x s1-24 contra w_park_u_uturn185_cen.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_park_u_abort185_s20.md`.
