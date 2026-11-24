# Respuesta: w_park_u_base_s20

_2026-11-24 10:07 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_park_u_base_s20 (e0bbb78) como la pediste: n1-4 x s20, base GEOM_EXT + GEOM_ROJO (sin PARK_UTURN_ABORT_PWM), ROUTE_PARK_GEOM_EXT=1, bajo twin.lock. No faltó ninguna dependencia del venv.

- resumen.py: `w_park_u_base_s20: 4/4 ok  media 37.49  mediana 37.48  mejor 36.89  fallas:`
- Reloj total: 12.3 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx 0.95, mem máx 3697 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.45-3.71.
- Tipo de prueba: carrera completa con estacionamiento geométrico y U (mismo tipo que w_park_u_abort185_s20).

Resultados por carrera (STOP):
- n1/s20: race_finished 37.306 s, sin choques.
- n2/s20: race_finished 36.893 s, sin choques.
- n3/s20: race_finished 37.651 s, sin choques.
- n4/s20: race_finished 38.125 s, sin choques, dentro del cajón (resumen.py no la marca como falla).

Comparación con w_park_u_abort185_s20 (mismo tipo, mismas 4 semillas): base 4/4 ok, media 37.49; abort185 3/4 ok, media 37.49, con n4/s20 a 113.5 mm fuera del cajón (34.806 s, reverse_motor_s 1.379). Aquí, sin el cambio, n4/s20 estaciona bien en 38.125 s con reverse_motor_s 3.781. La diferencia de ~3.3 s y de ~2.4 s de reversa en n4 apunta a que, con ABORT_PWM=185, la maniobra de n4 se cortó o tomó otra rama, y no a ruido. Con una sola muestra, es indicio, no prueba.

Ojo con el ruido: en w_park_u_base (tanda vieja, misma config base) n1/s20 chocó con el cajón magenta a t≈33.9 s; hoy n1/s20 termina limpio en 37.306 s. O sea, s20 tiene variación entre corridas incluso sin cambios.

Pregunta: ¿quieres que compare el trace.csv de n4/s20 entre esta tanda y w_park_u_abort185_s20 (ambos siguen en /tmp/lat en mi máquina) para ver dónde se separan los estados del estacionamiento?

Siguiente sugerido: repetir n4/s20 con abort185 2-3 veces (o n1-4 x s20 con abort185 otra vez) para confirmar que el fuera-de-cajón es reproducible antes de descartar ABORT_PWM=185.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_park_u_base_s20.md`.
