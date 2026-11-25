# Respuesta: s20cal1_sep

_2026-11-25 12:12 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/s20cal1_sep (3a0c10ae) con el comando que mandaste: n1-8 x s20 (8 carreras), CCW, -P 7, bajo twin.lock (esperó su turno detrás de las tandas que ya estaban en cola).

- resumen.py: `s20cal1_sep: 0/8 ok  fallas: n1/s20 n2/s20 n3/s20 n4/s20 n5/s20 n6/s20 n7/s20 n8/s20`
- Paradas: race_finished 8
- Reloj total: 20.7 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 20.7 s para 8 carreras, carga máx 5.99, mem máx 4497 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.19-3.69 (media 3.34).
- Tipo de prueba: carrera completa CCW con parámetros de estacionamiento; solo comparable con tandas de ese mismo tipo y semilla s20.


Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/s20cal1_sep.md`.

Por qué resumen.py da 0/8 aunque las 8 terminan (race_finished, sin collisions): ok_t exige además quedar ≤ 5 mm fuera del cajón y sin ignored_contacts. Medí fuera_mm sobre los trace.csv de cada carrera: n1 35.9, n2 39.3, n3 51.8, n4 40.6, n5 38.5, n6 31.1, n7 38.0, n8 51.1 mm, o sea que el estacionamiento final queda 31-52 mm fuera en todas. Además n2 y n7 tienen ignored_contacts 'pared exterior' (t≈4.6-4.7 s, 38.5 y 42.1 mm). Tiempos totales 39.5-40.9 s. Mismo patrón 0/8 en s20cal1 (misma semilla y tipo de prueba), así que con este cambio el cajón no mejora.

Siguiente sugerido: revisar la maniobra final de estacionamiento (dónde para respecto al cajón) más que la vuelta; si quieres, mándame una variante y la corro igual.

