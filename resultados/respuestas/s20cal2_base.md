# Respuesta: s20cal2_base

_2026-11-25 12:25 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/s20cal2_base (b2779476) con el comando que mandaste: n1-8 x s20 (8 carreras), CCW, -P 7, bajo twin.lock.

- resumen.py: `s20cal2_base: 0/8 ok  fallas: n1/s20 n2/s20 n3/s20 n4/s20 n5/s20 n6/s20 n7/s20 n8/s20`
- Paradas: race_finished 8
- Reloj total: 21.6 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 21.6 s para 8 carreras, carga máx 7.09, mem máx 4479 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.28-3.67 (media 3.39).
- Tipo de prueba: carrera completa CCW con parámetros de estacionamiento; solo comparable con tandas de ese mismo tipo y semilla s20 (misma config base que s20cal1, sin TWIN_SERVO_TAU).

Resultado y esta respuesta subidos a grok/resultados (y copiados a ~/.grokbot/respuestas/ en la Mac).

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/s20cal2_base.md`.

## Nota

Las 8 carreras terminaron (race_finished 8/8), pero resumen.py cuenta 0/8 ok porque su criterio de ok exige parada normal sin choques y quedar dentro del cajón de estacionamiento (<= 5 mm fuera); igual que en s20cal1 y s20cal1_tau16 (también 0/8). total_time_s medio: 40.15 s (s20cal1: 39.74 s, misma prueba y semilla).

Siguiente: si quieres que cuente como ok, revisa por qué ninguna carrera queda dentro del cajón (stop_reason/estacionamiento) o mándame la variante a probar.
