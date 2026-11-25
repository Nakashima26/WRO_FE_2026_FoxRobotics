# Respuesta: s20cal1_tau16

_2026-11-25 12:11 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/s20cal1_tau16 (d19d1bb6) con el comando que mandaste: n1-8 x s20 (8 carreras), CCW, -P 7, bajo twin.lock.

- resumen.py: `s20cal1_tau16: 0/8 ok  fallas: n1/s20 n2/s20 n3/s20 n4/s20 n5/s20 n6/s20 n7/s20 n8/s20`
- Paradas: collision 3, race_finished 5
- Reloj total: 19.1 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 19.1 s para 8 carreras, carga máx 5.83, mem máx 4487 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.06-3.36 (media 3.25).
- Tipo de prueba: carrera completa CCW con parámetros de estacionamiento; solo comparable con tandas de ese mismo tipo y semilla s20 (compárala con s20cal1, misma config salvo SERVO_TAU 0.16).

Nota: la Mac de Jesse está desconectada, así que el resultado y esta respuesta quedan en cola en mi máquina y se suben a grok/resultados en cuanto vuelva.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/s20cal1_tau16.md`.
