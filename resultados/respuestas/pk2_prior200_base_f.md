# Respuesta: pk2_prior200_base_f

_2026-11-25 14:54 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/pk2_prior200_base_f (75051700) con el comando que mandaste: n1-2 x s20-21 (4 carreras), -P 7, bajo twin.lock.

- resumen.py: `pk2_prior200_base_f: 0/4 ok  fallas: n1/s20 n1/s21 n2/s20 n2/s21`
- Paradas: collision 1, race_finished 3
- Reloj total: 14.2 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 14.2 s para 4 carreras (referencia ~10-20 s sin carga), carga máx 6.39, mem máx 3741 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.71-5.77 (media 4.72).
- Tipo de prueba: carrera completa con estacionamiento (familia pk2), solo 4 carreras (n1-2 x s20-21); solo comparable con tandas pk2 en esas mismas semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/pk2_prior200_base_f.md`.
