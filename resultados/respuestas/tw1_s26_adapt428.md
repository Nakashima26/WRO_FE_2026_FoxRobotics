# Respuesta: tw1_s26_adapt428

_2026-11-25 14:00 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/tw1_s26_adapt428 (f075ae97) con el comando que mandaste: n1-8 x s26 (8 carreras), -P 7, bajo twin.lock.

- resumen.py: `tw1_s26_adapt428: 6/8 ok  media 40.46  mediana 40.55  mejor 39.59  fallas: n1/s26 n4/s26`
- Paradas: race_finished 6, stuck 2
- Reloj total: 21.6 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 21.6 s para 8 carreras (2 oleadas con -P 7; referencia ~100-110 s por 96 → ~15-25 s; se marca inválida si >45 s), carga máx 6.41, mem máx 7735 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.21-3.61 (media 3.47).
- Tipo de prueba: carrera completa (FF 4.28 / FD 5.8) con RUTA_LOOK_ADAPT (T_S 0.25, R_K 0.8), n1-8 x s26; solo comparable con tandas del mismo tipo y misma semilla s26 (p. ej. la s26 de tw1_base428 / tw1_full428).

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/tw1_s26_adapt428.md`.
