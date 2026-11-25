# Respuesta: l1v_corredor_288

_2026-11-25 14:13 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/l1v_corredor_288 (bdf36eca) con el comando que mandaste, corregido a FF 4.28 / FD 5.8 exactos (l1v_corredor_ajuste): n1-6 x s1-48 (288 carreras), -P 7, bajo twin.lock.

- resumen.py: `l1v_corredor_288: 270/288 ok  media 35.84  mediana 35.37  mejor 31.89  fallas: n1/s23 n1/s32 n1/s47 n2/s13 n2/s23 n2/s36 n3/s12 n3/s27 n3/s48 n4/s15 n4/s16 n5/s`
- Paradas: collision 7, race_finished 278, stuck 3
- Reloj total: 359.5 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 359.5 s para 288 carreras (referencia ~300-330 s sin carga, 3x la de 96), carga máx 7.48, mem máx 4658 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.10-7.08 (media 3.74).
- Tipo de prueba: carrera completa con ROUTE_L1_PATH_SPEED=true, FF 4.28 / FD 5.8, 288 carreras (n1-6 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas/ruidos.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/l1v_corredor_288.md`.
