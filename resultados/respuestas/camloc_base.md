# Respuesta: camloc_base

_2026-11-26 15:19 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/camloc_base (560f3232, fijado al recibir el payload) en un worktree nuevo con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `camloc_base: 376/384 ok  media 37.00  mediana 36.49  mejor 32.22  fallas: n1/s39 n1/s45 n2/s20 n3/s3 n5/s36 n6/s23 n8/s29 n8/s3`
- Paradas: collision 4, race_finished 379, stuck 1
- Reloj total: 19093.2 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (espera de lock 36183 s, no incluida en el reloj; 0 procesos de banco ajenos al inicio), 19093.2 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 11.17, mem máx 13748 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.80-34.74 (media 5.26).
- Tipo de prueba: carrera completa FF 4.28/FD 5.8, banco nuevo con latencia, 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y semillas.

- Comparación: no encontré en grok/resultados una base válida del mismo tipo y semillas.

Pregunta / qué mandar después: esta es la base (sin LOC_CAM_WALL) para camloc_wall. Si camloc_wall no aparece arriba es porque seguía en la cola del twin.lock al terminar; compáralo en grok/resultados cuando salga (mismo árbol, mismas semillas). Si LOC_CAM_WALL mejora, manda una tanda de confirmación con otras semillas (p. ej. s49-96) para ambos; si no, dime qué variante de la localización por cámara probar.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/camloc_base.md`.
