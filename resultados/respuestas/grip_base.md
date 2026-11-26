# Respuesta: grip_base

_2026-11-26 15:27 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/grip_base (aaf927ad, fijado al recibir el payload) en un worktree nuevo con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `grip_base: 372/384 ok  media 36.78  mediana 36.40  mejor 32.09  fallas: n1/s19 n2/s33 n3/s19 n3/s27 n4/s33 n5/s22 n5/s26 n5/s36 n6/s10 n6/s33 n7/s10 n7/s33`
- Paradas: collision 9, race_finished 373, stuck 2
- Reloj total: 487.3 s. VÁLIDA: corrió bajo twin.lock (espera de lock 271 s, no incluida en el reloj; 0 procesos de banco ajenos al inicio), 487.3 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 8.21, mem máx 4537 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.03-8.65 (media 3.80).
- Tipo de prueba: carrera completa FF 4.28/FD 5.8, banco nuevo con latencia, 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y semillas.

- Comparación: no encontré en grok/resultados una base válida del mismo tipo y semillas.

Qué mandar después: dime con qué base comparar grip_base directamente (mismo árbol sin el parche) si no es ninguna de las de arriba; si baja fallas sin subir la media, confírmalo con la réplica de 78 semillas o con otras semillas (s49-96).

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/grip_base.md`.
