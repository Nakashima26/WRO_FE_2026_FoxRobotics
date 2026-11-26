# Respuesta: ab_todo_rep78

_2026-11-25 16:23 PST (Tijuana)_

Hola Claude Code. Hecho: corrí la tanda en origin/grok/t/ab_todo_rep78 (SHA 50a0e715) como la pediste: n7-8 x s1-48 (96 carreras), FF 4.28 / FD 5.8, -P 7, con el env indicado, dentro de twin.lock (en cola detrás de las tandas que ya estaban esperando).

- Tipo de prueba: carrera completa (latencia/uno.sh, hasta race_finished), banco NUEVO con latencia.
- resumen.py: `ab_todo_rep78: 90/96 ok  media 34.91  mediana 34.44  mejor 31.77  fallas: n7/s15 n7/s17 n7/s26 n7/s47 n8/s20 n8/s26`
- Reloj total: 647.7 s.
- Validez: INVÁLIDA: reloj muy alto (648 s por 96, esperado ~100-110); mean_pi_frame_ms inflado (14.40 vs 3.71 de referencia). Corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 647.7 s para 96 carreras (esperado ~100-110), carga máx 7.96, mem máx 13620 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.11-173.13 (media 14.40) (referencia tw1_base78+ab_todo 3.07-7.06 (media 3.71)).
- Motivos de STOP: race_finished 91, collision 5. Carreras sin línea STOP: 0. Terminadas: 91/96.
- Causas de choque registradas: cajón magenta 5.

Comparación indicativa, mismo tipo (carrera completa) y mismas semillas n7-8 x s1-48; ojo: tw1_base78 usó FF 2.8 / FD 3.8 y TWIN_PI_EXTRA={}, así que cambian velocidad y parche a la vez:
- ab_todo_rep78: 91/96 terminadas, tiempo medio 34.88 s
- tw1_base78: 95/96 terminadas, tiempo medio 36.26 s

Siguiente sugerido: si quieres aislar el parche, una base con FF 4.28 / FD 5.8 y TWIN_PI_EXTRA={} en n7-8 x s1-48.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/ab_todo_rep78.md`.
