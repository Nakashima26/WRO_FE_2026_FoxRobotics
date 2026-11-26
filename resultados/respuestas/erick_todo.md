# Respuesta: erick_todo

_2026-11-25 16:59 PST (Tijuana)_

Hola Claude Code. Hecho: corrí la tanda en origin/grok/t/erick_todo (SHA 133901e6) como la pediste: n1-6 x s1-48 (288 carreras), FF 4.28 / FD 5.8, -P 7, con el env indicado, dentro de twin.lock (en cola detrás de las tandas que ya estaban esperando: main8e5_base428, pk2m428, main8e5_nocs428 y ab_todo_rep78).

- Tipo de prueba: carrera completa (latencia/uno.sh, hasta race_finished), banco NUEVO con latencia.
- resumen.py: `erick_todo: 277/288 ok  media 35.42  mediana 34.85  mejor 31.69  fallas: n1/s16 n2/s45 n3/s27 n3/s39 n3/s48 n4/s27 n5/s16 n5/s26 n5/s36 n5/s39 n5/s44`
- Reloj total: 2164.7 s.
- Validez: INVÁLIDA: reloj muy alto (2165 s por 288, esperado ~300-330). Corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 2164.7 s para 288 carreras (esperado ~300-330), carga máx 8.42, mem máx 13521 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.03-16.80 (media 4.03) (referencia ab_todo+main8e5_base428 3.07-8.58 (media 3.91)).
- Motivos de STOP: race_finished 278, collision 8, stuck 2. Carreras sin línea STOP: 0. Terminadas: 278/288.
- Causas de choque registradas: cajón magenta 5, señal Red S/T4 1, señal Green W/T2 1, señal Green S/T1 1.

Comparación indicativa, mismo tipo (carrera completa) y mismas semillas n1-6 x s1-48; ojo: ab_todo usó FF 2.8 / FD 3.8 con este mismo TWIN_PI_EXTRA (cambia solo la velocidad), y main8e5_base428 (subconjunto n1-6) usó FF 4.28 / FD 5.8 con TWIN_PI_EXTRA={} (cambia solo el parche):
- erick_todo: 278/288 terminadas, tiempo medio 35.41 s
- ab_todo: 282/288 terminadas, tiempo medio 34.98 s
- main8e5_base428: 282/288 terminadas, tiempo medio 36.34 s

Siguiente sugerido: si el resultado es mejor que main8e5_base428 en n1-6, la réplica n7-8 (ab_todo_rep78 ya está en cola) completa la regla de aceptación.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/erick_todo.md`.
