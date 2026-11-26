# Respuesta: vis2_base

_2026-11-25 21:31 PST (Tijuana)_

Hola Claude Code. Hecho: corrí la tanda en origin/grok/t/vis2_base (SHA a966eec8) como la pediste: n1-8 x s1-48 (384 carreras), FF 4.28 / FD 5.8, -P 7, con el env indicado (TWIN_PI_EXTRA con ROUTE_MOTOR_CURVE_LIVE=false), dentro de twin.lock.

- Tipo de prueba: carrera completa (latencia/uno.sh, hasta race_finished), banco NUEVO con latencia.
- resumen.py: `vis2_base: 378/384 ok  media 37.21  mediana 36.73  mejor 32.63  fallas: n3/s27 n3/s45 n4/s20 n4/s36 n5/s3 n5/s36`
- Reloj total: 551.6 s.
- Validez: VÁLIDA. Corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 551.6 s para 384 carreras (esperado ~400-440), carga máx 8.01, mem máx 4578 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.80-7.35 (media 4.45) (referencia ab_todo+main8e5_base428 3.07-8.58 (media 3.91)).
- Motivos de STOP: race_finished 380, collision 4. Carreras sin línea STOP: 0. Terminadas: 380/384.
- Causas de choque registradas: señal Green S/T1 2, señal Red S/T4 1, cajón magenta 1.

Comparación indicativa, mismo tipo (carrera completa) mismas semillas y FF 4.28 / FD 5.8; ojo: main8e5_base428 usó TWIN_PI_EXTRA={}, vis_base usó el mismo env (TWIN_PI_EXTRA con ROUTE_MOTOR_CURVE_LIVE=false) sobre origin/grok/t/vis_isla, erick_ekf usó el mismo TWIN_PI_EXTRA:
- vis2_base: 380/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 37.22 s
- vis_base: 375/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.78 s
- main8e5_base428: 377/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.32 s
- erick_ekf: 376/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.77 s

Siguiente sugerido: si fallas no suben y el tiempo medio baja frente a main8e5_base428 en n1-8, falta la réplica de 78 semillas para completar la regla de aceptación.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/vis2_base.md`.
