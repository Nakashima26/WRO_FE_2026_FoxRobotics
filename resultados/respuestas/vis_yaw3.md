# Respuesta: vis_yaw3

_2026-11-25 18:57 PST (Tijuana)_

Hola Claude Code. Hecho: corrí la tanda en origin/grok/t/vis_yaw3 (SHA de900350) como la pediste: n1-8 x s1-48 (384 carreras), FF 4.28 / FD 5.8, -P 7, con el env indicado (TWIN_PI_EXTRA con ROUTE_MOTOR_CURVE_LIVE=false y ROUTE_VIEW_YAW_MAX_DEG=3.0), dentro de twin.lock (en cola detrás de main8e5_nocs428, ab_todo_rep78, erick_todo, erick_main y erick_ekf).

- Tipo de prueba: carrera completa (latencia/uno.sh, hasta race_finished), banco NUEVO con latencia.
- resumen.py: `vis_yaw3: 372/384 ok  media 36.80  mediana 36.35  mejor 32.77  fallas: n1/s19 n1/s21 n3/s19 n3/s27 n3/s3 n4/s26 n5/s36 n6/s47 n7/s33 n7/s36 n7/s45 n8/s12`
- Reloj total: 490.4 s.
- Validez: VÁLIDA. Corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 490.4 s para 384 carreras (esperado ~400-440), carga máx 8.20, mem máx 4547 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.04-7.46 (media 3.75) (referencia ab_todo+main8e5_base428 3.07-8.58 (media 3.91)).
- Motivos de STOP: race_finished 374, collision 7, stuck 3. Carreras sin línea STOP: 0. Terminadas: 374/384.
- Causas de choque registradas: señal Green S/T1 3, cajón magenta 2, señal Red S/T1 1, señal Red S/T4 1.

Comparación indicativa, mismo tipo (carrera completa) mismas semillas y FF 4.28 / FD 5.8; ojo: main8e5_base428 usó TWIN_PI_EXTRA={}, erick_todo usó el TWIN_PI_EXTRA largo (latencias 0.12, CORNER_SIGN null) en n1-6, erick_ekf usó TWIN_PI_EXTRA solo con ROUTE_MOTOR_CURVE_LIVE=false (misma base sin el límite de yaw de vista), erick_main ver su archivo:
- vis_yaw3: 374/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.81 s
- main8e5_base428: 377/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.32 s
- erick_ekf: 376/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.77 s
- erick_main: 280/288 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.75 s
- erick_todo: 278/288 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 35.41 s

Siguiente sugerido: si fallas no suben y el tiempo medio baja frente a main8e5_base428 en n1-8, falta la réplica de 78 semillas para completar la regla de aceptación.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/vis_yaw3.md`.
