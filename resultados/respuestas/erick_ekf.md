# Respuesta: erick_ekf

_2026-11-25 18:33 PST (Tijuana)_

Hola Claude Code. Hecho: corrí la tanda en origin/grok/t/erick_ekf (SHA 9e36bccc) como la pediste: n1-8 x s1-48 (384 carreras), FF 4.28 / FD 5.8, -P 7, con el env indicado (TWIN_PI_EXTRA solo con ROUTE_MOTOR_CURVE_LIVE=false), dentro de twin.lock (en cola detrás de main8e5_base428, main8e5_nocs428, ab_todo_rep78, erick_todo y erick_main).

- Tipo de prueba: carrera completa (latencia/uno.sh, hasta race_finished), banco NUEVO con latencia.
- resumen.py: `erick_ekf: 372/384 ok  media 36.79  mediana 36.39  mejor 32.00  fallas: n1/s21 n3/s12 n3/s27 n3/s3 n4/s38 n4/s7 n5/s26 n5/s36 n5/s38 n6/s21 n6/s47 n7/s13`
- Reloj total: 497.4 s.
- Validez: VÁLIDA. Corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 497.4 s para 384 carreras (esperado ~400-440), carga máx 7.96, mem máx 6088 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.14-6.90 (media 3.78) (referencia ab_todo+main8e5_base428 3.07-8.58 (media 3.91)).
- Motivos de STOP: race_finished 376, stuck 4, collision 4. Carreras sin línea STOP: 0. Terminadas: 376/384.
- Causas de choque registradas: señal Green S/T1 2, señal Red S/T4 1, cajón magenta 1.

Comparación indicativa, mismo tipo (carrera completa) mismas semillas y FF 4.28 / FD 5.8; ojo: main8e5_base428 usó TWIN_PI_EXTRA={}, erick_todo usó el TWIN_PI_EXTRA largo (latencias 0.12, CORNER_SIGN null) en n1-6, erick_main ver su archivo:
- erick_ekf: 376/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.77 s
- main8e5_base428: 377/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.32 s
- erick_todo: 278/288 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 35.41 s
- erick_main: 280/288 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.75 s

Siguiente sugerido: si fallas no suben y el tiempo medio baja frente a main8e5_base428 en n1-8, falta la réplica de 78 semillas para completar la regla de aceptación.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/erick_ekf.md`.
