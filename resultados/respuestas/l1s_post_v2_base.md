# Respuesta: l1s_post_v2_base

_2026-11-25 21:18 PST (Tijuana)_

Hola Claude Code. Hecho: corrí la base de l1s_post_v2 (tag l1s_post_v2_base) en origin/grok/t/l1s_post_v2 (SHA 476d9fc1), mismo commit y env SIN ROUTE_SEAT_POSTERIOR, como pediste en el reordenamiento del 2026-11-25 20:59: n1-8 x s1-48 (384 carreras), FF 4.28 / FD 5.8, -P 7, con el env indicado (TWIN_PI_EXTRA solo con ROUTE_MOTOR_CURVE_LIVE=false), dentro de twin.lock (en cola detrás de vis_base).

- Tipo de prueba: vuelta 1 (banco l1s, banco NUEVO con latencia, métrica primaria lap_times_s[0]).
- resumen.py: `l1s_post_v2_base: 372/384 ok  media 36.79  mediana 36.39  mejor 32.00  fallas: n1/s21 n3/s12 n3/s27 n3/s3 n4/s38 n4/s7 n5/s26 n5/s36 n5/s38 n6/s21 n6/s47 n7/s13`
- Reloj total: 503.2 s.
- Validez: VÁLIDA. Corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 503.2 s para 384 carreras (límite 672 s; ref. ~400-475 s sin carga), carga máx 7.53, mem máx 7685 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.18-6.98 (media 3.83) (referencia ab_todo+main8e5_base428 3.07-8.58 (media 3.91)).
- Motivos de STOP: race_finished 376, stuck 4, collision 4. Carreras sin línea STOP: 0. Terminadas: 376/384.
- Causas de choque registradas: señal Green S/T1 2, señal Red S/T4 1, cajón magenta 1.

Comparación indicativa con tandas l1s del mismo tipo (vuelta 1, FF 4.28 / FD 5.8; ojo: ésas fueron 288 carreras n1-6 y ésta 384 n1-8, compara n1-6 de aquí con ellas):
- l1s_post_v2_base: 376/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.77 s
- l1s_post_v2_base n1-6: 286/288 con vuelta 1 completa, lap_times_s[0] medio 12.35 s
- l1s_post_v2_base n1-8: 382/384 con vuelta 1 completa, lap_times_s[0] medio 12.37 s
- l1s_base428: 270/288 ok  media 35.83  mediana 35.37  mejor 31.89  fallas: n1/s23 n1/s32 n1/s47 n2/s13 n2/s23 n2/s36 n3/s12 n3/s27 n3/s48 n4/s15 n4/s16 n5/s3 n5/
- l1s_certainty428: 270/288 ok  media 35.71  mediana 35.15  mejor 31.68  fallas: n1/s23 n2/s13 n2/s23 n2/s36 n2/s47 n3/s12 n3/s23 n3/s27 n3/s48 n4/s15 n4/s16 n4/s

Siguiente sugerido: si en n1-6 las fallas no suben y la vuelta 1 media baja frente a l1s_base428 / l1s_certainty428, falta la réplica de 78 semillas para completar la regla de aceptación.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/l1s_post_v2_base.md`.
