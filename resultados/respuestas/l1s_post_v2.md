# Respuesta: l1s_post_v2

_2026-11-25 19:06 PST (Tijuana)_

Hola Claude Code. Hecho: corrí la tanda en origin/grok/t/l1s_post_v2 (SHA 476d9fc1) como la pediste: n1-8 x s1-48 (384 carreras), FF 4.28 / FD 5.8, -P 7, con el env indicado (TWIN_PI_EXTRA con ROUTE_MOTOR_CURVE_LIVE=false y ROUTE_SEAT_POSTERIOR=true), dentro de twin.lock (en cola detrás de ab_todo_rep78, erick_todo, erick_main, erick_ekf, vis_ambos, vis_isla y vis_yaw3).

- Tipo de prueba: vuelta 1 (banco l1s, banco NUEVO con latencia, métrica primaria lap_times_s[0]).
- resumen.py: `l1s_post_v2: 363/384 ok  media 36.63  mediana 36.09  mejor 32.25  fallas: n1/s23 n2/s23 n3/s12 n3/s23 n3/s27 n3/s28 n3/s3 n4/s23 n4/s38 n5/s23 n5/s26 n5/s36 n5/s38 n5/s40 n5/s44 n5/s45 n6/s23 n7/s13 n`
- Reloj total: 496.3 s.
- Validez: VÁLIDA. Corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 496.3 s para 384 carreras (límite 672 s; ref. ~400-475 s sin carga), carga máx 7.69, mem máx 4577 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.24-7.45 (media 3.94) (referencia ab_todo+main8e5_base428 3.07-8.58 (media 3.91)).
- Motivos de STOP: race_finished 370, collision 13, stuck 1. Carreras sin línea STOP: 0. Terminadas: 370/384.
- Causas de choque registradas: señal Green W/T1 7, señal Green S/T1 2, cajón magenta 2, señal Red S/T4 1, señal Red N/T4 1.

Comparación indicativa con tandas l1s del mismo tipo (vuelta 1, FF 4.28 / FD 5.8; ojo: ésas fueron 288 carreras n1-6 y ésta 384 n1-8, compara n1-6 de aquí con ellas):
- l1s_post_v2: 370/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.62 s
- l1s_post_v2 n1-6: 281/288 con vuelta 1 completa, lap_times_s[0] medio 12.24 s
- l1s_post_v2 n1-8: 375/384 con vuelta 1 completa, lap_times_s[0] medio 12.27 s
- l1s_base428: 270/288 ok  media 35.83  mediana 35.37  mejor 31.89  fallas: n1/s23 n1/s32 n1/s47 n2/s13 n2/s23 n2/s36 n3/s12 n3/s27 n3/s48 n4/s15 n4/s16 n5/s3 n5/
- l1s_certainty428: 270/288 ok  media 35.71  mediana 35.15  mejor 31.68  fallas: n1/s23 n2/s13 n2/s23 n2/s36 n2/s47 n3/s12 n3/s23 n3/s27 n3/s48 n4/s15 n4/s16 n4/s

Siguiente sugerido: si en n1-6 las fallas no suben y la vuelta 1 media baja frente a l1s_base428 / l1s_certainty428, falta la réplica de 78 semillas para completar la regla de aceptación.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/l1s_post_v2.md`.
