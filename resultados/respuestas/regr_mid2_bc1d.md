# Respuesta: regr_mid2_bc1d

_2026-11-26 00:35 PST (Tijuana)_

Hola Claude Code. Hecho: corrí la tanda en origin/grok/t/regr_mid2_bc1d (SHA c3f20afa) como la pediste: n1-8 x s1-48 (384 carreras), FF 4.28 / FD 5.8, -P 7, con TWIN_PI_EXTRA={"ROUTE_MOTOR_CURVE_LIVE": false} y el resto del env indicado, dentro de twin.lock (en cola detrás de las tandas que ya estaban esperando el lock).

- Tipo de prueba: carrera completa (latencia/uno.sh, hasta race_finished), banco NUEVO con latencia.
- resumen.py: `regr_mid2_bc1d: 376/384 ok  media 36.31  mediana 35.93  mejor 32.03  fallas: n2/s16 n3/s19 n3/s26 n3/s27 n4/s26 n5/s35 n5/s36 n7/s10`
- Reloj total: 515.3 s.
- Validez: VÁLIDA. Corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 515.3 s para 384 carreras (esperado ~400-440), carga máx 7.88, mem máx 4481 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.22-7.75 (media 3.98) (referencia ab_todo+main8e5_base428+cst_base916 n/d).
- Motivos de STOP: race_finished 377, collision 7. Carreras sin línea STOP: 0. Terminadas: 377/384.
- Causas de choque registradas: cajón magenta 3, señal Red S/T1 1, señal Red S/T4 1, señal Green S/T1 1, señal Red W/T2 1.

Comparación indicativa, mismo tipo (carrera completa), mismas semillas y FF 4.28 / FD 5.8 (solo las tandas regr_* cuyas salidas estén en el disco al terminar; si falta alguna, aún no corrió):
- regr_mid2_bc1d: 377/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.32 s
- regr_base7a: 313/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 36.36 s
- regr_mid1_0fca: salidas ya no están en el disco
- regr_main: salidas ya no están en el disco
- regr_main_fwrev: 380/384 terminadas (de las n1-8 x s1-48 presentes), tiempo medio 37.22 s

Siguiente sugerido: regr_mid2_bc1d queda igual o mejor que regr_base7a en terminadas y tiempo medio: la regresión estaría después de bc1d (entre mid2 y main).

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/regr_mid2_bc1d.md`.
