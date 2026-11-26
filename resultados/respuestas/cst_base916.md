# Respuesta: cst_base916

_2026-11-25 22:17 PST (Tijuana)_

Hola Claude Code. Hecho: corrí la tanda en origin/grok/t/cst_base916 (SHA b0fdd0ee) como la pediste: n9-16 x s1-48 (384 carreras), FF 4.28 / FD 5.8, -P 7, con TWIN_PI_EXTRA={"ROUTE_MOTOR_CURVE_LIVE": false} y el resto del env indicado, dentro de twin.lock (en cola detrás de vis2_i950, vis2_i950h55 y vis2_n950).

- Tipo de prueba: carrera completa (latencia/uno.sh, hasta race_finished), banco NUEVO con latencia.
- resumen.py: `cst_base916: 364/384 ok  media 36.80  mediana 36.38  mejor 32.30  fallas: n10/s3 n11/s26 n11/s4 n12/s10 n12/s21 n12/s39 n13/s23 n13/s35 n13/s40 n14/s25 n14/s40 n14/s41 n14/s48 n15/s17 n15/s33 n15/s36 `
- Reloj total: 506.4 s.
- Validez: VÁLIDA. Corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 506.4 s para 384 carreras (esperado ~400-440), carga máx 7.71, mem máx 4834 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.10-7.83 (media 3.83) (referencia ab_todo+main8e5_base428 3.07-8.58 (media 3.91)).
- Motivos de STOP: race_finished 370, collision 11, stuck 2, max_time 1. Carreras sin línea STOP: 0. Terminadas: 370/384.
- Causas de choque registradas: cajón magenta 5, señal Red E/T2 1, señal Red W/T2 1, señal Green W/T2 1, señal Green S/T2 1, señal Green S/T1 1, señal Green N/T2 1.

Comparación: mismo tipo (carrera completa), mismas semillas n9-16, FF 4.28 / FD 5.8:
- cst_base916: 370/384 terminadas (de las n9-16 x s1-48 presentes), tiempo medio 36.80 s
- n9: salidas ya no están en el disco
- n9: salidas ya no están en el disco

Siguiente sugerido: usar cst_base916 como base n9-16 y correr el candidato (misma env, mismas semillas n9-16, FF 4.28 / FD 5.8) para compararlo contra esta tanda; si mejora sin subir fallas, completar con la réplica de 78 semillas según la regla de aceptación.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/cst_base916.md`.
