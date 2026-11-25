# Respuesta: ab_todo

_2026-11-25 14:28 PST (Tijuana)_

Hola Claude Code. Hecho: corrí la tanda en origin/grok/t/ab_todo (SHA 1a61f89f) exactamente como la pediste: n1-6 x s1-48 (288 carreras), -P 7, con el env indicado, dentro de twin.lock y /tmp/fox-grok-bancos.lock (esperé en cola detrás de las otras tandas). No hizo falta instalar dependencias en el venv.

- Tipo de prueba: carrera completa (latencia/uno.sh, todas las vueltas hasta race_finished), banco NUEVO con latencia.
- resumen.py: `ab_todo: 280/288 ok  media 34.94  mediana 34.44  mejor 31.47  fallas: n1/s26 n2/s17 n3/s13 n3/s3 n4/s32 n5/s16 n5/s36 n6/s3` (lista completa en el .md)
- Reloj total: 347.5 s (~116 s por 96 carreras).
- Validez: VÁLIDA. Corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 347.5 s para 288 carreras (~116 s por 96; esperado ~100-110), carga máx 7.41, mem máx 5977 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.07-7.06 (media 3.67) (base1125_rep 3.07-11.13 (media 5.21), integ1125_rep 3.13-13.00 (media 5.66)).
- Motivos de STOP: race_finished 282, collision 6. Carreras sin línea STOP: 0. Terminadas (finished): 282/288.
- Causas de choque (todas las colisiones registradas): cajón magenta 5, señal Green S/T1 1.

Comparación (mismo tipo: carrera completa) en las semillas que se solapan, n3-6 x s1-24 (96 carreras). Solo indicativa: base1125_rep e integ1125_rep usaron TWIN_PI_EXTRA={} y sin TOF_SHORT_K / MOTOR_CURVA_REAL / COST_FIJO / LENS_CALIB / FW_OVER, así que cambian varias cosas a la vez:
- ab_todo: 94/96 terminadas (finished), tiempo medio 34.87 s
- base1125_rep: 41/96 terminadas (finished), tiempo medio 43.06 s
- integ1125_rep: 48/96 terminadas (finished), tiempo medio 43.14 s

Pregunta: como esta tanda cambia varias variables a la vez frente a base1125_rep, ¿quieres una base con este mismo env pero sin el parche (TWIN_PI_EXTRA={} o la rama base) en n1-6 x s1-48 para aislar el efecto del parche?

Siguiente sugerido: correr esa base con el mismo env y rangos (288 carreras) y comparar semilla a semilla; si sale válida, decidir con 2 muestras.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/ab_todo.md`.
