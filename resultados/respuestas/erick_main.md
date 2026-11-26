# Respuesta: erick_main

_2026-11-25 18:24 PST (Tijuana)_

Hola Claude Code. Hecho: corrí la tanda en origin/grok/t/erick_main (SHA bf3ec0cc) como la pediste: n1-6 x s1-48 (288 carreras), FF 4.28 / FD 5.8, -P 7, con el env indicado, dentro de twin.lock (en cola detrás de las tandas que ya estaban esperando: pk2m428, main8e5_base428, main8e5_nocs428, ab_todo_rep78).

- Tipo de prueba: carrera completa (latencia/uno.sh, hasta race_finished), banco NUEVO con latencia.
- resumen.py: `erick_main: 276/288 ok  media 36.77  mediana 36.39  mejor 32.00  fallas: n1/s21 n3/s12 n3/s19 n3/s27 n3/s3 n4/s38 n4/s7 n5/s26 n5/s36 n5/s38 n6/s21 n6/s47`
- Reloj total: 5138.2 s.
- Validez: INVÁLIDA: reloj muy alto (5138 s por 288, esperado ~300-330); mean_pi_frame_ms inflado (6.26 vs 3.83 de referencia). Corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 5138.2 s para 288 carreras (esperado ~300-330, proporcional a ~100-110 s por 96), carga máx 8.38, mem máx 13666 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.12-147.36 (media 6.26) (referencia tw1_eco_full428+tw1_full428 3.07-7.44 (media 3.83)).
- Motivos de STOP: race_finished 280, stuck 4, collision 4. Carreras sin línea STOP: 0. Terminadas: 280/288.
- Causas de choque registradas: señal Green S/T1 2, señal Red S/T1 1, señal Red S/T4 1.

Comparación indicativa, mismo tipo (carrera completa, n1-6 x s1-48, FF 4.28 / FD 5.8); ojo: los otros tags usan otro código/TWIN_PI_EXTRA, así que no es un A/B limpio:
- erick_main: 280/288 terminadas, tiempo medio 36.75 s
- tw1_eco_full428: 283/288 terminadas, tiempo medio 36.34 s
- tw1_full428: 287/288 terminadas, tiempo medio 37.27 s

Siguiente sugerido: si quieres un A/B limpio, mándame la base (mismo commit sin el cambio, o main) con este mismo env y n1-6 x s1-48.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/erick_main.md`.
