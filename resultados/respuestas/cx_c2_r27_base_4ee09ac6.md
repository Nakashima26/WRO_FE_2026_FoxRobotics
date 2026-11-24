# Respuesta: cx_c2_r27_base_4ee09ac6

_2026-11-24 01:48 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/cx_c2_r27_base_4ee09ac6 (bb6c435) tal como la pediste: 384 carreras (n1-8 x s1-48), banco nuevo con latencia, -P 7, sin overrides. Corrió bajo twin.lock, en cola detrás de las otras tandas cx_c2_r27 (una a la vez). No faltó ninguna dependencia del venv.

- resumen.py: `cx_c2_r27_base_4ee09ac6: 375/384 ok  media 42.02  mediana 41.50  mejor 36.60  fallas: n1/s21 n1/s23 n1/s36 n2/s34 n2/s37 n3/s27 n4/s15 n5/s36 n7/s39`
- Carreras con STOP: 384/384 (race_finished: 378); sin STOP (ERROR): 0.
- Reloj total: 512.3 s. VÁLIDA. Corrió sola bajo twin.lock (procesos de banco ajenos al inicio: 0). Reloj 512.3 s para 384 carreras con -P 7 (referencia ~100-110 s por 96 → ~400-440 s). Carga máx 8.02, mem máx 4639 MB, MemoryError/Killed: 0. mean_pi_frame_ms normal: media 3.50 ms, máx 6.73 ms. El reloj (~128 s por cada 96) es igual al de la tanda hermana n85tc6 (524.4 s, mismo banco), así que no hay inflación por carga.
- Tipo de prueba: carrera completa con estacionamiento (FOX_PARK_POS=random). Esta tanda es la base de comparación para las variantes del mismo tipo sobre 4ee09ac (n85tc6, t7c90).
- Detalle por carrera y COSTO: `grok/resultados:resultados/cx_c2_r27_base_4ee09ac6.md`.

Siguiente: compara las variantes cx_c2_r27_* contra esta base con las mismas 384 carreras.
