# Respuesta: w_park_u_base_cen

_2026-11-24 09:44 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_park_u_base_cen (d560c99) tal como la pediste: 48 carreras (n1-2 x s1-24), GEOM_EXT + GEOM_ROJO sin PARK_UTURN_PWM (base de la U), ROUTE_PARK_GEOM_EXT=1, -P 7, bajo twin.lock. No faltó ninguna dependencia del venv. (TWIN_FW_OVER venía sin comillas en el payload; lo pasé como el JSON {"PARK_GEOM_EXT":"1","PARK_GEOM_ROJO":"1"}, igual que en las tandas anteriores.)

- resumen.py: `w_park_u_base_cen: 45/48 ok  media 41.93  mediana 41.59  mejor 36.44  fallas: n1/s2 n1/s23 n2/s11`
- Reloj total: 76.3 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx / mem máx 5.63 / 10629 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.13-10.58 (el 10.58 es n1/s2, que choca a los 1.6 s y sólo promedia los primeros cuadros de arranque; las otras 47 están en 3.13-5.25, como w_park_u_uturn185_cen 3.05-5.33). Reloj 76.3 s para 48 carreras (misma tanda tipo w_park_u_uturn185_cen: 72.7 s para 48; referencia ~100-110 s para 96).
- Tipo de prueba: carrera completa con estacionamiento geométrico y U, centrado (mismo tipo que w_park_u_uturn185_cen, que es la comparación directa; w_park_u_base / w_park_u_scan245 / w_park_u_uturn185 sólo tenían s16/s20/s32).

Fallas (última línea STOP):
  - n1/s2: choque, STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 1.572, "turns_completed": 0, "tpr": 12, "lap_times_s": [], "mean_corner
  - n1/s23: termina la carrera (43.094 s, sin choques) pero resumen.py la cuenta como falla: queda fuera del cajón > 5 mm
  - n2/s11: termina la carrera (42.999 s, sin choques) pero resumen.py la cuenta como falla: queda fuera del cajón > 5 mm

Comparación directa, carrera por carrera:
- w_park_u_uturn185_cen (mismas 48 carreras, mismo tipo, sólo cambia PARK_UTURN_PWM=185): resumen allá `w_park_u_uturn185_cen: 45/48 ok  media 42.03  mediana 41.99  mejor 36.38  fallas: n1/s21 n2/s21 n2/s22`. Terminan (STOP race_finished) aquí 47/48, allá 47/48; en las 46 que terminan en ambas, base_cen - uturn185_cen = -2.51 s en total (-0.055 s/carrera). Mayores diferencias: n1/s12 -3.12, n1/s9 +2.94, n2/s17 -2.55, n1/s22 +2.13, n1/s17 -2.04.

Comparación en semillas comunes con las tandas cortas (s16 y s20; s32 no está en s1-24). Aquí "termina" se lee del STOP (race_finished), no del criterio del cajón de resumen.py:
- w_park_u_base_cen en esas 6 (n1-2 x s20 y s16; s32 no entra en s1-24): n1/s16 42.857, n1/s20 36.571, n2/s16 43.781, n2/s20 38.937
  - w_park_u_base: allá 5/6 ok; en las 3 que terminan en ambas +1.62 s en total (n1/s16 -0.13, n2/s16 +0.56, n2/s20 +1.19).
  - w_park_u_scan245: allá 6/6 ok; en las 4 que terminan en ambas -0.21 s en total (n1/s16 +0.45, n1/s20 -0.73, n2/s16 -1.39, n2/s20 +1.47).
  - w_park_u_uturn185: allá 5/6 ok; en las 4 que terminan en ambas -1.07 s en total (n1/s16 +1.26, n1/s20 -1.05, n2/s16 +0.77, n2/s20 -2.05).

Lectura: 45/48 en la base (sin PARK_UTURN_PWM) contra 45/48 con U a 185: mismo conteo, pero fallan semillas distintas. Aquí n1/s2 choca con la señal roja N/T1 a los 1.6 s (0 giros; con 185 esa misma carrera terminó en 40.19 s, así que no tiene que ver con la U: parece sensibilidad al arranque / no determinismo) y n1/s23 y n2/s11 terminan sin choque pero quedan fuera del cajón (>5 mm). Con 185 fallaban s21 (las dos n, fuera del cajón) y n2/s22 (choque); aquí esas tres pasan. En tiempo la diferencia es ruido: -0.055 s/carrera en las 46 comunes (media 41.93 vs 42.03, mediana 41.59 vs 41.99). Conclusión: PARK_UTURN_PWM=185 no mejora ni empeora de forma clara el conteo ni el tiempo en este set; las fallas de cajón cambian de semilla, lo que apunta a que el margen del cajón final es justo en ambos casos más que a la PWM de la U.

Pregunta / siguiente: ¿quieres que corra PARK_UTURN_PWM=170 (o 178) con este mismo set n1-2 x s1-24 para cerrar la curva de la U? Si no, mándame el siguiente parche de cajón para las semillas que fallan aquí.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_park_u_base_cen.md`.
