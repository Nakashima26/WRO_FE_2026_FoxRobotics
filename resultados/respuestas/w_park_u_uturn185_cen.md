# Respuesta: w_park_u_uturn185_cen

_2026-11-24 09:06 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_park_u_uturn185_cen (adcd857, sobre 2398305) tal como la pediste: 48 carreras (n1-2 x s1-24), PARK_UTURN_PWM=185 sobre GEOM_EXT + GEOM_ROJO, ROUTE_PARK_GEOM_EXT=1, -P 7. No faltó ninguna dependencia del venv.

- resumen.py: `w_park_u_uturn185_cen: 45/48 ok  media 42.03  mediana 41.99  mejor 36.38  fallas: n1/s21 n2/s21 n2/s22`
- Reloj total: 72.7 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx / mem máx 5.37 / 4469 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.05-5.33. Reloj 72.7 s para 48 carreras (referencia ~100-110 s para 96).
- Tipo de prueba: carrera completa con estacionamiento geométrico y U (mismo tipo que w_park_u_base / w_park_u_scan245 / w_park_u_uturn185, pero esas sólo tenían s16/s20/s32).

Fallas (última línea STOP):
  - n1/s21: termina la carrera (40.125 s, sin choques) pero resumen.py la cuenta como falla: queda fuera del cajón > 5 mm
  - n2/s21: termina la carrera (38.89 s, sin choques) pero resumen.py la cuenta como falla: queda fuera del cajón > 5 mm
  - n2/s22: choque, STOP collision {"finished": false, "stop_reason": "collision", "total_time_s": 43.703, "turns_completed": 12, "tpr": 12, "lap_times_s": [13.781, 9.269

Comparación en semillas comunes (s16 y s20; s32 no está en s1-24). Ojo: aquí "termina" se lee del STOP (race_finished), no usa el criterio del cajón de resumen.py:
- w_park_u_uturn185_cen en esas 6 (n1-2 x s20 y s16; s32 no entra en s1-24): n1/s16 42.973, n1/s20 37.743, n2/s16 43.141, n2/s20 36.907
  - w_park_u_base: allá 5/6 ok; en las 3 que terminan en ambas -0.94 s en total (n1/s16 -0.01, n2/s16 -0.08, n2/s20 -0.84).
  - w_park_u_scan245: allá 6/6 ok; en las 4 que terminan en ambas -1.59 s en total (n1/s16 +0.56, n1/s20 +0.44, n2/s16 -2.03, n2/s20 -0.56).
  - w_park_u_uturn185: allá 5/6 ok; en las 4 que terminan en ambas -2.45 s en total (n1/s16 +1.37, n1/s20 +0.12, n2/s16 +0.13, n2/s20 -4.08).

Lectura: 45/48 con U a 185. Las 3 fallas son de estacionamiento: n2/s22 choca ya con los 12 giros hechos, y en s21 (las dos n) termina pero queda fuera del cajón. Conviene mirar la geometría final del cajón en s21 y s22. Para saber si 185 ayuda o estorba, falta una base igual (mismo FW_OVER sin PARK_UTURN_PWM, o con 170) en n1-2 x s1-24. Con eso comparo carrera por carrera.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_park_u_uturn185_cen.md`.
