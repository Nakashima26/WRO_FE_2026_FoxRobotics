# Respuesta: integ1125_rep

_2026-11-25 00:23 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/integ1125_rep (40c883c3) tal como la pediste (n3-8 x s1-24, 144 carreras, -P 7, TWIN_PI_EXTRA={}), bajo twin.lock. No faltó ninguna dependencia del venv.

- resumen.py: `integ1125_rep: 58/144 ok  media 42.56  mediana 42.17  mejor 37.08  fallas: n3/s1 n3/s10 n3/s13 n3/s14 n3/s15 n3/s16 n3/s19 n3/s22 n3/s23 n3/s24 n3/s3 n3/s4 n3/s7 n4/s1 n4/s10 n4/s12 n4/s14 n4/s15 n4/s17 n4/s18 n4/s19 n4/s2 n4/s21 n4/s22 n4/s23 n4/s4 n4/s7 n5/s1 n5/s10 n5/s13 n5/s14 n5/s15 n5/s17 n5/s19 n5/s22 n5/s23 n5/s3 n5/s4 n5/s7 n6/s1 n6/s10 n6/s14 n6/s15 n6/s19 n6/s21 n6/s22 n6/s23 n6/s3 n6/s4 n6/s7 n6/s8 n7/s1 n7/s10 n7/s12 n7/s13 n7/s14 n7/s15 n7/s18 n7/s19 n7/s2 n7/s21 n7/s22 n7/s23 n7/s3 n7/s4 n7/s5 n7/s7 n7/s8 n8/s1 n8/s10 n8/s11 n8/s12 n8/s13 n8/s14 n8/s15 n8/s17 n8/s19 n8/s21 n8/s22 n8/s23 n8/s24 n8/s3 n8/s4 n8/s5 n8/s6 n8/s7`
- Reloj total: 165.7 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx 7.31, mem máx 14410 MB de 16002, OOM/Killed/MemoryError: 0, stderr vacío. Reloj 165.7 s para 144 carreras (≈110.5 s por 96). mean_pi_frame_ms mediana 5.42 (rango 3.13-13.0; los valores altos son de carreras que chocaron en los primeros segundos).
- Tipo de prueba: carrera completa con el banco nuevo de latencia. En grok/resultados no hay otra tanda 1125 de este tipo para comparar. Vi que también existe origin/grok/t/base1125_rep: cuando llegue esa tanda, la comparación correcta es contra ella (mismo tipo y mismas semillas).

Fallas: 86/144 son choques (ninguna sin salida ni ERROR). Primera causa de choque: señales 40 (sobre todo Red E/X2, Red W/X2, Red S/T3, Red N/T4), pared 37, isla 3, otras 6. Muchos choques ocurren muy pronto (4-20 s). Las semillas s1, s4, s7, s10, s14, s15, s19, s22 y s23 fallan en todas o casi todas las n, así que el patrón parece depender de la semilla y no del ruido.

Pregunta: ¿quieres que, cuando corra base1125_rep, te mande una tabla por semilla de integ contra base (ok/falla y tiempo)?

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/integ1125_rep.md`.
