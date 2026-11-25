# Respuesta: base1125_rep

_2026-11-25 00:25 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/base1125_rep (b8a314e4) tal como la pediste: n3-8 x s1-24 (144 carreras), TWIN_PI_EXTRA={}, -P 7, bajo twin.lock. No faltó ninguna dependencia del venv.

- resumen.py: `base1125_rep: 59/144 ok  media 42.72  mediana 42.51  mejor 37.45  fallas: n3/s1 n3/s10 n3/s13 n3/s14 n3/s15 n3…` (lista completa de fallas en el .md)
- Reloj total: 149.9 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), 149.9 s para 144 carreras (~100 s por 96), carga máx 7.47, mem máx 4566 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.07-11.13 (media 5.21).
- Tipo de prueba: carrera completa (mismo tipo que integ1125_rep).
- Paradas: 61 race_finished y 83 collision (resumen.py cuenta 59 ok, así que 2 terminaron pero las marca como fallas). Causas de choque: pared 42, isla 5, y el resto contra señales (sobre todo Red E/X2, W/X2, N/T4).

Comparación con integ1125_rep (mismo tipo y las mismas 144 semillas): integ 58/144 ok, media 42.56, mejor 37.08; esta base 59/144, media 42.72, mejor 37.45. La diferencia (+1 ok, +0.16 s de media) cabe en el ruido entre corridas; no veo una mejora ni una regresión clara de integ frente a la base. mean_pi_frame_ms de integ1125_rep: 3.13-13.00 (media 5.66).

Ojo: integ1125_rep arrancó con la memoria del box muy ocupada (12.3 GB usados al inicio, máx 14.4 GB) aunque sin OOM ni procesos ajenos; esta corrida empezó con 4.2 GB. Si quieres una comparación limpia, conviene repetir una de las dos.

Nota: la Mac de Jesse está desconectada, así que este resultado y esta respuesta quedan en cola en mi máquina y se suben a grok/resultados en cuanto vuelva (también lo de integ1125_rep).

Siguiente sugerido: repetir base1125_rep e integ1125_rep una vez más cada una (o con otra semilla de pista) para tener 2 muestras por lado antes de decidir.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/base1125_rep.md`.
