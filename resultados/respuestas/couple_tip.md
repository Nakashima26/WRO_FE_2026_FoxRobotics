# Respuesta: couple_tip

_2026-11-23 23:08 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/couple_tip (7cf6702) con los parámetros que mandaste: 48 carreras (n1-2 x s1-24), TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, TWIN_CAMERA_FROM_CALIB=1, TWIN_PI_EXTRA='{}', COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7. Corrió sola bajo twin.lock y no faltó ninguna dependencia del venv.

- Tipo de prueba: carrera completa desde el cajón (--solo-cajon), banco nuevo con latencia.
- resumen.py: `couple_tip: 17/48 ok  media 42.02  mediana 41.69  mejor 36.91  fallas: n1/s1 n1/s10 n1/s11 n1/s12 n1/s14 n1/s15 n1/s19 n1/s22 n1/s23 n1/s24 n1/s3 n1/s4 n1/s5 n1/s7 n1/s9 n2/s1 n2/s10 n2/s11 n2/s12 n2/s15 n2/s17 n2/s19 n2/s22 n2/s23 n2/s24 n2/s3 n2/s4 n2/s5 n2/s6 n2/s8 n2/s9`
- Causa de las 30 carreras con colisión (primera colisión; resumen.py cuenta 31 fallas, la restante terminó sin colisión): pared 8, isla 5, señal Red E/X2 3, señal Red N/T4 3, señal Green N/T2 2, señal Red S/T2 1, señal Red S/T1 1, señal Green W/T4 1, señal Red W/T3 1, señal Green E/X1 1, señal Red S/T4 1, señal Green S/X1 1, señal Red W/X2 1, señal Red E/T1 1.
- Reloj total: 50.7 s. La doy por **válida**: 0 procesos ajenos al inicio, carga máx 6.77, sin OOM/Killed/MemoryError.
- Contexto (mismo tipo y mismo env, corridas justo antes en la caja, con otras ramas o TWIN_PI_EXTRA): couple_cut9n85t9 20/48 (media 42.89), couple_n85 19/48 (media 42.47). couple_tip con extra vacío da 17/48 (media 42.02). Las tres quedan en ~35-42 % de éxito, así que con este env el problema dominante (choques con señal) no parece venir de las perillas del parche. Ojo: 2-3 carreras de diferencia sobre 48 puede ser ruido.

Resultados completos (STOP y COSTO por carrera): `grok/resultados:resultados/couple_tip.md`.

Siguiente: si quieres separar el efecto del parche, manda la base (mismo env sin parche) y la corro igual. También puedo sacar el detalle de los choques con señal.
