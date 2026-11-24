# Respuesta: couple_l3_10n85

_2026-11-23 23:10 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/couple_l3_10n85 (b82de1c) tal como la mandaste: 48 carreras (n1-2 x s1-24), TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, TWIN_CAMERA_FROM_CALIB=1, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7, con el TWIN_PI_EXTRA del payload. Corrió sola bajo twin.lock y no faltó ninguna dependencia del venv.

- resumen.py: `couple_l3_10n85: 19/48 ok  media 42.61  mediana 41.67  mejor 36.88` (29 fallas)
- Reloj total: 49.3 s. Es bajo porque muchas carreras chocan pronto. La tomo como **válida**: 0 procesos ajenos, carga máx 6.3 y sin OOM/Killed.
  - Ojo: mean_pi_frame_ms medio 5.84 y media_ms (COSTO) 17.5. En w_inicio_v2 eran 3.73 y 9.6. No hubo carga ajena, así que probablemente viene del env (cámara de calib / VBAT), no del banco. Si quieres, la repito para confirmarlo.
- **Las 29 fallas son choques con turns_completed ≤ 4**, es decir, en la vuelta 1 o justo al cerrarla (tc ≤ 4). Ninguna llega a la ventana del parche (L23, TC 6-10, ROUTE_LAP3_*). Causas: pared 9, isla 4 y señales 16 (Red E/X2 x3, Green W/T4 x3, Green E/T2 x2, Green N/T2 x2 y otras sueltas). Varias son muy tempranas: n1/s10 a 4.8 s, n2/s17 a 4.7 s y n2/s19 a 5.0 s.
- Mi lectura (hipótesis, sin verificar): el problema está en la vuelta 1 con este env (VBAT 11.5 + CAMERA_FROM_CALIB), no en couple_l3_10n85. Las tandas anteriores con cámara de calib también salieron bajas (calib_g 2/8, calj2 67/96). Con 19 carreras terminadas no se puede evaluar el efecto del parche en la vuelta 3.
- Comparación: en grok/resultados no hay ninguna tanda previa del mismo tipo con este mismo env (48 carreras, VBAT 11.5 + cámara de calib), así que no comparo totales.

Resultados completos (STOP y COSTO por carrera): `grok/resultados:resultados/couple_l3_10n85.md`.

Siguiente: manda la base (el mismo env sin TWIN_PI_EXTRA, o la rama sin el parche) para separar el efecto. Si la base también da ~19/48, el problema es de la vuelta 1 con VBAT/cámara de calib. Pregunta: ¿VBAT 11.5 + CAMERA_FROM_CALIB es el env nuevo de referencia, o fue un cambio puntual de esta tanda?
