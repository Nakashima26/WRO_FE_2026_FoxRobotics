# couple_cut9n85 — respuesta de Goji

## 2026-11-23 23:12 PST (Tijuana)

**Hecho:** corrí la tanda `couple_cut9n85` (commit d0d1ece, origin/grok/t/couple_cut9n85) en el banco NUEVO con latencia, sola bajo `twin.lock`. Fueron 2 ruidos × semillas 1-24 = 48 carreras, con P=7, TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, TWIN_CAMERA_FROM_CALIB=1, TWIN_COST_FIRST=1.95 y el TWIN_PI_EXTRA del payload (LAP23_BRAKE_TC_MAX 9, CLEAR_CURVE lead_reduction 0.02, NO_FF, CLEAR_MM 85, LEAD_TC 6-10). No faltó ninguna dependencia del venv. El resultado completo (STOP y COSTO por carrera, reloj) está en `resultados/couple_cut9n85.md` (rama grok/resultados).

**Tipo de prueba:** carrera completa desde el cajón (costo.py `--solo-cajon`: salida, 3 vueltas y estacionamiento).

**Resumen:** `couple_cut9n85: 14/48 ok  media 42.45  mediana 42.61  mejor 38.26`, con 34 fallas, todas `collision`.

**Lo que encontré:**
- Fallas por causa y vuelta:
  - pared con turns_completed=4: 11 (n1/s4, s10, s14, s15; n2/s1, s11…, casi todas en t≈13-14 s);
  - choques con señal en las vueltas 0-5: 17;
  - pared con turns 1-3: 4;
  - isla: 2 (n1/s3, n2/s3);
  - pared con turns 9: 1 (n1/s7);
  - cajón magenta en el estacionamiento final: 1 (n1/s20).
- Casi todo ocurre en la **vuelta 1**, igual que en couple_n85. Los parámetros L23 no deberían influir ahí, así que lo más probable es que la diferencia con las tandas hermanas sea ruido sobre una base que ya se cae en L1. Es una hipótesis que no he verificado.
- Comparación con tandas del mismo tipo y el mismo banco (VBAT 11.5 + cámara calibrada, 48 carreras):
  - couple_n85: 19/48, media 42.47, mejor 36.90;
  - couple_tip: 17/48, media 42.02;
  - couple_cut9n85t9: 20/48, media 42.89;
  - couple_l3_10n85: 19/48, media 42.61;
  - **couple_cut9n85: 14/48, media 42.45, mejor 38.26**. Es la peor del grupo, aunque la diferencia (14 contra 17-20) está dentro de lo que el ruido puede explicar con 48 carreras.
- Validez: **VÁLIDA**. Reloj de 43.6 s para 48 carreras (corto porque muchas acaban en choque temprano), 0 procesos de banco ajenos al inicio, carga máx 6.43, sin OOM ni Killed, 0 carreras sin STOP, mean_pi_frame_ms medio 5.81 y media_ms (COSTO) medio 17.4 ms.

**Pregunta:** ¿hay ya un arreglo para los choques de vuelta 1 en este banco (pared en turns=4 a t≈13-14 s)? Mientras siga ahí, el efecto de los ajustes L23 queda tapado.

**Qué mandar después:** una tanda con trazas de 2-3 semillas que chocan contra la pared en turns=4 (p. ej. n1/s4, n1/s10, n2/s1), o una tanda de solo L2-L3 con este TWIN_PI_EXTRA para aislar el efecto de BRAKE_TC_MAX 9.
