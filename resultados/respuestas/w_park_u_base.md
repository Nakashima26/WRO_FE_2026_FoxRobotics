# Respuesta: w_park_u_base

_2026-11-24 09:02 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_park_u_base (28b640f: commit vacío sobre 2398305, el mismo árbol que w_park_leg_s35_base) tal como la pediste: 6 carreras (n1-2 x s16 s20 s32), banco nuevo de latencia, TWIN_MOTOR_REAL=1, VBAT 11.5, **con** TWIN_CAMERA_FROM_CALIB=1 + TWIN_LENS_CALIB=pure_pursuit/lens_calib.npz, TWIN_FW_OVER PARK_GEOM_EXT=1 + PARK_GEOM_ROJO=1, TWIN_PI_EXTRA ROUTE_PARK_GEOM_EXT=1, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7. Corrió sola bajo twin.lock al primer intento; no faltó ninguna dependencia del venv. (Tu TWIN_FW_OVER venía sin comillas simples en el comando; lo pasé como JSON entre comillas, igual que en w_park_leg_s35_base.)

- resumen.py: `w_park_u_base: 5/6 ok  media 42.44  mediana 43.22  mejor 37.75  fallas: n1/s20`
- Reloj total: 11.3 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx / mem máx 1.51 / 14332 MB, sin OOM/Killed/MemoryError, mean_pi_frame_ms 3.20-3.74, media de media_ms (COSTO) 10.2 ms.
- Tipo de prueba: carrera completa con estacionamiento geométrico (mismo tipo que w_park_leg_s35_base, w_geom_rojo_s35_base y r16_wstop).

Hallazgos:
- **Falla n1/s20: choque con el cajón magenta** a t=33.87 s (total_time_s 33.162; 12 giros completos, vueltas 13.38 / 8.34 / 8.38 s), o sea ya en la maniobra de estacionamiento, después de las 3 vueltas. n2/s20 (mismo mapa, otro ruido) termina en 37.746 s. s20 no se había corrido antes con estacionamiento geométrico, así que no hay referencia directa. Ojo: en otros árboles sin GEOM ya hubo choques con el cajón magenta en s20 (w_vel_r54_l1387_f295g n1/s20 y w_frenos_r36_g_cen_l1387_f295 n2/s20), así que ese mapa parece delicado al estacionar en general.
- s16/s32 (4/4 ok), frente al mismo árbol sin cámara calib (w_park_leg_s35_base): 4 carreras -4.03 s en total. Frente a w_geom_rojo_s35_base (094f93c, con cámara calib): 4 carreras +0.96 s. Frente a r16_wstop: 4 carreras -1.44 s. Con 4 carreras esto entra dentro del ruido del banco.
- Tabla por carrera, líneas STOP y COSTO completas: `grok/resultados:resultados/w_park_u_base.md`.

Pregunta: ¿la "u" del tag significa que la tanda apunta a la vuelta en U? Si es así, dime qué semillas tienen U para incluirlas; en s16/s20/32 no vi nada distinto en STOP.

Siguiente sugerido: s20 con n1-8 en este árbol (y en 094f93c) para ver si el choque con el cajón magenta es sistemático o cae en el ruido, y una corrida con traza de n1/s20 para ver la geometría de la entrada al cajón.
