# Respuesta: w_park_leg_s35_base

_2026-11-24 01:05 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_park_leg_s35_base (ce769ef: commit vacío sobre 2398305) como la pediste: 8 carreras (n1-2 x s35 s32 s16 s36), TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, TWIN_FW_OVER PARK_GEOM_EXT=1 + PARK_GEOM_ROJO=1, TWIN_PI_EXTRA ROUTE_PARK_GEOM_EXT=1, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7. Corrió sola bajo twin.lock. No faltó ninguna dependencia del venv.

- resumen.py: `w_park_leg_s35_base: 8/8 ok  media 44.41  mediana 44.06  mejor 42.93  fallas:`
- Reloj total: 18.9 s. VÁLIDA. Corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx 2.69, sin OOM/Killed/MemoryError, mean_pi_frame_ms 3.35-5.28, media de media_ms (COSTO) 10.9 ms.
- Tipo de prueba: carrera completa con estacionamiento geométrico, el mismo tipo que w_geom_rojo_s35_base y r16_wstop.

Hallazgos:
- **8/8 terminan.** n1/s35 termina en 43.455 s; en w_geom_rojo_s35_base (base 094f93c) chocaba con el cajón magenta a t=46.44 s.
- 2398305 tiene el mismo mensaje que 094f93c, pero está sobre una base anterior. Frente a 094f93c le faltan 1192b0f..6f9a7b6: Open MAP, el freno de isla por ToF (ROUTE_ISLAND_TOF_*) en route_mode.py, el perfil por caras de la isla y cambios en config.py. Si la única diferencia buscada es esa, la tanda apunta a que el choque de n1/s35 sin WSTOP viene de esos commits del freno de isla y no del estacionamiento GEOM.
- **Ojo, hay un factor mezclado:** esta orden no traía TWIN_CAMERA_FROM_CALIB=1 ni TWIN_LENS_CALIB=pure_pursuit/lens_calib.npz y w_geom_rojo_s35_base sí. Por eso no corrí con cámara desde calibración. La diferencia en n1/s35 puede venir también de la cámara.
- En las carreras que terminan en ambas: frente a w_geom_rojo_s35_base, 5 carreras, +4.47 s en total; frente a r16_wstop, 6 carreras, -0.68 s. Tabla por carrera en `grok/resultados:resultados/w_park_leg_s35_base.md`.

Pregunta: ¿omitiste la cámara desde calibración a propósito?

Siguiente: para aislar el efecto, repite esta tanda con TWIN_CAMERA_FROM_CALIB=1 TWIN_LENS_CALIB=pure_pursuit/lens_calib.npz. Si n1/s35 sigue terminando, manda s35 con n1-8 en las dos bases (2398305 contra 094f93c).
