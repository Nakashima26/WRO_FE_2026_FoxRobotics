# Respuesta: w_park_u_abort185_cen

_2026-11-24 09:58 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_park_u_abort185_cen (708ba13) tal como la pediste: 48 carreras (n1-2 x s1-24), PARK_UTURN_ABORT_PWM=185 sobre GEOM_EXT + GEOM_ROJO, ROUTE_PARK_GEOM_EXT=1, -P 7. No faltó ninguna dependencia del venv.

- resumen.py: `w_park_u_abort185_cen: 46/48 ok  media 42.08  mediana 42.25  mejor 36.75  fallas: n1/s2 n1/s5`
- Reloj total: 73.6 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx / mem máx 5.11 / 4687 MB, OOM/Killed/MemoryError: 0. mean_pi_frame_ms 3.19-5.16 en las carreras completas; el 11.43 sale sólo de n1/s2, que dura 1.6 s (puro arranque).
- Tipo de prueba: carrera completa con estacionamiento geométrico y U. Es el mismo tipo y el mismo conjunto (n1-2 x s1-24) que w_park_u_base_cen y w_park_u_uturn185_cen.

Fallas (última línea STOP):
- n1/s2: choque con la señal Red N/T1 a t≈2.28 s, con 0 giros. Es idéntica a la de w_park_u_base_cen (mismo choque, 1.572 s), así que no viene del cambio.
- n1/s5: choque con el cajón magenta a t≈45.6 s, con los 12 giros hechos, o sea en el estacionamiento. Es nueva: en base_cen y en uturn185_cen s5 terminó bien en las dos n.

Comparación (mismo tipo y mismas semillas):
- w_park_u_base_cen (45/48; fallas n1/s2 n1/s23 n2/s11): en las 44 carreras ok en ambas, +9.86 s en total (+0.22 s por carrera).
- w_park_u_uturn185_cen (45/48; fallas n1/s21 n2/s21 n2/s22): en las 43 carreras ok en ambas, +4.44 s en total (+0.10 s por carrera).

Lectura: el aborto a 185 en s1-24 sube a 46/48, la mejor cuenta de las tres tandas _cen. Resuelve las fallas de cajón que tenía uturn185_cen (s21 n1/n2 y n2/s22 terminan aquí) y las de base_cen en n1/s23 y n2/s11. A cambio aparece una falla nueva en el cajón (n1/s5, magenta) y sale unas décimas más lento por carrera (+0.1 a +0.2 s). n1/s2 es la falla de señal que ya existía y no tiene que ver con el estacionamiento. Con una sola repetición por semilla, n1/s5 puede ser ruido. Sugiero repetir s5 (y s20, que falló en w_park_u_abort185) con 3-4 repeticiones para ver si se reproduce antes de adoptar el cambio.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_park_u_abort185_cen.md`.
