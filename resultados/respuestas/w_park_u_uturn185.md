# Respuesta: w_park_u_uturn185

_2026-11-24 09:03 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_park_u_uturn185 (5ec9fa0, commit vacío sobre 2398305) como la pediste: 6 carreras (n1-2 x s16 s20 s32), PARK_UTURN_PWM=185 sobre GEOM_EXT + GEOM_ROJO, cámara desde calibración, -P 7. Corrió sola bajo twin.lock, justo después de w_park_u_base y w_park_u_scan245. No faltó ninguna dependencia del venv.

- resumen.py: `w_park_u_uturn185: 5/6 ok  media 41.33  mediana 41.60  mejor 37.62  fallas: n2/s32`
- Reloj total: 14.6 s. VÁLIDA: corrió sola bajo twin.lock (0 procesos de banco ajenos al inicio), carga máx 2.84, mem máx 10288 MB, 0 OOM/Killed/MemoryError, mean_pi_frame_ms ~3.6. Reloj 14.6 s para 6 carreras (w_park_u_base 11.3 s, w_park_u_scan245 16.1 s).
- Tipo de prueba: carrera completa con estacionamiento geométrico y U. Es el mismo tipo que w_park_u_base y w_park_u_scan245, con las mismas semillas.

Hallazgos:
- **5/6 terminan. n2/s32 choca con el cajón magenta a t=43.16 s** (12 giros completos, así que el choque es en el estacionamiento). En base y en scan245, n2/s32 terminaba (44.005 / 44.279 s).
- Recupera n1/s20, que chocaba en base (33.16 s), igual que scan245.
- Tiempos: en las 4 que terminan en base y en uturn185, +0.84 s en total frente a base. En las 5 que terminan en scan245 y en uturn185, -0.49 s. Es casi un empate. n2/s20 pierde 3.2 s frente a base, y el resto gana entre 0.2 y 1.4 s.
- Conclusión: con 6 carreras, U a 185 no mejora sobre scan245 (6/6) y mete un choque nuevo en el cajón. Así como está, no la recomiendo.

Siguiente: si quieres seguir con la U, prueba PARK_UTURN_PWM 175-180 o combínala con PARK_SCAN_U_PWM=245. Mándalo con más semillas (por ejemplo s1-24) y n2/s32 incluida, porque 6 carreras no separan bien del ruido.

Nota: la Mac de Jesse no responde ahora. El resultado y esta respuesta quedan en cola en la caja y se suben a `grok/resultados` (`resultados/w_park_u_uturn185.md`, `resultados/respuestas/w_park_u_uturn185.md`) cuando vuelva.
