# Respuesta: w_geom_rojo_r17_noprepka

_2026-11-24 01:05 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_geom_rojo_r17_noprepka (348e769: `acercar` solo con `along >= rutaPka`, sobre 094f93c) tal como la pediste: 12 carreras (n1-2 x s35 s32 s16 s36 s19 s26), TWIN_MOTOR_REAL=1, VBAT 11.5, cámara desde calibración, PARK_GEOM_EXT=1 + PARK_GEOM_ROJO=1, ROUTE_PARK_GEOM_EXT=1, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7. Corrió sola bajo twin.lock y no faltó ninguna dependencia del venv.

- resumen.py: `w_geom_rojo_r17_noprepka: 8/12 ok  media 44.57  mediana 43.99  mejor 42.45  fallas: n1/s19 n1/s35 n1/s36 n2/s35`
- Reloj total: 20.5 s. La doy por **válida**: carga máx 2.18, sin OOM/Killed.
- Tipo de prueba: carrera completa con estacionamiento geométrico, el mismo que s35_base/r16_wstop/r15b/r14.

Hallazgos:
- **El parche empeora el resultado.** Pasa de 5/6 en s35_base a 8/12 aquí. En las 6 semillas comunes queda en 4/6, porque se suma la falla de n2/s35.
- **s35 sigue chocando con el cajón magenta** en el estacionamiento: n1 a t=42.64 s y ahora también n2 a t=44.55 s (en s35_base, n2 terminaba en 45.782). Quitar el acercamiento antes de pka no resuelve el caso s35@60. Parece que lo traslada.
- **Regresión nueva en n1/s19**: choca con la señal Green W/T1 a 34.47 s, justo al acabar la vuelta 3, en la aproximación al estacionamiento. En r14 terminaba (42.200).
- n1/s36 tiene el choque temprano de siempre (Green S/T1, 6.65 s) y no tiene que ver con esto. n2/s36 termina.
- Tiempos en las carreras que terminan en ambas tandas: frente a s35_base, 4 carreras, +1.41 s; frente a r16_wstop, 4 carreras, -1.00 s; frente a r14, 8 carreras, -1.87 s.
- Tabla por carrera: `grok/resultados:resultados/w_geom_rojo_r17_noprepka.md`.

Mi opinión: descartaría noprepka tal como está. La única variante que ha terminado s35 (n1 y n2) en la base 094f93c es WSTOP=262 (r16_wstop). Su problema es s36, que es un choque temprano que no depende del estacionamiento.

Siguiente: r16_wstop (WSTOP=262, sin noprepka) con las 6 semillas de esta tanda más s19, y n1-4 para medir la regresión en s19 y la robustez de s35. Si quieres seguir con noprepka, prueba un margen (`along >= rutaPka - X`) en vez del corte duro.
