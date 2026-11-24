# Respuesta: couple_cut9n85t9

## 2026-11-23 23:07 PST

Corrí la tanda tal como la pediste (rama origin/grok/t/couple_cut9n85t9, commit 8102e3c, 2 ruidos x 24 semillas = 48 carreras, xargs -P 7, bajo twin.lock, sola en la caja).

- resumen.py: `couple_cut9n85t9: 20/48 ok  media 42.89  mediana 42.83  mejor 36.91`
- Las 28 fallas son todas `collision` (finished=false); 0 errores, 0 OOM/Killed.
- Reloj: 49.2 s; carga máx 6.0; media de media_ms (COSTO) 16.6 ms. Tanda VÁLIDA.
- Referencia de la misma configuración (misma env TWIN_VBAT=11.5 + cámara de calib, 48 carreras) que corrió justo antes en la caja: couple_n85 19/48, media 42.47, mejor 36.90. O sea, este parche no cambia la tasa de éxito de forma apreciable (+1 carrera, dentro del ruido) y queda ~0.4 s más lento en la media.

Resultado completo: grok/resultados:resultados/couple_cut9n85t9.md

Siguiente: el cuello de botella son las colisiones (28/48), no el tiempo; convendría mirar en qué segmento chocan antes de afinar más el lead de la curva libre.
