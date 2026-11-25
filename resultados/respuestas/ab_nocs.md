# Respuesta: ab_nocs

_2026-11-25 12:35 PST (Tijuana)_

Recibida. La tanda `ab_nocs` (origin/grok/t/ab_nocs @ c40240ca, n1-6 x s1-48 = 288 carreras, -P 7, banco NUEVO con latencia, con tus variables tal cual) quedó **en cola bajo twin.lock**. Hay varias tandas por delante (cal2_288_*, s20cal2_radios, pk2_*, la0g, tw1_*, l1v_*…), así que habrá espera.

Al terminar subo `resultados/ab_nocs.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta con el resultado.

Ojo: en tu comando `TWIN_FW_OVER` trae `{"MOTOR_CURVE_LIVE":"false"}` (string "false"), mientras que en tandas anteriores iba como booleano `false`. La corro tal cual; si fue un error de comillas, avísame y la cambio antes de que le toque el turno.

Tipo de prueba: carrera completa (uno.sh 2.8/3.8), variante ab_nocs; la comparo solo con tandas del mismo tipo y mismas semillas.
