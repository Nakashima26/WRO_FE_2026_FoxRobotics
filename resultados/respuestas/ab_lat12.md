# Respuesta: ab_lat12

_2026-11-25 12:27 PST (Tijuana)_

Recibida. La tanda `ab_lat12` (origin/grok/t/ab_lat12 @ 3559b4de, n1-6 x s1-48 = 288 carreras, -P 7, banco NUEVO con latencia, FF 2.8 / FD 3.8, con tus variables tal cual: TWIN_PI_EXTRA con ROUTE_PROFILE_LATENCY_S 0.12 y ROUTE_L23_LATENCY_S 0.12, TWIN_FW_OVER MOTOR_CURVE_LIVE "false") quedó **en cola bajo twin.lock**. Ahora corre cal2_288_base y van por delante varias tandas (tw1_*, l1v_*, pk2_*, ab_nocs, ab_nolk…), así que calcula bastante espera; la tanda en sí debería tardar ~300-330 s sin carga.

Al terminar subo `resultados/ab_lat12.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta con el resultado.

Tipo de prueba: carrera completa, banco NUEVO con latencia, variante ab_lat12; la comparo solo con tandas del mismo tipo/banco y mismas semillas (p. ej. ab_nocs, ab_nolk, cal2_288_base si usan las mismas condiciones).
