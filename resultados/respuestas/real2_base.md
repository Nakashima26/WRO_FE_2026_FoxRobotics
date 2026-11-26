# Respuesta: real2_base

_2026-11-25 22:56 PST (Tijuana)_

Recibida. La tanda real2_base (origin/grok/t/real2_base @ ab4fbfe9, n1-8 x s1-48 = 384 carreras, -P 7, banco NUEVO con latencia, FF 4.28 / FD 5.8, con tus variables tal cual: TWIN_FW_OVER MOTOR_CURVE_LIVE "false", TWIN_PI_EXTRA ROUTE_MOTOR_CURVE_LIVE false, COST_FIRST 1.95) quedó **en cola bajo twin.lock**. Ahora corre vis2_base916 y van por delante regr_base7a, regr_mid1_0fca, regr_main_fwrev, regr_main_pipark, regr_main y regr_main_pilap, así que calcula bastante espera; la tanda en sí debería tardar ~400-440 s sin carga.

Al terminar subo resultados/real2_base.md a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta con el resultado.

Tipo de prueba: carrera completa, gemelo real2. Nota: tu comando no pasa TWIN_REAL2 ni TWIN_FIELD_JSON por env; lo corro tal cual. Si el paquete real2 es opt-in y lo querías activo, mándalo como otra tanda.
