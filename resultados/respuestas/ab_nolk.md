# Respuesta: ab_nolk

_2026-11-25 12:30 PST (Tijuana)_

Recibida. La tanda `ab_nolk` (origin/grok/t/ab_nolk @ 820d1145, n1-6 x s1-48 = **288 carreras**, -P 7, FF 2.8 / FD 3.8, banco NUEVO con latencia) quedó **en cola bajo twin.lock**. Hay varias tandas por delante (cal2_288_base, cal2_288_radios, s20cal2_radios, pk2_*, tw1_*, l1v_*, ab_nocs…), así que corre en cuanto se libere el lock, sin carga paralela.

Comando tal cual lo mandaste: `TWIN_FW_OVER={"MOTOR_CURVE_LIVE":"false"}` (string), `TWIN_PI_EXTRA={"ROUTE_MOTOR_CURVE_LIVE": false, "ROUTE_PROFILE_LEFT_K_FACTOR": 1.0, "ROUTE_L23_LEFT_K_FACTOR": 1.0}`, MOTOR_CURVA_REAL=1, COST_FIJO=1, COST_FIRST 1.95, VBAT 11.5, TOF_SHORT_K 0.5.

Al terminar subo `resultados/ab_nolk.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta. Referencia de reloj sin carga: ~300-330 s para 288 carreras; la marco inválida si pasa de ~400 s, hay OOM o carga máx ≥ 12.

Tipo de prueba: carrera completa (FF 2.8 / FD 3.8), n1-6 x s1-48. Comparable con otras tandas de 288 del mismo tipo y semillas (p. ej. ab_nocs, cal2_288_base), no con FF 4.28.
