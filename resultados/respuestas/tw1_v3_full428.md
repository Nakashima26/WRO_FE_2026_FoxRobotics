# Respuesta: tw1_v3_full428

_2026-11-25 12:32 PST (Tijuana)_

Recibida. La tanda `tw1_v3_full428` (origin/grok/t/tw1_v3_full428 @ f9c3f010, n1-8 x s1-48 = **384 carreras**, -P 7, FF 4.28 / FD 5.8, banco NUEVO con latencia) quedó **en cola bajo twin.lock**. Hay varias tandas por delante (entre ellas tw1_base428, tw1_screen428, tw1_full428 y tw1_v3_screen428), así que corre en cuanto se libere el lock, sin carga paralela.

Al terminar subo `resultados/tw1_v3_full428.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta. Referencia de reloj sin carga: ~400-440 s para 384 carreras; la marco inválida si pasa de ~560 s, hay OOM o carga máx ≥ 12.

Comando tal cual lo mandaste (`TWIN_FW_OVER={"MOTOR_CURVE_LIVE":"false"}` como string, `TWIN_PI_EXTRA='{}'`, COST_FIRST 1.95).

Tipo de prueba: carrera completa (FF 4.28 / FD 5.8), n1-8 x s1-48. Comparable directamente con tw1_full428 y tw1_base428 (mismo tipo, FF/FD y semillas).
