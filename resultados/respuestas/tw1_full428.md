# Respuesta: tw1_full428

_2026-11-25 12:28 PST (Tijuana)_

Recibida. La tanda `tw1_full428` (origin/grok/t/tw1_full428 @ f7197be6, n1-8 x s1-48 = **384 carreras**, -P 7, FF 4.28 / FD 5.8, banco NUEVO con latencia) quedó **en cola bajo twin.lock**. Hay varias tandas por delante (s20cal2_base, cal2_288_base/radios, pk2_base_0ee69022, la0g, tw1_base78, pk2_scan_a, tw1_eco_screen, tw1_curva_screen, l1v_cartas288, pk2_scan_b, tw1_base428, pk2_scan_v2, tw1_screen428, pk2_base_v2…), así que corre en cuanto se libere el lock, sin carga paralela.

Al terminar subo `resultados/tw1_full428.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta. Referencia de reloj sin carga: ~400-440 s para 384 carreras; la marco inválida si pasa de ~560 s, hay OOM o carga máx ≥ 12.

Comando tal cual lo mandaste (mismo entorno que tw1_base428: `TWIN_FW_OVER={"MOTOR_CURVE_LIVE":"false"}` como string, `TWIN_PI_EXTRA='{}'`, COST_FIRST 1.95).

Tipo de prueba: carrera completa (FF 4.28 / FD 5.8), n1-8 x s1-48. Comparable con tw1_base428 (mismo tipo, mismos FF/FD y semillas), no con tw1_base78 (FF 2.8 / FD 3.8).
