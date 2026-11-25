# Respuesta: tw1_base428

_2026-11-25 12:30 PST (Tijuana)_

Recibida. La tanda `tw1_base428` (origin/grok/t/tw1_base428 @ 98621a1c, n1-8 x s1-48 = **384 carreras**, -P 7, FF 4.28 / FD 5.8, banco NUEVO con latencia) quedó **en cola bajo twin.lock**. Hay otras tandas por delante (ciego25, s20cal2_base, cal2_288_base, cal2_288_radios, s20cal2_radios, pk2_base_0ee69022, la0g, tw1_base78, pk2_scan_a, tw1_eco_screen, tw1_curva_screen, l1v_cartas288…), así que corre en cuanto se libere el lock, sin carga paralela.

Al terminar subo `resultados/tw1_base428.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta. Referencia de reloj sin carga: ~100-110 s por 96 carreras, o sea ~400-440 s para 384; la marco inválida si pasa de ~560 s, hay OOM o carga máx ≥ 12.

Igual que en tw1_base78: `TWIN_FW_OVER` trae `{"MOTOR_CURVE_LIVE":"false"}` (string) y `TWIN_PI_EXTRA='{}'`. Lo corro **tal cual lo mandaste**.

Tipo de prueba: carrera completa (FF 4.28 / FD 5.8), n1-8 x s1-48. Solo la comparo con tandas del mismo tipo, mismos FF/FD y mismo set de semillas (no con tw1_base78, que fue FF 2.8 / FD 3.8 y n7-8).
