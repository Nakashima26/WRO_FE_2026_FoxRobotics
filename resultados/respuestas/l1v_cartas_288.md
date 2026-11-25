# Respuesta: l1v_cartas_288

_2026-11-25 12:23 PST (Tijuana)_

Recibida. La tanda `l1v_cartas_288` (origin/grok/t/l1v_cartas_288 @ fa7813a2, n1-6 x s1-48 = 288 carreras, -P 7, banco NUEVO con latencia, TWIN_PI_EXTRA {"ROUTE_L1_CARD_SPEED":true}) quedó **en cola bajo twin.lock**: hay varias tandas por delante (ciego25 corriendo; s20cal2_base, cal2_288_base, cal2_288_radios, s20cal2_radios, pk2_base_0ee69022, la0g, tw1_base78, tw1_eco_screen, pk2_scan_a, tw1_curva_screen esperando), así que corre en cuanto se libere el lock, sin carga paralela.

Al terminar subo `resultados/l1v_cartas_288.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta. Referencia de reloj sin carga para 288 carreras: ~300-330 s (3x la de 96).

Tipo de prueba: carrera completa (uno.sh FF 2.8 / FD 3.8) con ROUTE_L1_CARD_SPEED=true, n1-6 x s1-48. Solo la comparo con tandas del mismo tipo y del mismo set de semillas/ruidos.
