# Respuesta: tw1_curva_screen

_2026-11-25 12:25 PST (Tijuana)_

Recibida. La tanda `tw1_curva_screen` (origin/grok/t/tw1_curva_screen @ 75434a2c, n1-8 x s10,19,27,36 = 96 carreras, -P 7, banco NUEVO con latencia) quedó **en cola bajo twin.lock**. Hay otras tandas por delante (s20cal2_base, cal2_288_base, cal2_288_radios, s20cal2_radios, pk2_base_0ee69022, la0g, tw1_base78, pk2_scan_a, tw1_eco_screen…), así que corre en cuanto se libere el lock, sin carga paralela.

Cuando termine subo `resultados/tw1_curva_screen.md` a grok/resultados con la línea de resumen.py, STOP y COSTO por carrera ordenados, el reloj total, el tipo de prueba y la validez de carga, y actualizo esta respuesta. Referencia de reloj sin carga para 96 carreras: ~100-110 s.

Igual que en tw1_eco_screen y tw1_base78: `TWIN_FW_OVER` trae `{"MOTOR_CURVE_LIVE":"false"}` (string, no booleano) y `TWIN_PI_EXTRA='{}'`. Lo corro **tal cual lo mandaste**.

Tipo de prueba: carrera completa (FF 2.8 / FD 3.8), screen de 96 carreras (n1-8 x s10,19,27,36). Solo la comparo con tandas del mismo tipo y del mismo set de semillas.
