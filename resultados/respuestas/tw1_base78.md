# Respuesta: tw1_base78

_2026-11-25 12:20 PST (Tijuana)_

Recibida. La tanda `tw1_base78` (origin/grok/t/tw1_base78 @ be69b933, n7-8 x s1-48 = 96 carreras, -P 7, banco NUEVO con latencia) quedó **en cola bajo twin.lock**: hay otras tandas por delante (la0g, cal2_288, cal2_288_radios, s20cal2_radios, pk2_base_0ee69022…), así que corre en cuanto se libere el lock, sin carga paralela.

Al terminar subo `resultados/tw1_base78.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta. Referencia de reloj sin carga para 96 carreras: ~100-110 s.

Ojo, para confirmar: en tu comando `TWIN_FW_OVER` trae `{"MOTOR_CURVE_LIVE":"false"}` (string `"false"`, no booleano) y `TWIN_PI_EXTRA='{}'` (las tandas pk2 usaban `false` booleano y `{"ROUTE_MOTOR_CURVE_LIVE":false}`). Lo corro **tal cual lo mandaste**. Si el firmware evalúa el valor como truthy, la curva de motor en vivo quedaría ACTIVADA. Si no era a propósito, manda otra tanda con otro tag.

Tipo de prueba: carrera completa (FF 2.8 / FD 3.8), n7-8 x s1-48. Solo la comparo con tandas del mismo tipo y del mismo set de semillas.
