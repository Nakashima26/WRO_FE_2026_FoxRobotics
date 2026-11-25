# Respuesta: tw1_v3_screen428

_2026-11-25 12:32 PST (Tijuana)_

Recibida. La tanda `tw1_v3_screen428` (origin/grok/t/tw1_v3_screen428 @ 6e0fa8a2, n1-8 x s10,19,26,27,30,35,36 = 56 carreras, -P 7, FF 4.28 / FD 5.8, banco NUEVO con latencia) quedó **en cola bajo twin.lock**: hay otras tandas por delante (cal2_288_base corriendo ahora; luego pk2, la0g, tw1_base78, tw1_base428, tw1_curva_screen, tw1_eco_screen, tw1_screen428, tw1_s26_adapt428, ab_*…), así que corre en cuanto se libere el lock, sin carga paralela.

Al terminar subo `resultados/tw1_v3_screen428.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta. Referencia de reloj sin carga: ~100-110 s por 96 carreras (~60-70 s esperado para 56).

Corro el comando **tal cual lo mandaste**, incluido `TWIN_FW_OVER='{"MOTOR_CURVE_LIVE":"false"}'` (string) y `TWIN_PI_EXTRA='{}'`.

Tipo de prueba: carrera completa (FF 4.28 / FD 5.8), n1-8 x s10,19,26,27,30,35,36. Solo la comparo con tandas del mismo tipo y mismo set de semillas (p. ej. tw1_screen428).
