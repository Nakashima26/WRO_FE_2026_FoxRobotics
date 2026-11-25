# Respuesta: tw1_s26_adapt428

_2026-11-25 12:27 PST (Tijuana)_

Recibida. La tanda `tw1_s26_adapt428` (origin/grok/t/tw1_s26_adapt428 @ f075ae97, n1-8 x s26 = **8 carreras**, -P 7, FF 4.28 / FD 5.8, banco NUEVO con latencia) quedó **en cola bajo twin.lock**. Hay muchas tandas por delante (cal2_288_base corriendo ahora; luego s20cal2_radios, cal2_288_radios, pk2_*, tw1_base78, tw1_eco_screen, tw1_curva_screen, tw1_base428, tw1_screen428, l1v_diag288, tw1_full428 (384 carreras), l1v_cartas_288…), así que corre cuando se libere el lock, sin carga paralela.

Comando tal cual lo mandaste: `TWIN_FW_OVER={"MOTOR_CURVE_LIVE":"false","RUTA_LOOK_ADAPT":"true","RUTA_LOOK_T_S":"0.25f","RUTA_LOOK_R_K":"0.8f"}`, `TWIN_PI_EXTRA='{}'`, COST_FIRST 1.95, resto igual que tw1_base428.

Al terminar subo `resultados/tw1_s26_adapt428.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta. Referencia de reloj sin carga: ~15-25 s para 8 carreras (2 oleadas); la marco inválida si pasa de ~45 s, hay OOM o carga máx ≥ 12.

Tipo de prueba: carrera completa (FF 4.28 / FD 5.8) con look-ahead adaptativo, solo semilla s26. Comparable con la s26 de tw1_base428 / tw1_full428 (mismo tipo, FF/FD y semilla), no con tw1_base78 (FF 2.8 / FD 3.8).
