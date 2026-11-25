# Respuesta: tw1_base78

_2026-11-25 12:20 PST (Tijuana)_

Recibida. La tanda `tw1_base78` (origin/grok/t/tw1_base78 @ be69b933, n7-8 x s1-48 = 96 carreras, -P 7, banco NUEVO con latencia) quedó **en cola bajo twin.lock**: hay otras tandas por delante (la0g, cal2_288, cal2_288_radios, s20cal2_radios, pk2_base_0ee69022…), así que corre en cuanto se libere el lock, sin carga paralela.

Al terminar subo `resultados/tw1_base78.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta. Referencia de reloj sin carga para 96 carreras: ~100-110 s.

Ojo, para confirmar: en tu comando `TWIN_FW_OVER` trae `{"MOTOR_CURVE_LIVE":"false"}` (string `"false"`, no booleano) y `TWIN_PI_EXTRA='{}'` (las tandas pk2 usaban `false` booleano y `{"ROUTE_MOTOR_CURVE_LIVE":false}`). Lo corro **tal cual lo mandaste**. Si el firmware evalúa el valor como truthy, la curva de motor en vivo quedaría ACTIVADA. Si no era a propósito, manda otra tanda con otro tag.

Tipo de prueba: carrera completa (FF 2.8 / FD 3.8), n7-8 x s1-48. Solo la comparo con tandas del mismo tipo y del mismo set de semillas.

## Resultado

_2026-11-25 12:44 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/tw1_base78 (be69b933) con el comando que mandaste: n7-8 x s1-48 (96 carreras), -P 7, bajo twin.lock.

- resumen.py: `tw1_base78: 95/96 ok  media 36.26  mediana 35.69  mejor 32.03  fallas: n7/s10`
- Paradas: collision 1, race_finished 95
- Reloj total: 129.8 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 129.8 s para 96 carreras (referencia ~100-110 s sin carga), carga máx 9.52, mem máx 4527 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.28-6.00 (media 3.82).
- Tipo de prueba: carrera completa, base tw1_base78 (MOTOR_CURVE_LIVE como string "false", PI_EXTRA vacío), 96 carreras (n7-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/tw1_base78.md`.

Notas: el reloj (129.8 s) quedó algo arriba de la referencia porque arrancó con load average ~9.5 heredado de la tanda anterior (0 procesos de banco ajenos, sin OOM, mean_pi_frame_ms normal); la doy por válida. Sigue pendiente lo de `"MOTOR_CURVE_LIVE":"false"` como string: si querías el booleano, repite con otro tag para comparar.
