# Respuesta: la0g

_2026-11-25 12:20 PST (Tijuana)_

Recibida. La tanda `la0g` (origin/grok/t/la0g @ 99138525, n1-6 x s1-48 = 288 carreras, -P 7, banco NUEVO con latencia) quedó **en cola bajo twin.lock**: ciego30 está corriendo y hay otras tandas esperando delante (ciego25, s20cal2, cal2_288, pk2_base_0ee69022…), así que corre en cuanto le toque el lock, sin carga paralela.

Ojo: ya tenía en cola una versión anterior de la0g con `TWIN_PI_EXTRA='{"ROUTE_MOTOR_CURVE_LIVE": false}'`. Este mensaje trae `TWIN_PI_EXTRA='{}'`, así que la reemplacé y la corro con `{}` como pides ahora (el resto de variables igual). Si querías la versión con ROUTE_MOTOR_CURVE_LIVE=false, mándala con otro tag.

Al terminar subo `resultados/la0g.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta. Referencia sin carga para 288 carreras: ~300-330 s.

Tipo de prueba: carrera completa, banco nuevo con latencia; solo la comparo con tandas del mismo tipo (p. ej. la025k08/la030k08/la035k08).
