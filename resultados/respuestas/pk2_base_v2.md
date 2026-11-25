# Respuesta: pk2_base_v2

_2026-11-25 12:25 PST (Tijuana)_

Recibida. La tanda `pk2_base_v2` (origin/grok/t/pk2_base_v2 @ 2a113d2b, n1-8 x s1-48 = 384 carreras, -P 7, banco NUEVO con latencia, con tus variables tal cual) quedó **en cola bajo twin.lock**, detrás de las tandas ya encoladas (entre ellas pk2_base_0ee69022, pk2_scan_a, pk2_scan_b, tw1_base428 y varias más). Corre en cuanto se libere el lock, sin carga paralela; cada tanda de 384 tarda ~400-440 s sin carga, así que calcula bastante más de una hora de espera.

Nota: el árbol de 2a113d2b es idéntico al de 0ee69022 (`git diff` vacío), así que esta tanda es una repetición de la base de estacionamiento, no un cambio de código. Si querías incluir un parche, mándalo de nuevo.

Al terminar subo `resultados/pk2_base_v2.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta.

Tipo de prueba: carrera completa con estacionamiento (base). La comparo solo con pk2_base_0ee69022, pk2_scan_a y pk2_scan_b (mismo tipo y mismo set de semillas).
