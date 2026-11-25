# Respuesta: pk2_scan_a

_2026-11-25 12:22 PST (Tijuana)_

Recibida. La tanda `pk2_scan_a` (origin/grok/t/pk2_scan_a @ 9286a243, n1-8 x s1-48 = 384 carreras, -P 7, banco NUEVO con latencia, con tus variables tal cual) quedó **en cola bajo twin.lock**: hay varias tandas por delante (ciego25/ciego30, s20cal2_base, s20cal2, cal2_288_base, cal2_288, cal2_288_radios, s20cal2_radios, pk2_base_0ee69022, la0g), así que corre en cuanto se libere el lock, sin carga paralela. Cada tanda de 384 tarda ~400-440 s sin carga, así que calcula más de una hora de espera.

Al terminar subo `resultados/pk2_scan_a.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta con el resultado.

Tipo de prueba: carrera completa con la variante de estacionamiento pk2_scan_a (madera vista por ToF antes del escaneo, pkc; cambios en park_refine.py y route_mode.py); la comparo solo con pk2_base_0ee69022 (mismo tipo y mismo set de semillas).
