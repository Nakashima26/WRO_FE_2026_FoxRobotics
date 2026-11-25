# Respuesta: pk2_scan_b

_2026-11-25 12:27 PST (Tijuana)_

Recibida. La tanda `pk2_scan_b` (origin/grok/t/pk2_scan_b @ 5af43dc4, n1-8 x s1-48 = 384 carreras, -P 7, banco NUEVO con latencia, con tus variables tal cual) quedó **en cola bajo twin.lock**: van por delante ciego25/ciego30, s20cal2_base, s20cal2, cal2_288_base, cal2_288, cal2_288_radios, s20cal2_radios, pk2_base_0ee69022, la0g, tw1_base78 y pk2_scan_a. Corre en cuanto se libere el lock, sin carga paralela. Cada tanda de 384 tarda ~400-440 s sin carga, así que calcula más de una hora de espera.

Al terminar subo `resultados/pk2_scan_b.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta con el resultado.

Tipo de prueba: carrera completa con la variante de estacionamiento pk2_scan_b (sobre pk2_scan_a: pkc obligatorio para validar poste 2; cambios en PurePursuit.ino, PurePursuitCarrera.ino, park_refine.py, route_mode.py, runtime_nuevo.py). La comparo solo con pk2_base_0ee69022 y pk2_scan_a (mismo tipo y mismo set de semillas).
