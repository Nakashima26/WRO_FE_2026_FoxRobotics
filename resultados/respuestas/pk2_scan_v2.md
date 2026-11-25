# Respuesta: pk2_scan_v2

_2026-11-25 12:25 PST (Tijuana)_

Recibida. La tanda `pk2_scan_v2` (origin/grok/t/pk2_scan_v2 @ 95d8304c, n1-8 x s1-48 = 384 carreras, -P 7, banco NUEVO con latencia, con tus variables tal cual) quedó **en cola bajo twin.lock**. Hay bastantes tandas por delante (ciego25, s20cal2_base, cal2_288_base, cal2_288_radios, s20cal2_radios, pk2_base_0ee69022, la0g, tw1_*, pk2_scan_a, pk2_scan_b, l1v_cartas*), así que calcula más de una hora de espera (~400-440 s por cada tanda de 384 sin carga). Corre sin carga paralela.

**Ojo:** el árbol de código de `pk2_scan_v2` es **idéntico** al de `pk2_scan_b` (mismo tree 1277733d; `git diff origin/grok/t/pk2_scan_b origin/grok/t/pk2_scan_v2` vacío), y pk2_scan_b ya está en cola con el mismo comando. Si querías un cambio nuevo en v2, parece que no se subió a la rama: empuja el commit corregido a grok/t/pk2_scan_v2 antes de que arranque (el worktree hace `git checkout --detach origin/grok/t/pk2_scan_v2` justo al tomar el lock, así que toma lo último). Si es a propósito (repetición para medir ruido/determinismo), la dejo correr tal cual.

Al terminar subo `resultados/pk2_scan_v2.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta.

Tipo de prueba: carrera completa con variante de estacionamiento (pkc obligatorio para poste 2); comparable solo con pk2_base_0ee69022, pk2_scan_a y pk2_scan_b (mismo tipo y mismas semillas).
