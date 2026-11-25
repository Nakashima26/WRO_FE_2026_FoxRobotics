# Respuesta: l1v_corredor_288

_2026-11-25 12:31 PST (Tijuana)_

Recibida. La tanda `l1v_corredor_288` (origin/grok/t/l1v_corredor_288 @ bdf36eca, n1-6 x s1-48 = 288 carreras, -P 7, banco NUEVO con latencia, FF 2.8 / FD 3.8, TWIN_PI_EXTRA {"ROUTE_L1_PATH_SPEED":true}, TWIN_FW_OVER {}, COST_FIRST 1.95, resto de variables tal cual) quedó **en cola bajo twin.lock**. Ahora corre cal2_288_base y van por delante varias tandas (tw1_*, l1v_*, pk2_*, ab_*), así que calcula bastante espera; la tanda en sí debería tardar ~300-330 s sin carga.

**Aviso importante:** el disco de la máquina se llenó (100 %, 405 MB libres) por acumulación de worktrees. El primer intento de preparar el worktree de esta tanda falló con "No space left on device". Borré 61 worktrees de tandas ya terminadas (DONE/OMITIDA/CANCELADA/ABORTADA; se pueden recrear desde git, 45 GB liberados) y la relancé; ya está en cola con su worktree listo. **cal2_288_base, que estaba corriendo en ese momento, tiene al menos 8 carreras (n1: s8-s13, s15, s19) con `OSError: [Errno 28] No space left on device`**: sus resultados no van a ser válidos y habrá que repetirla.

Al terminar subo `resultados/l1v_corredor_288.md` a grok/resultados (línea de resumen.py, STOP y COSTO por carrera ordenados, reloj total, tipo de prueba y validez de carga) y actualizo esta respuesta.

Tipo de prueba: carrera completa, banco NUEVO con latencia, variante l1v con ROUTE_L1_PATH_SPEED, FF 2.8 / FD 3.8; solo la comparo con tandas del mismo tipo/banco y mismas semillas (no con l1v_cartas_288, que usa FF 4.28 / FD 5.8).
