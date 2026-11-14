# cxv1_estado_r1 — estado de la repetición aislada cxv1_*_1113a_r1

Tipo: carrera completa desde el cajón en el banco con latencia (costo.py/uno.sh, FF 2.8 / FD 3.8, primer plan 1.95 s, VBAT 12.5, 96 carreras, P=7).

Situación al recibir la orden (2026-11-13 18:49 PST):
- Caja saturada: carga 49.14 71.82 66.28 con 8 CPU, memoria casi agotada (15.6 GB), con OOM kills recientes; ~30 tandas de otros bancos corriendo en paralelo (cxpark_*, cxv23_*, v1*, gw14*, cxlb...).
- De las seis tandas originales cxv1_*_1113a, cinco siguen corriendo (debt_base, debt, stale_base, stale, lock3); cxv1_lock_base_1113a nunca arrancó. Todas corren saturadas, así que sus resultados deben considerarse **inválidos**. No se duplicaron ni se mataron procesos ajenos.

Qué se hizo:
- Se crearon checkouts aislados (`cxv1_*_1113a_r1_wt`) en los commits de origin/grok/t/cxv1_*_1113a.
- Quedó en cola un script secuencial que: espera a que terminen las cinco originales; adquiere flock exclusivo `/tmp/fox-grok-bancos.lock` (y `/workspace/fox/twin.lock`) y lo conserva toda la secuencia; antes de cada tanda espera a que no haya ningún otro proceso uno.sh/costo.py y carga<10 (máx 12 h por tanda; si no, la marca sin correr); corre las seis UNA TRAS OTRA en orden lock_base, lock3, debt_base, debt, stale_base, stale; anota carga inicial, memoria, carga/memoria máximas, OOM del cgroup y reloj; marca **inválida** cualquier tanda con OOM, reloj >130 s, carga máx ≥12 o carreras sin STOP.
- Cada resultado se publicará en resultados/cxv1_<x>_1113a_r1.md al terminar.

Limitación: los otros bancos no usan este lock, así que el aislamiento se verifica por tanda (sin otros bancos al arrancar) y la carga durante la tanda queda registrada para invalidar si alguien más arrancó en medio.
