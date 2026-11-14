# cxv1_estado_cancelacion — cancelación de las cinco tandas originales cxv1_*_1113a

Tipo: carrera completa desde el cajón en el banco con latencia (costo.py/uno.sh, FF 2.8 / FD 3.8, primer plan 1.95 s, VBAT 12.5, 96 carreras, P=7). Sin comparación de tiempos: no hay resultados.

Hora: 2026-11-13 18:51 PST.

Procesos terminados (SIGTERM por PID exacto, sin pkill/killall):

| tanda | PID run.sh | descendientes | carreras hechas |
|---|---|---|---|
| cxv1_lock3_1113a | 184286 | solo `sleep 5` (bucle de espera) | 0/96 |
| cxv1_debt_base_1113a | 160738 | solo `sleep 5` | 0/96 |
| cxv1_debt_1113a | 163723 | solo `sleep 5` | 0/96 |
| cxv1_stale_base_1113a | 164896 | solo `sleep 5` | 0/96 |
| cxv1_stale_1113a | 164502 | solo `sleep 5` | 0/96 |

Explicación: las cinco seguían en su bucle inicial de espera ("esperar a que terminen otras tandas", hasta 15 min, entre 10 y 26 min de vida), así que nunca llegaron a lanzar uno.sh/costo.py. No había carpetas en /tmp/lat/<tanda> ni salida en los logs: **0/96 cada una, sin resultados parciales; se marcan inválidas/canceladas**. No consumían CPU real, pero de haber seguido habrían arrancado sus 96 carreras en la caja saturada.

Intacto: la cola secuencial `cxv1_r1_seq.sh` (PID 197778) y sus checkouts `*_r1_wt` siguen vivos; no se tocaron bancos ajenos ni se reiniciaron las originales. No se creó otra cola ni banco. Al terminar las originales, la cola r1 puede avanzar a su espera de caja libre (carga<10, sin otros uno.sh/costo.py).

Carga al cancelar: 29.3 / 55.0 / 60.7 (1/5/15 min, 8 CPU), sigue alta por otros bancos.
