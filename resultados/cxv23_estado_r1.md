# cxv23 – estado de la repetición aislada (rv23/chain.sh) – r1

Fecha: 2026-11-13 18:56 PST  
Tipo de prueba: banco con latencia (twin, 96 corridas por tanda = 2 vueltas-config × 48 semillas, VBAT 12.5), repetición aislada de cxv23_b05 / g14c / b04 / los2. Solo coordinación: no se crearon tandas nuevas ni se cambiaron parámetros ni código de ramas fuente.

## Locks
- **Aún no adquirido** al momento de este estado. La secuencia está esperando aislamiento para la primera tanda (cxv23_rb05) desde 18:48:43.
- Estado del box ahora: carga ≈ 38.60, 72 procesos uno.sh/costo.py ajenos en ejecución (tandas cxpark_*, cxv23_b0x, gw*, v1* corriendo fuera de esta secuencia). twin.lock lo tiene hoy otra tanda (v1kB); cxv1_r1_seq.sh y varias tandas esperan en cola.
- Ajuste hecho (sin matar procesos, sin reiniciar chain.sh, sin duplicar la secuencia):
  - chain.sh (en ejecución) se editó solo **después** de la línea que ya ejecuta: tras terminar rb05 adquiere `flock` exclusivo de `/workspace/fox/twin.lock` (fd 8) y `/tmp/fox-grok-bancos.lock` (fd 9) y los **retiene durante g14c, b04 y los2** hasta ALLDONE (mismo orden twin→bancos que cxv1_r1_seq y cxpark_r1_seq, sin riesgo de deadlock por orden).
  - rb05 corre con la función `run` ya cargada (flock twin.lock por tanda); tanda.sh ahora además toma `fox-grok-bancos.lock` cuando no está bajo la secuencia con locks, así rb05 también corre con ambos locks.
  - Antes de cada tanda (g14c, b04, los2): espera **cero** uno.sh/costo.py ajenos y carga < 10. Ya **no** espera scripts `*_run.sh`/`*_wait.sh` (algunos son `flock twin.lock ./x_run.sh` dormidos en la cola del lock; esperarlos con el lock tomado sería deadlock). La espera de rb05 aún incluye ese chequeo antiguo, pero no tiene locks tomados mientras espera, así que no bloquea.
  - cxpark_r1_seq.sh espera a que termine rv23/chain.sh y cxv1_r1_seq.sh por pgrep antes de tomar locks; chain.sh no espera a ningún coordinador → no hay espera circular.
  - Respaldos: rv23/chain.sh.bak_coord_r1, rv23/tanda.sh.bak_coord_r1.

## Tandas
| orden | tanda | fuente | estado |
|---|---|---|---|
| 1 | cxv23_rb05 | origin/grok/t/cxv23_b05 | pendiente (esperando aislamiento) |
| 2 | cxv23_rg14c | origin/grok/t/cxv23_g14c | pendiente |
| 3 | cxv23_rb04 | origin/grok/t/cxv23_b04 | pendiente |
| 4 | cxv23_rlos2 | origin/grok/t/cxv23_los2 | pendiente |

Cada tanda publica resultados/cxv23_r*.md en grok/resultados al terminar. Una tanda con wall muy por encima de ~100–110 s, OOM o procesos ajenos (registrados en rv23/<tanda>.mon) se marcará inválida.
