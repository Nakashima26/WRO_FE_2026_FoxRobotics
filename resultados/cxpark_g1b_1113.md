# CANCELADA cxpark_g1b_1113 (relanzamiento 19:16 PST, PARK_PL_GUARD0=1, commit a1e5c8f)

Tipo: ESTACIONAMIENTO (carrera completa desde el cajón con estacionamiento), banco con latencia (costo.py/uno.sh, FF 2.8 / FD 3.8, primer plan 1.95 s, VBAT 12.5, ROUTE_PLAN_ASYNC, 96 carreras, P=7).

**Estado: CANCELADA el 2026-11-13 ~19:20 PST por orden de seguimiento PARK** (estaba activa: tenía /workspace/fox/twin.lock y bloqueaba la cola). Árbol identificado por cmdline y cwd (/workspace/fox/cxpark_g1b_1113_wt/...): bash -c 254727 -> flock 254740 -> run.sh 254741 -> xargs 254746 -> uno.sh/costo.py hijos; SIGTERM solo a esos PIDs (y a un uno.sh/costo.py rezagado del mismo tag, 257752/257758). Sin pkill/killall. No se tocó ningún proceso de otro worker. No se ejecutó cxpark_g1b_1113_md.py ni post.sh tras la cancelación.

Parcial (75 de 96 carreras con salida; las últimas de n2 cortadas por la cancelación cuentan como falla):

    cxpark_g1b_1113: 50/75 ok  media 37.59  mediana 37.27  mejor 33.85  fallas: n1/s22 n1/s23 n1/s26 n1/s33 n1/s34 n1/s35 n1/s36 n1/s4 n1/s42 n1/s5 n1/s7 n1/s9 n2/s10 n2/s19 n2/s2 n2/s21 n2/s22 n2/s23 n2/s24 n2/s25 n2/s26 n2/s4 n2/s5 n2/s7 n2/s9

**INVÁLIDA / no comparable**: corrió en paralelo con otra tanda ajena (gb6, 7 procesos costo.py extra) con carga ~12. Sustituida por cxpark_rg1_1113 (repetición aislada).
