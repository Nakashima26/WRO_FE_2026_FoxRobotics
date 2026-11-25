# Respuesta: l1v_ajuste

## 2026-11-25 12:35 PST (Tijuana) — corrección aplicada

Hecho:
- **l1v_cartas288** (sin guion bajo, bbfab1fc): cancelado antes de arrancar (estaba esperando twin.lock). No corrió ninguna carrera.
- **l1v_cartas_288**: conservado y fijado a **fa7813a2**. Corregido a **FF 4.28 / FD 5.8 exactos** (mi acuse decía 2.8/3.8; ya no). Resto: COST_FIRST=1.95, TWIN_COST_FIJO=1, PI_EXTRA {"ROUTE_L1_CARD_SPEED":true}, FW_OVER {}, TOF_SHORT_K 0.5, MOTOR_CURVA_REAL=1, CAMERA_FROM_CALIB=1, MOTOR_REAL=1, VBAT 11.5, lens_calib.npz, n1-6 x s1-48, -P 7. Relanzado en cola bajo twin.lock (aún no arrancó).
- **la0g** (ae0f25ed): todavía no había arrancado, así que lo cancelé. Cuando termine **l1v_diag288** (63886cd9, padre 0ee69022; árbol de ae0f25ed = árbol de 0ee69022, verificado), publico sus STOP/COSTO como l1v_diag288.md y también como la0g.md con nota de origen. La sonda ya tenía FF 4.28 / FD 5.8, PI_EXTRA {}, FW_OVER {}, COST_FIRST 1.95.
- **rep78**: cuando ambas terminen, comparo por carrera (éxito/falla, total_time_s, lap_times_s) l1v_diag288 vs l1v_cartas_288; para las semillas con algún cambio corro ambos brazos en n7 y n8 con las mismas variables: **l1v_base_rep78** sobre ae0f25ed (costo.py normal, PI_EXTRA {}) y **l1v_cartas_rep78** sobre fa7813a2, en un solo bloque bajo twin.lock.
- No interrumpí otras tareas; twin.lock intacto.

Ojo: la base 288 sale de la sonda (l1v_diag.py en lugar de costo.py). Si l1v_diag.py cambia la simulación además de registrar, avísame y corro la0g normal.

Iré agregando secciones a este archivo (la0g publicado, rep78 terminado). Se publican todos los STOP/COSTO en resultados/ como siempre.

## 2026-11-25 13:20 PST — FALLO

l1v_diag288 terminó con estado `ABORTA_WT`; no seguí con la0g/rep78. Revisa y reenvía.
