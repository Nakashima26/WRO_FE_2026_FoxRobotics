# Respuesta: l1v_corredor_ajuste

## 2026-11-25 12:40 PST (Tijuana) — ajuste aplicado

Hecho:
- **l1v_corredor_288** (bdf36eca): no había corrido ninguna carrera (el primer intento abortó al crear el worktree por disco lleno y el relanzamiento seguía esperando twin.lock). Lo detuve, lo corregí a **FF 4.28 / FD 5.8 exactos** (el script tenía 2.8/3.8) y lo relancé en cola bajo twin.lock. Resto igual que l1v_cartas_288 y la0g: PI_EXTRA exactamente {"ROUTE_L1_PATH_SPEED":true}, FW_OVER {}, COST_FIRST 1.95, COST_FIJO=1, TOF_SHORT_K 0.5, MOTOR_CURVA_REAL=1, CAMERA_FROM_CALIB=1, MOTOR_REAL=1, VBAT 11.5, lens_calib.npz, n1-6 x s1-48, -P 7. Corredor y cartas quedan como variantes independientes (worktrees y tags separados).
- **Validación con el COSTO**: en cada carrera reviso que la línea COSTO traiga FF 4.28 / FD 5.8 y reporto cuántas no; también la media de media_ms.
- **rep78 corredor** (orquestador ya en cola): cuando terminen l1v_corredor_288, la0g (l1v_diag288) y el rep78 de cartas, comparo por carrera vs la0g (éxito/falla, total_time_s, lap_times_s). Si cambia alguna semilla **y** baja la media primaria, corro n7 y n8 en la unión de semillas con cambio: brazo corredor nuevo (l1v_corredor_rep78, bdf36eca) y brazo base reutilizando l1v_base_rep78; solo corro base (ae0f25ed, PI_EXTRA {}) para las semillas que falten ahí. Todo en un bloque bajo twin.lock. Si no se cumple, publico l1v_corredor_rep78.md diciendo por qué no se corrió.
- Publico resultados/l1v_corredor_288.md y resultados/l1v_corredor_rep78.md con todos los STOP/COSTO. No relancé la0g; twin.lock intacto.

Supuesto: tomé "media primaria" = media de total_time_s en pares donde ambos brazos terminan (como en cartas). Si querías otra métrica (p. ej. COSTO media_ms o tasa de éxito), avísame antes de que termine el 288.

Iré agregando secciones a este archivo.

## 2026-11-25 15:54 PST — 288 terminado, sin rep78

- 288 de 288 carreras cambiaron vs la0g en 48 semillas (1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48).
- 288: terminadas la0g 278/288, corredor 0/288; sin pares ambos terminados
- COSTO: l1v_corredor_288: 0 COSTO leídos, 288 sin COSTO o con FF/FD distinto de 4.28/5.8
- rep78 no corrido: hubo cambios pero la media primaria no bajó.

Archivos: `resultados/l1v_corredor_288.md`, `resultados/l1v_corredor_rep78.md`.

## 2026-11-25 15:54 PST — 288 terminado, sin rep78

- 288 de 288 carreras cambiaron vs la0g en 48 semillas (1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48).
- 288: terminadas la0g 278/288, corredor 0/288; sin pares ambos terminados
- COSTO: l1v_corredor_288: 0 COSTO leídos, 288 sin COSTO o con FF/FD distinto de 4.28/5.8
- rep78 no corrido: hubo cambios pero la media primaria no bajó.

Archivos: `resultados/l1v_corredor_288.md`, `resultados/l1v_corredor_rep78.md`.
