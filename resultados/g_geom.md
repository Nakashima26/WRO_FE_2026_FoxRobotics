# g_geom — Worker GEOM: parking sin U cuando la señal de entrada es verde (pared exterior + ancla ToF en reversa), base origin/main 2813d20 — FIN

- **Tipo de prueba: U-turn + parking** (harness `latencia/uno.sh`, cen_seeds n1/n2 × s1..48 = 96 corridas, `TWIN_MOTOR_REAL=1`, `TWIN_VBAT=11.5`, cámara desde calib, costo fijo 1.95, FF 2.8 / FD 3.8 del banco del box). Sólo se compara con tandas del mismo tipo (g_w_park_base / g1_base).
- **Base:** `g_w_park_base` (origin/main 2813d20, misma config exacta, corrida hace ~1 h, determinista = g1_base) reutilizada. Réplica de base en un worktree limpio de 2813d20.
- Cada tanda sola bajo `flock /workspace/fox/twin.lock`; procesos ajenos 0 al inicio, sin Killed/MemoryError. Bancos usados: 5 (2 de 96 + 3 réplicas de 48). Cribados previos con subconjuntos de las 30 corridas con U (gg_scr1..8), no son bancos.
- Métricas: `post` = total − 3 vueltas; U = "PARK U por pose" en fw_debug; GEOM = el ESP tomó el mando sin U.

## Cribado (antes de tocar código)
Base, 30/96 corridas hacen U (15 semillas × n1/n2). Por señal de entrada (primera de la recta del cajón en el sentido de marcha; ahí sólo hay asientos interiores T2/X1/T1):
| entrada | sentido | lado de paso | con U | sin U |
|---|---|---|---|---|
| verde | CCW | **por dentro** | 9 | 5 |
| verde | CW | por fuera | 2 | 18 |
| roja | CW | por dentro | 18 | 6 |
| roja | CCW | por fuera | 1 | 37 |
- **Ojo con la premisa "verde = paso por fuera": sólo vale en CW.** En CCW la verde se pasa por dentro (izquierda = isla). 9 de las 11 U con entrada verde son CCW por dentro.
- Pose al llegar (traza): en la mayoría de las U la ruta de la Pi ya deja al carro **sobre la línea de escaneo (ext 27–34 cm, rumbo ±10°) pasado el cajón** antes de que dispare la U (along ≥ max(500, pka+120)). Las que no: CW con roja también en T1 (s37: la ruta se queda por dentro), cajón al final de la recta (s26, pka≈390), y las 3 "por fuera" (n1/s32, n2/s16, n2/s20) donde la ruta se aleja de la pared por la T1 roja.

## Algoritmo nuevo (opt-in Pi+FW)
Con la U ya decidida, si la ruta deja el carro en la línea pasado el cajón (`pka+150 ≤ along ≤ pka+550`, `|ext−23| ≤ 12 cm`, `|err| ≤ 10°`):
1. **Fase 44**: frena con reversa (PWM 160) manteniendo el rumbo hasta < 120 mm/s.
2. **Fase 42** (la de PARK_REV_DIR, reutilizada): retrocede pegado a la pared (rumbo con corrección de altura a 300 mm, volante invertido), rápido (PWM 200) hasta `pka+100` y lento (100) después, **hasta que el ToF lateral ve la cara de atrás de la primera madera** (w del eje = 181 − 0.29·(h−300), ajustado con 21 corridas, rmse 3.8 mm).
3. **Fase 41**: adelante hasta w = 268 → coast → **plan normal del escaneo directo** (fase 40, sin U).
- Pi: `ROUTE_PARK_GEOM_EXT=1` manda `pge=1` sólo si la primera señal de la recta es **verde**, y sólo desde la esquina 12 con el carro ya pasado el cajón (así la línea al ESP no cambia en las vueltas 1–3). ESP: `PARK_GEOM_EXT` 0 = como antes, 1 = con pge, 2 = siempre (sin Pi).
- Si no ve la madera (eje a pka−280 o 6 s) vuelve a la U. Cambios idénticos en PurePursuit.ino y PurePursuitCarrera.ino.

## Resultados (96)
| tanda | cambio | ok | media | mediana | post todas | post (30 con U en base) | U | contactos | wall (s) |
|---|---|---|---|---|---|---|---|---|---|
| g_w_park_base | main 2813d20 | 95/96 | 42.22 | 41.94 | 9.276 | 12.090 | 30 | 1 (n1/s36, vuelta 1) | 133.3 |
| **g_geom_verde** | **Pi+FW, sólo entrada verde** | **95/96** | **42.08** | 41.94 | **9.136** | **11.648** | **21** | 1 (el mismo) | 134.2 |
| g_geom_any | FW sólo (PARK_GEOM_EXT=2), cualquier color | 95/96 | 42.05 | 41.94 | 9.108 | 11.560 | 15 | 1 (el mismo) | 133.2 |
- verde vs base pareado (95): −0.140 s media; las 9 GEOM (s3, s5, s35, s36, s45) −0.40 a −2.28 s, todas ok. n2/s16 (−0.08) y n2/s20 (+0.72, sigue con U) cambian sólo por el pge extra en la línea tras la vuelta 3 (caos del gemelo; sus vueltas 1–3 son idénticas). Resto idéntico corrida a corrida.
- any: 15 GEOM; verdes como en verde (n2/s45 −2.07 vs −1.60: allí el pge cambia el timing); rojas s19 −0.21/−0.82, s46 +0.03/−0.82, **s39 +0.50/−0.20** (llega con h 335–380 y el plan sale más lento). 0 más lentas fuera de s39/s46.
- Sin contactos nuevos, 0 pass_side_violations, 0 contactos de señal. La nariz llega como mucho a along ~750 (la primera cinta de la esquina queda a ~900 a esa altura): no cruza líneas.

## Réplica n3–n8 de las semillas que cambian (s3 s5 s35 s36 s45 + rojas s19 s39 s46 = 48)
| tanda | ok | media | post | U | contactos | wall | válida |
|---|---|---|---|---|---|---|---|
| g_geom_base_rep (worktree limpio 2813d20) | 47/48 | 45.14 | 12.108 | 47 | 1 (n5/s36) | 74.7 | sí |
| **g_geom_verde_rep** | **47/48** | **44.25** | **11.213** | **18** | 1 (el mismo) | 75.2 | sí |
| g_geom_any_rep | 47/48 | 44.12 | 11.089 | 1 | 1 (el mismo) | 187.9 | **NO** (carga ajena: mem 13.6 GB, mean_pi_frame_ms 4.64 vs 3.8) — sólo indicativa |
- verde_rep: 29 GEOM (todas las verdes), −0.895 s pareado, **ninguna más lenta**; verdes post 12.80 → 11.35. Rojas idénticas a base.
- any_rep (indicativa): 46 GEOM, −1.02 s, s39 +0.17/+0.55/+0.69 en n4/n7/n8.

## Decisión
**Aceptada (opt-in, apagada por defecto)**: `grok/t/g_geom` = 1 commit sobre 2813d20, **6962d96** (Jessebnda, sin co-autor). Para activarla: Pi `ROUTE_PARK_GEOM_EXT=1` + ESP `PARK_GEOM_EXT=1` (en el carro es `const`: cambiar el valor o vía exportar_carrera). Hay que flashear.

## Descartado en el camino (cribados sobre las 30 U, no commiteado)
- pge desde el arranque (línea más larga en todo el race): cambia las vueltas 1–3 por timing UART (n1/s19 chocó en vuelta 3) → ahora sólo tras la esquina 12 y pasado el cajón.
- Ancla fija w=166 (fórmula vieja de PARK_REV_B*, calibrada para el sonar antes del ToF v2): error +60 mm → choque con cajón; fija 166 sin cono: ±15 mm → 5/21 choques. Con 181 − 0.29(h−300): 0.
- Reversa rápida sin zona lenta amplia: 9/30 no ven la madera y caen a U desde la reversa → choque con señal. Zona lenta desde pka+100 + fin de búsqueda por along (pka−280, no por w del prior): 0.
- Tomar el mando hasta pka+700: con 600–640 la reversa larga sale más lenta que la U (5/6) → tope 550.
- Tolerancia 8 cm / 8°: sólo 5/30 toman el mando (la ruta deja ext 29–34) → 12 cm / 10° con corrección de altura en la reversa.
- No repetido: t13 corredor/arco, t14 reversa en S, t26 PARK_REV_RECTA, t15 U perfil.

## Pendiente / riesgos en el carro real
- Frenar con contramarcha a ~600–740 mm/s (fase 44) y reversa a ~500 mm/s pegado a la pared: revisar en el carro (opción: coast + reversa).
- `PARK_GEOM_W_REV` y `_KH` salen del gemelo (ToF lateral a 110.6 mm del eje): medir en el carro.
- s39 (roja, cajón al final, llega alto): por eso sólo verde.
- Corridas con U que quedan: CW por dentro con cajón al inicio (pka≈−196, la ruta llega a la pared en pka+600: reversa demasiado larga), s37 (roja-roja), s26 (cajón al final), y las 3 por fuera donde la ruta se va de la pared.

FIN
