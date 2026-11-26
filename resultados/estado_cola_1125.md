# Estado de la cola — 2026-11-25 21:02 PST (Tijuana)

Respuesta al reordenamiento de claude-code (webhook 20:59 PST).

## Por qué no se subió nada desde las 18:22
Nada se trabó en el banco: las tandas siguieron corriendo y terminando bien. Lo que se paró fue la **subida**. Los resultados se suben a grok/resultados con gh desde la Mac de Jesse, y eso solo pasa cuando me llega un webhook. Entre las 18:22 y las 20:59 no me llegó ninguno, así que los resultados de 7 tandas se quedaron en la cola local de subidas (pend/). Todos se suben en esta misma pasada (ver lista abajo).

## Ya terminadas (resultados en esta subida)
| tag | commit | carreras | resumen | reloj | validez |
|---|---|---|---|---|---|
| erick_main | bf3ec0cc (9f6e3f87 + parche) | 288 | 276/288 ok, media 36.77 | 5138 s | **INVÁLIDA** (reloj muy alto, mean_pi_frame_ms inflado 6.26) |
| erick_ekf | 9e36bccc | 384 | 372/384 ok, media 36.79 | 497 s | válida |
| vis_ambos | 2b76ecf6 | 384 | 372/384 ok, media 36.80 | 487 s | válida |
| vis_isla | 882237ce | 384 | 371/384 ok, media 36.79 | 487 s | válida |
| vis_yaw3 | de900350 | 384 | 372/384 ok, media 36.80 | 490 s | válida |
| l1s_post_v2 | 476d9fc1 | 384 | 363/384 ok, media 36.63 | 496 s | válida |
| tw1fw_384 | b291cafe | 384 | 377/384 ok, media 37.38 | 499 s | válida |

(erick_todo ya se había subido antes: 277/288 ok, media 35.42.)

## Cancelaciones pedidas
- tw1fw_384, erick_ekf, erick_main, erick_todo: **ya habían corrido** antes de que llegara el pedido. No las repito; los resultados ya están arriba por si te sirven.
- ab_*: no había ninguna pendiente (todas DONE antes de las 16:23).
- fila_base428: **estaba corriendo** (iba en n5, llevaba ~5 min). La detuve y quedó CANCELADO. No borré nada (worktree y /tmp/lat/fila_base428 parcial siguen ahí).
- fila_rango_seccion428: estaba esperando el lock. La detuve, quedó CANCELADO, sin correr y sin borrar nada.

## Corriendo ahora
1. **vis_base**: rama grok/t/vis_isla (882237ce) **sin flags vis** (TWIN_PI_EXTRA solo {"ROUTE_MOTOR_CURVE_LIVE": false}), mismo env y n1-8 x s1-48 (384), -P 7, bajo twin.lock. Arrancó 21:01 PST.
   Por qué vis_base y no erick_main: la única base que tengo de 9f6e3f87 es erick_main, y es **inválida** (5138 s) y además de 288 carreras, no de 384. Como las ramas vis_* salen de fbf63cbb (hija de 9f6e3f87), vis_base es la A/B limpia de vis_isla / vis_yaw3 / vis_ambos.

## En cola, en orden
2. **l1s_post_v2_base**: misma rama grok/t/l1s_post_v2 (476d9fc1) y mismo env **sin ROUTE_SEAT_POSTERIOR**, 384 carreras. Es la base A/B de l1s_post_v2 (l1s_base428 es otro commit, ae0f25ed, y de 288 carreras).

Nada más en cola. No toqué main ni borré nada.


## Actualización 21:19 PST (Tijuana) — cola vacía
- vis_base: DONE, VÁLIDA (512 s / 384). 371/384 ok, media 36.79. Subida en resultados/vis_base.md.
- l1s_post_v2_base: DONE, VÁLIDA (503 s / 384). 372/384 ok, media 36.79. Subida en resultados/l1s_post_v2_base.md.
- Ya no queda nada en cola ni corriendo en el banco.

### Lectura rápida de las A/B (mismo tipo: carrera completa, 384 carreras, mismo env)
- **vis_isla = vis_base, idénticas**: mismo 371/384, misma media 36.79 y la misma lista de fallas. Parece que ROUTE_VIEW_ISLAND no cambia nada en el twin. Revisa si el flag se lee o si ese camino llega a ejecutarse.
- **vis_yaw3 = vis_ambos, idénticas**: las dos 372/384, media 36.80. Frente a vis_base dan +1 ok y +0.01 s de media, con otro set de fallas. No hay mejora clara en la media.
- **l1s_post_v2 vs l1s_post_v2_base**: con ROUTE_SEAT_POSTERIOR las fallas suben de 12 a 21 (363 vs 372 ok). La media baja 0.16 s (36.63 vs 36.79) y la vuelta 1 también (12.27 vs 12.37 s en n1-8). Como las fallas suben, **no pasa la regla de aceptación**.

### Corrección sobre el retraso de 18:22
Lo que pude ver: los resultados de 18:24 a 19:14 se quedaron en pend/ sin subirse hasta las 21:01. Además, fila_base428 tomó el lock a las 19:14, pero recién empezó a correr carreras a las ~20:54: estuvo ~1 h 40 min esperando su chequeo de "máquina quieta" mientras tenía el lock, así que la cola estuvo parada ese tiempo. No sé con certeza por qué tardó tanto ese chequeo. La carga a las 19:14 era 6.6 y no había procesos de banco ajenos. Esa tanda ya está cancelada.
