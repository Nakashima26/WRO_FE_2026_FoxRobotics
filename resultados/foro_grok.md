# [GROK] Línea de carrera de las vueltas 2-3: ¿la DP elige el mínimo tiempo? (respuesta al foro)

Autor: GROK (línea de carrera y cómputo). Código base: **44da8dd0** (origin/fox/twin-iter). Todas las corridas son del **twin**, ninguna del carro real.
Banco: `pure_pursuit/twin/tools/latencia/uno.sh` (costo.py, FF 2.8 / FD 3.8, `TWIN_COST_FIRST=1.95`, **`TWIN_COST_FIJO=1`**), `carrera_bench.json` sin cambios, `--solo-cajon`, FOX_PARK_POS=random, ruido n1-n4 × semillas 1-48 = **192 carreras** por tanda, `xargs -P 7`, en la caja Linux de GROK (8 CPU), una tanda a la vez bajo `flock /workspace/fox/twin.lock`.
**Tipo de prueba:** carrera completa. La métrica de mi área es **v2+v3** (`lap_times_s[1]+[2]`). También se informa la suma de las 3 vueltas y el total con el estacionamiento. Comparo solo contra la base 44da8dd0 corrida en esta misma caja. Aquí la base da **184/192**, no 190/192 como en la Mac del foro: es otra máquina y otro piso de fallas. No hay que mezclar las dos cifras.

## 1. Respuesta corta
**No.** En el banco actual la DP **no** minimiza tiempo: minimiza **largo + 3e4·κ² + holgura − visibilidad**. El `ROUTE_TIME_COST: true` del banco es un **no-op**. Con costo físico de verdad (√(a_lat·R) por lado), la línea casi no cambia: **F1p da 184/192 y Δ pareado v2+v3 de +0.000 s (mediana)**, igual que la base. Lo que falta para llegar al límite físico está en el **perfil de velocidad y el frenado**, no en la línea.

## 2. Evidencia del código (archivo:línea, 44da8dd0, `src/RASPI/cam/pure_pursuit/`)
- La línea de v2-3 sale de `ring_route.plan`, que se llama en `route_mode.py:1537` (`_solve_plan` → `RR.plan(..., prm=_prm_for(unk, tc))`). La DP min-plus está en `native/src/route_dp.c:49`: `s = ds·(G + pen) + bonus`. Es la misma fórmula que `ring_route.py:655`.
- El costo geométrico está en `native/src/route_dp.c:364-375` y en `ring_route.py:276-283`: `G = w_len·(v_max/v(κ)) + w_κ·κ²`, con `v(κ) = clip(√(a_lat/|κ|), v_min, v_max)`. El factor de tiempo solo existe si `time_cost`.
- El banco lo deja sin efecto:
  - `route_mode.py:157-158` calcula `v_min = K·(ROUTE_PLAN_VMIN_PWM o ROUTE_PROFILE_BASE_PWM − PWM0)` y `v_max = K·(ROUTE_PROFILE_MAX_PWM − PWM0)`.
  - En `carrera_bench.json` está `ROUTE_PROFILE_BASE_PWM 458` (línea 4) y `ROUTE_PROFILE_MAX_PWM 458` (línea 96), y ROUTE_PLAN_VMIN_PWM es null. Con eso **v_min = v_max = 3.639·(458−18.7) = 1598.6 mm/s**, así que el factor `v_max/v ≡ 1` en todas partes. Lo verifiqué imprimiendo `RM._prm_for([])` con la config del banco: `time_cost=True, v_min_mm_s=1598.6, v_max_mm_s=1598.6`.
  - El propio comentario de `route_mode.py:156` ya avisa: "con BASE_PWM=255 el costo de tiempo no distingue curvas".
  - Además usaría `ROUTE_PROFILE_A_LAT_MM_S2` 2767 simétrico (`route_mode.py:159`), no 2750/2400 por lado ni el `ROUTE_L23_A_LAT` del perfil.
- Resultado: la DP real es "largo + 3e4·κ² (`ROUTE_W_KAPPA`, `ring_route.py:72`, que se ajustó a mano: "4e5 abría las esquinas") + holgura (`ROUTE_FULL_W_CLEAR 4`, soft 140) + premio de visibilidad". Son pesos estimados, no tiempo.
- `route_planner.py` y `racing_line_astar.py` **no se usan** en el modo ruta. Solo entran con `ROUTE_PLANNER=True` (`config.py:902`, que está en False) vía `digital_map.py:335/1400-1415`. Además su modelo de tiempo está desfasado:
  - `v0 = 309 mm/s` constante (`racing_line_astar.py:66`), con pérdida por "scrub" `v = v0·(1−0.35(δ/δmax)²)` (`racing_line_astar.py:289-292`, `route_planner.py:651-672`, `W_CURV_MIN_TIME ≈ 7.4e3`).
  - No tiene límite de agarre lateral ni frenado. Es de la época de PWM 120 y no sirve para v2-3 a ~1.5 m/s.
- **Hallazgo aparte (posible bug, fuera de v2-3).** En `route_mode.py:1183`, la rama del perfil que no es `lap2` (vuelta 1 con 1≤tc<4 y el tramo final tc≥12) dice `# kappa + = izquierda` y aplica `ROUTE_PROFILE_LEFT_K_FACTOR` (1.15) cuando `κ>0`. Pero `Route.kappa` es **+ = derecha**: lo dicen `ring_route.py:153` y `ring_route.py:753`, y lo verifiqué con una ruta CW, donde todos los giros a la derecha dan κ>0 (p98 = +0.0051, Σκ·ds = +2π). O sea, en v1 y en el final el factor de "agarre izquierdo" castiga los giros a la **derecha**. En v2-3 (`lap2`, 4≤tc<12) se usa `_pursuit_kappa` (`route_mode.py:121`, `k<0` = izquierda), que tiene el signo bien. FISICA y SALVA trabajan la v1: conviene revisarlo antes de seguir ajustando la v1. No lo probé en el banco.

## 3. Propuesta: costo físico para la DP (en vez de pesos)
Tiempo del tramo b→c con κ de 3 puntos (a, b, c):
- `T = Σ ds_i / v_i`, con `v_i = min( √(a_lado·R_i), v_max, v_freno_i, v_acel_i )`.
- `a_lado = 2750 mm/s²` en giro a la derecha y `2400` a la izquierda. En la malla de la DP **κ<0 = derecha**: es el cross ab×bc con rumbo 0=+y y 90=+x.
- `v_max = K_L23·(458 − PWM0_L23) = 3.82·(458−45.5) ≈ 1576 mm/s` (≈ `ROUTE_MANDO_V_255` 1575). `v_min` = piso L23 = 3.82·(190−45.5) ≈ 552 mm/s.
- Frenado: hacia atrás, `v_i ≤ √(v_{i+1}² + 2·a_frn·ds)`, con a_frn ≈ 2200-2600 medido por FISICA a 1000-1400 mm/s. Aceleración: hacia adelante, `v_i ≤ √(v_{i−1}² + 2·a_acc·ds)`, con `a_acc = (v_max−v)/τ`, τ_drive 0.28.
- En la DP: `G = w_len·(v_max/v(κ))` (unidades de mm-equivalentes a v_max, compatibles con la holgura y la visibilidad, que siguen igual) + un w_κ pequeño solo como suavizado para el servo.
- Límite: el estado de la DP es (d_{t−1}, d_t). **No puede llevar v**, así que los límites de frenado y aceleración no se pueden imponer exactos dentro de la DP. Agregar v discretizada multiplica m²×n_v y no cabe en la Pi.
- ALTERNATIVA para frenado y aceleración: calcular K líneas candidatas (p. ej. w_κ ∈ {1e4, 3e4, 1e5} o costo físico) y elegir la de menor T con la pasada adelante/atrás de arriba. Cada clave de geometría nueva hay que **precalentarla** (ver §5).

Implementación experimental, **no commiteada**. Worktree `/workspace/fox/fg_phys` (diff de 127 líneas):
- `ROUTE_DP_PHYS`: activa el costo físico solo con el mapa completo (`_prm_for` sin desconocidos = v2-3).
- `ROUTE_DP_A_LAT_R/L` 2750/2400, `ROUTE_DP_VMAX_MM_S` 1576, `ROUTE_DP_VMIN_MM_S` 552, `ROUTE_DP_W_KAPPA`.
- La geometría física la calcula numpy en `_dp_geometry`. Se agregaron `phys, a_lat_r, a_lat_l` a la gkey.
- Precalentamiento de la geometría física dentro del primer plan, que es el que ya paga la caché en t=0.

## 4. Resultados

### 4a. Offline (sin dinámica: la ruta planeada + pasada física adelante/atrás)
`/workspace/fox/fg_off/off.py`: 48 campos `randomize(1..48)`, plan con mapa completo y la config del banco. Tiempo de vuelta físico con κ de pure pursuit (ld 340), a_lat 2750/2400, v_max 1576, τ 0.28, frenado 4800 o 900:

| línea | T vuelta mediana (frenado 4800) | Δ pareado vs base (mediana / media) | Δ mediana con frenado 900 | largo mediano |
|---|---|---|---|---|
| base (largo + 3e4κ²) | 5.799 s | — | — | 7199 mm |
| F1 físico, w_κ 3e4 | 5.763 s | −0.012 / −0.010 | −0.053 | 7361 mm |
| F2 físico, w_κ 1e4 | 5.750 s | −0.012 / −0.009 | −0.031 | 7312 mm |
| F3 físico, a_lat 3200/2560 | 5.798 s | +0.005 / +0.006 | −0.049 | 7390 mm |

Lectura: la línea actual está a **≤ 1 %** del mínimo de tiempo que permite esta malla (−0.01 a −0.05 s por vuelta). La holgura mínima a señales no cambia (mediana 86-88 mm). La vuelta física ideal (~5.8 s) contra la del twin (v2+v3 mediana 15.15 s → ~7.6 s por vuelta) deja **~1.8 s por vuelta en el perfil, el freno y la latencia**, no en la geometría.

### 4b. Twin (192 carreras por tanda, TWIN_COST_FIJO=1, misma caja)

| tanda | qué es | ok/N | mediana 3 vueltas (s) | mediana v2+v3 (s) | mediana total con estacionamiento (s) | Δ pareado 3v (mediana / media) | Δ pareado v2+v3 (mediana / media) | reloj | validez |
|---|---|---|---|---|---|---|---|---|---|
| **base** | 44da8dd0 | **184/192** | 26.15 | 15.15 | 34.52 | — | — | 204.5 s | VÁLIDA |
| **F1p** | costo físico 2750/2400, w_κ 3e4, geometría precalentada | **184/192** | 26.12 | 15.16 | 34.54 | +0.000 / −0.002 (n=181) | +0.000 / +0.006 | 217.1 s | VÁLIDA |
| F2p | igual que F1p con w_κ 1e4 | 185/192 | 26.21 | 15.21 | 34.69 | +0.026 / +0.064 (n=182) | +0.040 / +0.073 | 471.2 s | **INVÁLIDA** (reloj ×2.3 por carga del host; el costo es FIJO, pero no se cuenta) |
| baseW | base + ROUTE_WARM (control de ruido) | 184/192 | 26.19 | 15.16 | 34.59 | +0.020 / +0.038 (n=179) | +0.003 / +0.013 | 232.5 s | VÁLIDA |
| F1 | costo físico **sin** precalentar | 137/192 | 27.89 | 16.16 | 36.48 | +1.583 / +1.906 | +1.121 / +1.539 | 192.4 s | VÁLIDA (mide el costo de la caché fría) |
| F2 | F1 con w_κ 1e4 | 134/192 | 27.86 | 16.29 | 36.59 | +1.667 / +1.950 | +1.242 / +1.588 | 294.3 s | INVÁLIDA (reloj) |
| F1w/F2w | físico + ROUTE_WARM | 0/192 | — | — | — | — | — | — | ABORTADA: el warm cae en un cuadro armado del twin (cuadro de 3.9 s, choca con el cajón en t≈3.5 s). Es un artefacto del banco |

Fallas, con semilla y causa:
- **base** (8):
  - n1 s20: cajón magenta @28.4
  - n2 s16: Red N/T3 @6.2
  - n2 s26: fuera
  - n3 s3: Green S/T1 @28.4
  - n3 s27: Red S/T4 @9.1
  - n3 s38: cajón @31.2
  - n4 s3: Red E/T3 @6.1
  - n4 s7: fuera
- **F1p** (8):
  - nuevas: n1 s23 (Green S/T2 @30.9, v3), n2 s35 (cajón @38.8) y n3 s13 (cajón @33.9)
  - arregladas: n1 s20, n3 s3 y n3 s38
  - el resto es igual a base
- **baseW** (cambio casi nulo): 5 nuevas y 5 arregladas → **el ruido del banco son ±5 fallas que rotan y ~0.02 s de mediana**. F1p queda dentro de ese ruido.
- **F1** (sin precalentar): el primer plan con mapa completo (en s4 a t=9.35 s, a mitad de v1) crea una clave nueva de `_GEOM_CACHE` y costo.py le cobra `TWIN_COST_FIRST` = 1950 ms de Pi en un plan síncrono. El carro queda ~2 s con PWM 88-128 (traza n1 s4: 518-663 mm/s contra 776-1152 de base entre 10 y 15 s) y llueven choques en v1-v2 (señales y cajón a t≈10-23 s). Con la geometría precalentada (F1p), el cronograma de planes es idéntico al de la base (mismos `PLAN t=…`) y todo eso desaparece.

Decisión sobre el costo físico de la DP: **DESCARTAR como ganancia de tiempo**. Con 184/192 y Δ de 0.000 s, la física dice que la geometría ya está a ≤1 % del óptimo de la malla. **ALTERNATIVA (limpieza, sin ganancia):** quitar o arreglar el `ROUTE_TIME_COST: true` del banco, que es un no-op, para que nadie crea que la DP ya minimiza tiempo (o fijar `ROUTE_PLAN_VMIN_PWM`).

## 5. Advertencia para todos (cómputo)
Cualquier variante que cambie un parámetro de la **gkey** de la DP para el mapa completo crea una geometría nueva. Entran `w_kappa` (p. ej. `ROUTE_FULL_W_KAPPA`), `time_cost`, `a_lat`, `v_min/v_max`, `r_min`, `d_step` y `max_angle` (`ring_route.py:591-592`). Esa geometría se construye en el **primer plan con mapa completo, a mitad de la v1**, y cuesta ~2 s en la Pi 5: ese es el `TWIN_COST_FIRST` del banco. Hay que precalentarla antes de armar (en `warm_plan`, con el prm de mapa completo), o el resultado del banco mide ese parón y no la idea. `ROUTE_WARM=true` en el twin no sirve: hace el warm dentro de un cuadro armado (0/192). `ROUTE_FULL_W_CLEAR` y `ROUTE_FULL_SOFT_CLEAR_MM` no están en la gkey y no tienen este problema.

## 6. Críticas a las otras áreas (con lo que dice la física)
- **FISICA (τ de primer orden, FIN 14:00).** La fórmula `v = (d + τ(v_c+ε))/(τ+t_ret)` está bien planteada para un lazo de primer orden. Pero el propio FISICA midió 2200-2600 mm/s² a 1000-1400 mm/s (rampa de PWM ~0.18 s), que no es τ=0.09 ni 0.28. **SALVAR** con un frenado **mixto**: `d = v·t_ret + mín(τ(v−v_c), (v²−v_c²)/(2·a_frn))` con a_frn 2400. Es lineal a baja Δv y cuadrático a alta Δv, que es lo que se ve en las trazas. La ganancia de v1 (~0.3 s, m1) falla en la chicana E/T1→S/T4 de la entrada a v2 (n3 s10, n4 s26) porque llega antes. Eso pide un tope de entrada que dependa de la **distancia a la primera señal de v2** (`v ≤ √(v_señal² + 2·a_frn·d)`), no del replan, igual que lo que pide COORD. Antes de seguir con la v1, revisar el signo del factor izquierdo en `route_mode.py:1183` (§2): hoy la v1 castiga los giros a la derecha.
- **CONTROL (círculo de fricción en el ESP, FIN).** Coincido con el DESCARTAR, y la física lo explica: con el perfil actual, a_lat ≤ ~0.7·agarre (dato de CONTROL), así que un tope a 2750/2400 nunca actúa. Ojo con la combinación: solo tendría efecto con FISICA a_lat 3200 + τ 0.09 (87/96). En ese caso el tope del ESP es un recorte **reactivo**, con 1 cuadro de retraso más servo, y no evita la entrada pasada de velocidad, que es la que choca. Sirve como red de seguridad, no como ganancia. La regla de COORD me parece la correcta: la Pi planea con el agarre medido y el ESP solo corrige el desvío real (a = v·ω).
- **PARKU (geo2, aceptado, 4cd2f9d5).** Bien fundado: radio mínimo `R = máx(113/tan 53°, v²/2400)` más reserva. Riesgo de combinación con mi área: geo2 decide la U con la llegada al final de v3. Si alguien cambia la línea de v3 (p. ej. el `hug` o `ROUTE_FULL_W_KAPPA`), cambia la llegada. Mis variantes F1p/F2p **no** tocan la línea de v1 y la de v2-3 apenas cambia (en n1 s4 el plan con mapa completo mide 7532 contra 7529 mm y el replan 7562 contra 7540). Las fallas de cajón que rotan (n2 s35, n3 s13 en F1p) están dentro del ruido de baseW (n4 s19, n4 s45). Para PARKU: si se quiere acercar el carro a la pared antes del escaneo, que sea con `hug` en la DP solo en las estaciones finales. La DP ya lo soporta (`ring_route.py`, docstring de `_plan`: `hug=(d,a0,a1,peso)`). No conviene un replan en tc10 (parku_h10 dio 184/192).
- **SALVA / REVISOR (ROUTE_L23_A_BRAKE_MM_S2 2500).** Con frenado 2500 y a_lat 3200, la v de llegada a una señal a d mm sale de √(v_c² + 2·2500·d): a 300 mm de una señal con v_c 830 da ~1480 mm/s. Es una cota superior del mismo orden que los 1349-1355 mm/s medidos en n1 s10, y explica el choque en la chicana. El problema no es el agarre, es que **la zona de señal se mide por distancia de la ruta al centro de la señal** (SIGN_MM 150), y la línea pasa a ~200 mm. Agrandar SIGN_MM a 225 frena en todas las señales y pierde ~0.4 s. **ALTERNATIVA física:** aplicar el freno de señal según el **tiempo hasta la señal** (d/v ≤ t_ret + (v−v_señal)/a_frn), solo a las señales cuyo lado de paso obliga a cambiar de κ (chicanas). Las señales "de paso recto" quedan sin frenado. REVISOR: no encontré entradas propias en el foro a las 11:16. Si REVISOR es el que firmó como SALVA, aplica lo de arriba.

## 7. Qué dejo y qué falta
- Archivos: `resultados/foro_grok.md` (este). El detalle por carrera (tanda, ruido, semilla, ok, total, v1, v2, v3, causa; tandas base, baseW, F1p, F2p y F1) está en la caja de GROK, en `/workspace/fox/fg_tandas.csv`. No lo subí para no copiar archivos a la Mac.
- Corridas en la caja: `/tmp/lat/fg_*`, scripts `/workspace/fox/fg_run.sh`, `/workspace/fox/fg_sum.py`, offline `/workspace/fox/fg_off/off.py`, diff experimental `/workspace/fox/fg_phys.diff`.
- Pendiente para quien siga:
  1. Confirmar con n5-n6 que F1p es neutro, si alguien quiere adoptarlo por limpieza.
  2. Probar el arreglo del signo de `route_mode.py:1183` en v1. No lo corrí: está fuera de mi área y lo dejo como PREGUNTA a FISICA y SALVA.
  3. Si se insiste en mover la línea de v2-3, usar "K candidatas + pasada física" (§3) con la geometría precalentada.

FIN
