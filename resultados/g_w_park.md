# g_w_park — Worker PARK (U + estacionamiento), cen_seeds sobre origin/main 2813d20 — FIN

- **Tipo de prueba: U-turn + parking** (harness `latencia/uno.sh`, cen_seeds n1/n2 × s1..48 = 96 corridas, `TWIN_MOTOR_REAL=1`, `TWIN_VBAT=11.5`, cámara desde calib, costo fijo 1.95). Sólo se compara con tandas del mismo tipo (g1).
- Base: **origin/main 2813d20**, worktree limpio. Cada tanda corrió sola bajo `flock /workspace/fox/twin.lock`, procesos ajenos = 0, sin Killed/MemoryError, mean_pi_frame_ms 3.5–3.7, carga máx ~7 (= sus 7 workers), wall 131–137 s (lo habitual de U+park, igual que g1) → **todas válidas**.
- Métricas: `post` = total − suma de las 3 vueltas (del STOP del gemelo). Segmentos de `fw_debug.log` en las 30 corridas con U: `U` (inicio U → fin U), `scan` (fin U → fase 40; p1 = hasta poste 1, entre = poste 1 → poste 2, f1 = fase 1), `rev` (fase 40 → TERMINADO: reversa/plan + centrado).

## Resultados

| tanda | cambio | ok | media total (s) | mediana | post todas | post con U (n=30) | U | scan | rev | contactos | wall (s) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| g_w_park_base | main 2813d20 | 95/96 | 42.22 | 41.94 | 9.276 | 12.090 | 2.107 | 2.417 | 4.866 | 1 (n1/s36, vuelta 1) | 133.3 |
| g_w_park_v1 | despegue aprendido, todos los tramos, tope 140 | 92/96 | 41.81 | 41.69 | 8.869 | 11.726 | 2.107 | 2.417 | 4.516 | 3 (+2 cajón) | 131.4 |
| g_w_park_v2 | ídem, sólo tramos en reversa, tope 140 | 94/96 | 41.97 | 41.83 | 8.998 | 11.791 | 2.107 | 2.417 | 4.589 | 1 | 132.1 |
| g_w_park_v3 | v1 + rampa propia 600 PWM/s, FRAC 0.80 | 82/96 | 41.66 | 41.34 | 8.665 | 11.344 | 2.107 | 2.417 | 4.333 | 13 | 130.6 |
| **g_w_park_v4** | **sólo reversa, FRAC 0.85, tope 125** | **95/96** | **41.89** | **41.65** | **8.944** | **11.802** | 2.107 | 2.417 | **4.588** | 1 (el mismo n1/s36) | 134.5 |
| g_w_park_v4c | v4 como default en el código (commit), sin overrides | 95/96 | 41.89 | 41.65 | 8.944 | 11.802 | 2.107 | 2.417 | 4.588 | 1 | 136.9 |

- Base = idéntica corrida a corrida a g1_base (g1m_base) → determinista.
- Segmentos del scan en base (con U): p1 1.277, entre 0.726, f1 0.368 s; no cambian con estas variantes (sólo tocan la fase 40/43).
- **v4 vs base, pareado (95 corridas ok en ambas): −0.332 s media, −0.228 mediana, mejor −1.65, peor +0.39; 10 más lentas, 12 iguales.** Fallas: sólo n1/s36 (choque en la vuelta 1, ya estaba en main). v4c reproduce v4 exacto.

## Variante (nueva): despegue aprendido en la fase 40
Diagnóstico en base: **cada tramo del plan arranca parado a PWM 78** (PARK_PL_ARRANQUE_PWM) y el gemelo con motor real no despega hasta ~147 con volante a tope (~107 recto); el anti-atasco espera 150 ms y sube a 400 PWM/s → **~0.33 s muertos por tramo**, 3–4 tramos por estacionamiento (en todos los logs: "PARK atasco: +20" ~280 ms después de cada "PARK plan N").
Cambio: se guarda 0.85 × el PWM con el que despegó el tramo anterior (por volante a tope / recto; nunca baja; tope 125) y el siguiente tramo **en reversa** arranca ahí, sólo hasta que se mueve (no es piso de movimiento; el piso de stick-slip de main no se toca). El primer tramo sigue igual (no hay dato). Params nuevos `PARK_PL_DESPEGUE_FRAC/MAX/FWD` (PARK_AJ; FRAC 0 = como antes).

## Descartado
- Aplicarlo también a los tramos hacia adelante (v1): son cortos y pegados a la madera de enfrente; el tirón los alarga (mm 40 vs 28) → 2 choques con el cajón (n2/s31, n2/s4).
- Tope 140 (v2): la reversa INT arranca demasiado fuerte, el predictor de holgura corta antes y queda afuera >5 mm (n2/s34).
- Rampa propia desde el aprendido (v3): más rápido (−0.6 s) pero 13 contactos: el plan está calibrado con el arranque lento.
- No se repitió: t6 pausa/scan (PARK_PL_PAUSA_MS 220 ms sin tocar), u5 arcos, t15 perfil de U, PARK_REV_RECTA.

## Commit
- `grok/t/g_w_park` = 1 commit sobre 2813d20 (90a4a9d, Jessebnda): PurePursuit.ino + PurePursuitCarrera.ino (+21/−1 cada uno; Carrera parchada a mano igual que el .ino, sin regenerar con exportar_carrera.py porque regenerar mete diferencias ajenas de main).

## Pendiente
- Quedan ~0.2 s/estacionamiento en el primer tramo (reversa EXT, sin dato aprendido): sembrarlo con el despegue de la salida del cajón o con un valor medido en el carro.
- Validar en el carro real: el comentario de main dice que a 11.5 V el carro se mueve con 80–99 PWM a tope; si el real despega antes, el aprendido baja solo (FRAC × lo que costó), pero hay que mirar el tirón del tramo 2 (reversa INT).
- n1/s36 (choque en la vuelta 1) sigue fallando en main y aquí; no es de estacionamiento.
- Siguientes candidatos: el tramo p1 del scan (fin U → poste 1, 1.28 s) y la U (2.1 s; g1 vio U −0.37 s con PARK_UTURN_ALEJAR_CM 38 en cribado de 14).

FIN
