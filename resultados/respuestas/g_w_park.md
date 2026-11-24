# g_w_park — respuesta a cursor-coord (2026-11-23 18:06 PST)

**Hecho:** base g_w_park_base en origin/main 2813d20 (U+park, cen_seeds 96, TWIN_MOTOR_REAL=1, VBAT 11.5) y 4 variantes de un mismo algoritmo nuevo en la fase 40 ("despegue aprendido"), todas bajo `flock twin.lock`, una a la vez, válidas (wall 131–137 s, ajenos 0, sin OOM).

**Base:** 95/96, media 42.22 s, post (total − 3 vueltas) 9.28 s; con U (30 corridas): post 12.09, U 2.11, scan 2.42, rev 4.87. Idéntica a g1_base. Única falla n1/s36 (vuelta 1).

**Hallazgo:** cada tramo del plan arranca a PWM 78 y el carro (gemelo, motor real) no despega hasta ~147 a tope → ~0.33 s muertos por tramo.
**Ganadora v4** (los tramos en reversa arrancan en 0.85 × el PWM de despegue del tramo anterior, tope 125): **95/96 igual que main, media 41.89 (−0.33 s pareado), post 8.94, rev 4.59, sin contactos nuevos**; repetida como default (v4c) idéntica. Commit en `grok/t/g_w_park` (90a4a9d).
Descartado: también adelante (2 choques con el cajón), tope 140 (n2/s34 afuera), rampa propia (13 contactos).

**Entrega:** el box no tiene credenciales de GitHub (sin gh auth ni tool de escritura) y el Mac de Jesse está desconectado → quedan en cola en el box: `pend/g_w_park_put.json`, `pend/g_w_park_resp_put.json` y la rama en `pend/g_w_park_branch/` (bundle + patch + README). Salen cuando haya push.

**Preguntas:** ¿el carro real despega a tope con 80–99 PWM (comentario de main) o más? Si es menos, el aprendido baja solo, pero conviene un dato real del tramo 2. ¿Quieren que siembre el primer tramo (~0.2 s más)?
**Siguiente que me pueden mandar:** sembrar el primer tramo; scan p1 (fin U → poste 1, 1.28 s); confirmar U con PARK_UTURN_ALEJAR_CM 38 en 96 (g1 lo vio en cribado).

## 2026-11-23 18:30 PST — reenvío del mismo encargo
Recibí otra vez la misma tarea (Worker PARK, base 2813d201). Ya estaba terminada a las 18:06 PST (ver arriba y `resultados/g_w_park.md`), así que **no volví a correr las tandas** (mismo código, mismo banco → resultados deterministas e idénticos; g_w_park_base = g1_base corrida a corrida).
**Novedad:** la Mac volvió a estar en línea, así que ahora se publicó todo lo que estaba en cola: `resultados/g_w_park.md` (FIN), esta respuesta y la rama `grok/t/g_w_park` (1 commit sobre 2813d201, despegue aprendido v4, Jessebnda).
Si querían algo distinto a lo ya hecho, mándenlo con otro id/tag. Siguientes candidatos: sembrar el primer tramo de la reversa EXT (~0.2 s), scan p1 (fin U → poste 1, 1.28 s), U con PARK_UTURN_ALEJAR_CM 38 en 96 corridas.
