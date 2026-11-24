# g_gs_t1 — respuesta a cursor-coord (2026-11-23, 20:15 PST)

**Hecho:** diagnóstico de n1/s36 con trazas, algoritmo nuevo opt-in en la Pi (`ROUTE_FIX_SEAT_ROWS`, fijación del cajón) y bancos en el box con motor real, VBAT 11.5, cámara desde calib + lente, costo fijo 1.95; cada tanda sola bajo `flock twin.lock`. Tipo de prueba: carrera completa con arranque desde el cajón (la falla objetivo es de la vuelta 1).

**Causa de n1/s36:** no es la detección de la verde. Ya se veía a 1.0–1.2 m desde la salida del cajón, pero sin el avance del cajón fijado la ruta va sin señales. En esa corrida la cinta no vota y los ToF tardan 113 lecturas: el cajón se fija en la esquina (t=5.4 s) y el carro, que venía por fuera de la roja W/T2, ya no alcanza a cruzar hacia adentro de la Green S/T1.

**Algoritmo:** cada señal seguida desde la recta del cajón da hipótesis de avance. Las de la propia recta (fila interior) lo dan módulo 500 mm; las de la recta siguiente lo dan módulo 200 mm, porque su columna no depende del avance y su fila sí. Se queda el avance que explica todas con asiento legal dentro del rango del cajón, y los ToF desempatan. Sólo CCW.

**Resultado:** base 95/96 41.89 s → variante **96/96 41.42 s** (−0.49 s pareado, 0 contactos, CW idénticas). Réplica n3–n8 de las 23 semillas que cambian: 136/138 → **138/138**, −1.04 s pareado (n5/s36 también se arregla). Error del cajón CCW: máx 122 mm → 28 mm en las 38 corridas que ancla. **Aceptada**: `grok/t/g_gs_t1` bdbdc4e (Jessebnda). Activar: Pi `ROUTE_FIX_SEAT_ROWS: true`, sin flashear el ESP.

**Ojo:** el carro (`carrera.json`) ya tiene `ROUTE_FIX_UNIQUE_PREFIX_SEAT: true`, pero el banco (`carrera_bench.json`) no. Con sólo ese flag el banco ya da 96/96 41.49 s, así que la base del banco no es la config del carro. Mi flag lo contiene y suma la recta siguiente (38 vs 29 corridas ancladas temprano, −0.06 s vs ese flag solo). La versión con CW sumó 2 choques y se descartó.

**Preguntas:** ¿ponemos `ROUTE_FIX_UNIQUE_PREFIX_SEAT=true` en `carrera_bench.json` para que la base del banco sea la del carro? ¿Activamos `ROUTE_FIX_SEAT_ROWS` en `carrera.json`?
**Siguiente que me pueden mandar:** re-base del banco con ese flag y re-medir `ROUTE_FIX_SEAT_ROWS` encima. Ver las 14 corridas CCW que siguen fijando con la cinta (error hasta 122 mm, n1/n2 s21). Medir en el carro el ruido del pie BEV a 1.0–1.3 m antes de activarla.
