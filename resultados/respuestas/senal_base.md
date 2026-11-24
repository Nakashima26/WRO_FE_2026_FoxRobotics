# Respuesta Grok — senal_base (choques con señal)

**Qué se hizo**
- Worktree nuevo `/workspace/fox/wt-senal`, detached en `origin/fox/twin-iter` @ 0844147.
- Tipo de prueba: **carrera completa de 3 vueltas con señales** en el banco con latencia (4.28/5.8, VBAT 12.5, primer plan 1.95, costo FIJO, lens_calib). 6 ruidos × 48 semillas = 288 carreras, `xargs -P 7`.
- Cada tanda corrió sola, bajo `flock /workspace/fox/twin.lock`. Tandas: senal_base, senal_fix (v1), senal_fix2 (v2) y una mini tanda de 14 carreras para comprobar que con el flag apagado no cambia nada. Las tres tandas completas son válidas: sin OOM, carga máx < 8 y reloj de 313–383 s.
- Cada choque con señal se volvió a correr con un envoltorio que solo agrega log (pose del Pi, CTE, rutas adoptadas). Los resultados coinciden con la tanda.

**Qué se encontró**
- Base: 269/288 ok, mediana 33.96 s, 7 choques con señal.
- Clasificación: **pose 6, seguimiento 1 (n3/s27, mixto), planificador 0**. El plan siempre tenía la señal con tiempo, con holgura de carrocería de 63–110 mm.
- Causa dominante: s33 en los ruidos 2–5. El FIX de salida usa el voto "recta" de la línea naranja, que con la cinta a 30° mide el cruce ~120 mm más lejos de lo real. Eso deja la pose ~140 mm corrida a lo largo, w0 = 1270–1294 cuando el real es 1128, y el auto roza E/T1 a los ~4 s.
- n5/s36 y n6/s23: deriva a lo largo en la recta, que se vuelve lateral tras la esquina en la vuelta 2 (a ~1.1 m/s).
- Arreglo opt-in `ROUTE_SIGN_FIX` (rama `grok/t/senal_fix`): tras el FIX, si la cámara ve un pilar que se asocia sin ambigüedad a un asiento conocido, se corrige la pose y w0. Con v2: **273/288, 0 fallas nuevas, +4 (todas las s33), mediana 33.91 s**; las 269 comunes salen con tiempo idéntico. v1 (umbrales más laxos) metía una falla nueva (n5/s35), por eso se ajustó.
- Detalle completo: `resultados/senal.md`. CSV: `resultados/senal_base_choques.csv`.

**Preguntas**
1. `fox/twin-iter` ya está en ad8e5dd con ROUTE_FIX_PILLAR_PAIR (mismo problema del FIX, otro enfoque) y mi rama choca con ese commit en route_mode.py. ¿Hago rebase de `ROUTE_SIGN_FIX` sobre ad8e5dd y mido si suma algo encima de PILLAR_PAIR, o lo descartamos a favor de PILLAR_PAIR?
2. ¿Atacamos la raíz, el voto "recta" de la cinta inclinada (|n·a| bajo)? Sería más limpio que corregir después.
3. ¿Interesa n5/s36 / n6/s23 (pose en vuelta 2 a alta velocidad)? Haría falta una corrección con pilares también en vueltas 2–3, y eso es más riesgoso (v1 mostró correcciones falsas).

**Qué mandar después**
- Si sigue: "rebase de senal_fix sobre ad8e5dd, tanda con ROUTE_SIGN_FIX=true vs ad8e5dd, mismo banco 288".
- O: "arreglar voto recta con cinta inclinada" con s33 como casos.
