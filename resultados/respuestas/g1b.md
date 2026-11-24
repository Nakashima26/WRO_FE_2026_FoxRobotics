# Respuesta g1b (2026-11-23, PST)

## Qué se hizo
- Worktrees nuevos `g1b_wt` (2813d20 limpio) y `g1b_rm_wt` (2813d20 + MOTOR_REV_RAMPA_MIN 55->70), con el mismo banco de g1 (cen_seeds 96, U+park, TWIN_MOTOR_REAL=1, VBAT 11.5). Las tandas corrieron una a la vez bajo flock.
- Volví a correr la base: 95/96, media 42.22, falla n1/s36, idéntica a g1m_base. El "93/96 media 42.20" de `g1_base_resumen.txt` venía de a943835 con parches, no de main.
- RMIN70 solo en 96: **94/96, media 42.05** (falla nueva n2/s34, 5.0 mm fuera contra 4.9 mm en la base). Pareado −0.20 s.
- Réplica n3..n8 de s11/s13/s34/s36/s39/s40: base 35/36 (42.13), RMIN70 35/36 (41.99). Las dos fallan solo n5/s36 (señal en la vuelta 1).
- **Rechazado por el criterio** (más fallas que la base): sin commit, no se creó grok/t/g1b y main no se tocó.

## Hallazgos
- n2/s34 es parking **sin U**, al límite en main (4.9 mm de 5). La pausa corta o RMIN70 mueven la pose final 2–5 mm o 1–2° y la sacan del umbral. No hay contacto: el carro queda poco metido, del lado de la pista.
- n2/s39 (con U) solo falla en la combinación c2 (PAUSA 100 + ARC1 150 + RMIN70): el plan 3 se corta por FRENTE y el plan 4 (REVERSA INT) choca con el cajón magenta. Con PAUSA 150 sola o con RMIN70 solo pasa.
- Mecanismo: con pausa < ~220 ms el tramo arranca antes de que las ruedas lleguen al ángulo (servo de 600 °/s supuesto + τ 0.10 s medido), y el arco sale más abierto.
- No encontré `probado.md` en ninguna rama (main, grok/resultados ni el resto). Usé g1.md y pend/g1_t.md como registro de lo probado.

## Preguntas de vuelta
1. ¿Se acepta un criterio con tolerancia para fallas al umbral (por ejemplo fuera ≤ 5.0 mm ±0.1, o confirmar con réplica n3–n8)? Con eso RMIN70 pasaría (−0.2 s, sin fallas en la réplica).
2. ¿Hay medición real del slew del servo? La sensibilidad a PAUSA del gemelo depende de 600 °/s, que es un supuesto.
3. ¿Dónde vive probado.md? ¿Lo creo en grok/resultados?

## Qué mandar después
- Una tarea para el margen lateral del final del plan (s34 sin U): corrección de w o un objetivo de profundidad con margen en el tramo 3. Luego volver a probar RMIN70 encima.
- Una pausa dependiente del giro (esperar a que el servo esté en posición, o pausa proporcional a |Δservo|) en lugar de bajar PAUSA_MS fija.
