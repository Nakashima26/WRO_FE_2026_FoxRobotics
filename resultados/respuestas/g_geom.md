# g_geom — respuesta a cursor-coord (2026-11-23 19:15 PST)

**Hecho:** cribado de las U con la señal de entrada, algoritmo nuevo Pi+FW opt-in (pared exterior → reversa con ancla ToF en la 1ª madera → plan normal, sin U) y 5 bancos U+park en el box (cen_seeds 96, TWIN_MOTOR_REAL=1, VBAT 11.5), cada uno solo bajo `flock twin.lock`. Base = g_w_park_base (2813d20, misma config).

**Hallazgo clave:** "verde = paso por fuera" sólo es cierto en **CW**; en **CCW la verde se pasa por dentro**. De las 30 U de base, 11 tienen entrada verde y 9 de ellas son CCW por dentro. Aun así la idea funciona: en esas la ruta de la Pi ya deja al carro en la línea de escaneo (ext 27–34 cm) pasado el cajón antes de que dispare la U; ahí el ESP frena, retrocede pegado a la pared hasta que el ToF lateral ve la cara de atrás de la 1ª madera y hace el plan normal.

**Resultado (96):** verde 95/96 = base (sólo n1/s36, vuelta 1), U 30 → 21, media 42.22 → 42.08 s, post 9.28 → 9.14 s (en las 30 que hacían U: 12.09 → 11.65), sin contactos nuevos; las 9 sin U −0.4 a −2.3 s. **Réplica n3–n8** (48): 47/48 = base, U 47 → 18, −0.90 s pareado, ninguna más lenta.
**Aceptada** opt-in: `grok/t/g_geom` 6962d96 (Jessebnda). Activar: Pi `ROUTE_PARK_GEOM_EXT=1` + ESP `PARK_GEOM_EXT=1` (flashear). FW-only con cualquier color (`PARK_GEOM_EXT=2`, sin Pi) da U 30 → 15 y −0.17 s, pero s39 (roja) sale +0.5 s; su réplica quedó **inválida** por carga ajena en el box (indicativa: −1.0 s, s39 +0.2/+0.7).

**Preguntas:** ¿la regla que querían era "verde" literal o "paso por fuera" (verde en CW / roja en CCW)? En el gemelo las U por fuera casi no existen (3/30) y ahí la ruta se va de la pared por la T1. ¿Frenar con contramarcha a ~0.7 m/s y reversa a ~0.5 m/s pegado a la pared es aceptable en el carro real, o prefieren coast?
**Siguiente que me pueden mandar:** réplica limpia de `PARK_GEOM_EXT=2` (cualquier color) y corregir la altura antes de la reversa en s39; reversa más rápida para las CW con cajón al inicio (la ruta llega a la pared en pka+600, hoy siguen con U); medir en el carro `PARK_GEOM_W_REV` (181 − 0.29·(h−300), ajustado en el gemelo).
