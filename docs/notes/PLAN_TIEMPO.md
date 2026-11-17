# Plan de acción: bajar el tiempo de verdad

Fecha: 2026-10-14. Base: `fox/seed-3170839-red-object` (`89ea73f`).

## Dónde estamos (hechos, no suposiciones)

- **Twin, obstáculos, modo ruta:** ~65 s mediana (vuelta 1 ~27.7 s, vueltas 2-3 ~17.7 s c/u), con `MOTOR_MAX=155`.
- **El modo ruta nunca se ha probado en el carro real.** Todo lo anterior es twin.
- **Firmware real:** `MOTOR_MAX = rondaObstaculos ? 120 : 180` (`PurePursuit.ino:1694`). El twin corre a 155, más rápido que el carro hoy en obstáculos.
- **Modelo del motor en el twin:** lineal, `v = 3.5 mm/s × (PWM − pwm0)` (`twin/params.py:71`), sin saturación ni caída de batería. El Planner Robótica de Grok ya avisó que el motor real no llega a lo que supone el twin.
- **El modo ruta no tiene perfil de velocidad por curvatura:** va a tope constante y solo frena con topes (`route_mode.py`). En las vueltas 2-3, con el mapa ya conocido, no aprovecha las rectas.
- **Video real de hoy (abierto, sin señales):** ~18 s por vuelta, ~3 vueltas en 55 s, va por el centro del pasillo y abre las esquinas.
- **100/155 (vuelta 1):** reproducido en el twin, −0.95 s, sin evidencia de fallas nuevas (ver `GROK_HANDOFF.md` §6).

## Por qué el equipo de Grok se quedó en milisegundos

1. Optimizaba el twin, no el carro. Sin calibrar contra la realidad, cada décima ganada puede no existir.
2. Ocho agentes con crítico por área: cada cambio chico exigía discusión estadística, y nadie era dueño de las apuestas grandes.
3. Atacaba los frenos uno por uno en lugar de cambiar la estrategia de velocidad.

## Las apuestas grandes (ordenadas por segundos esperados)

| # | Apuesta | Ganancia estimada | Depende de |
|---|---|---|---|
| A | **Perfil de velocidad por curvatura en vueltas 2-3**: rápido en rectas, frenar antes de curvas, con el mapa congelado tras la vuelta 1 | 5-10 s (dos vueltas de ~17.7 s) | Saber la velocidad máxima real y la aceleración lateral sin derrapar |
| B | **Subir el tope real** en obstáculos (hoy 120 en el carro) hasta lo que aguante el chasis | 5-15 s | Medición real de velocidad y estabilidad |
| C | **Vuelta 1: separar el freno de "señal no vista" de los frenos de seguridad** (Grok: hasta −7 s, choques en seeds 16/33 por el freno compartido) | 3-7 s | Arreglar el localizador de la seed 16 (`backup/grok-algo`) |
| D | **Ver las señales antes** (alcance de cámara `ROUTE_VIEW_MAX_MM=550`): confirmar antes = frenar menos | 2-5 s | Calidad de detección a más distancia, en el carro |
| E | **Ronda abierta:** línea de carreras fija y velocidad máxima; ya va 180 PWM, pero el video muestra trazada central y esquinas abiertas | 2-4 s por vuelta | Probar en pista |
| F | Salida por dentro (19/42 seeds tardan 1.8 s extra) | ~1 s promedio | — |

Lo que **no** vamos a perseguir ahora: ajustes de servo/slew, filtros finos, cambios de décimas. Solo vuelven si una apuesta grande los necesita.

## Fase 0 (bloqueante, en el carro): calibrar la realidad — 1 sesión de pista

Sin esto, A-D son ficción. Objetivo: que el twin prediga el carro.

1. **Velocidad vs PWM:** recta de 2 m a PWM 100, 120, 155, 180, 220, 255, con batería llena y a media carga. Medir tiempo (cronómetro o encoder). Da la curva real del motor.
2. **Frenado e inercia (τ):** a cada PWM, cortar y medir cuánto avanza. Arregla el riesgo de TERMINANDO.
3. **Curva máxima sin derrapar:** círculos a radio fijo subiendo velocidad. Da la aceleración lateral máxima.
4. **Modo ruta en el carro**, primero sin señales y lento: subida de ruta, seguir una recta, una vuelta. Es la lista de `MODO_RUTA.md` "Lista para el carro real".
5. Grabar el log de la Pi y del ESP en cada prueba y un video desde arriba.

Con 1-3 se ajusta `twin/params.py` y se corre `tools/paridad.py` hasta que twin y carro coincidan en tiempo de vuelta dentro de ~5 %.

## Fase 1: apuestas A y B en el twin calibrado

- Implementar el perfil de velocidad por curvatura (`v ≤ sqrt(a_lat_max / κ)`, con rampas de aceleración y frenado medidas) solo en vueltas 2-3.
- Barrer el tope de recta con el banco estándar (abajo). Aceptar si cumple la regla.
- Probar en el carro el mejor candidato.

## Fase 2: C y D (vuelta 1)

- Separar los topes (señal no vista vs seguridad), traer el arreglo del localizador de `backup/grok-algo`, revisar.
- Medir a qué distancia la cámara confirma bien las señales en el carro, no en el twin.

## Fase 3: E y F

## Banco estándar (para no discutir cada cambio)

Un solo comando que corre base vs candidato y escribe una tabla, para que la decisión sea mecánica:
- 42 seeds × 5 ruidos, preset `ruta`, `FOX_PARK_POS=random`, `TWIN_NOISE_SEED` (ya existe en `fox/verify-lap1-155`).
- Reporta: tasa de fallas, tiempo pareado (mediana), pares discordantes, y una imagen por falla.
- **Regla de aceptación:** ahorra ≥ 0.5 s mediano **y** pares solo-candidato ≤ solo-base + 2 **y** ninguna seed que nunca fallaba falla en ≥ 2 ruidos. Si dudoso, 20 ruidos más en las seeds discordantes.
- Ningún cambio se da por bueno hasta probarlo en el carro.

## Equipo (más chico que el de Grok)

| Rol | Modelo | Qué hace |
|---|---|---|
| Orquestador (sesión principal) | grande | Dueño del presupuesto de tiempo, elige apuestas por segundos esperados, decide con el banco |
| `fox-ojo-creativo` | sonnet | Sin código: solo ve imágenes de la trayectoria sobre el mapa, velocidad por tramo y dónde choca. Propone ideas de estrategia |
| `fox-banco` | haiku | Corre el banco estándar y devuelve la tabla y las imágenes |
| `haiku-coder` | haiku | Implementa los cambios del plan |
| `sonnet-reviewer` | sonnet | Un solo crítico: revisa el diff contra el plan y la regla del banco |

El crítico ya no negocia estadística en cada caso: la regla está escrita arriba.

## Decisiones del usuario (2026-10-14)

- Prioridad: **ronda de obstáculos**. Objetivo: recortar las vueltas; el estacionamiento queda fuera por ahora (el tiempo se corta al estacionar).
- El tope de 120 en obstáculos era prudencia con el algoritmo viejo, no un límite del hardware.
- Velocidades: con batería promedio; la caída de batería se ignora.
- Las rectas son la prioridad: apuestas A y B primero.

## Motor: cuánta velocidad queda

- N20 50:1 + etapa LEGO 2:1 (100:1), rueda de 43 mm (135.1 mm por vuelta), 3S LiPo ~11.1 V (README §2.3, §3).
- Única medición real: ~245 mm/s a PWM 95 (`HANDOFF.md:142`). El twin está ajustado a ese punto: `v = 3.5·(PWM−25)` → 332 mm/s a 120, 455 a 155, 612 a 200, 805 a 255.
- Si fuera la versión de 12 V a 460 rpm (Robotshop), a 11.1 V daría solo ~480 mm/s sin carga a PWM 255, menos que lo que ya se midió a PWM 95 escalado. Así que el motor es más rápido que esa versión (probablemente uno de 6 V alimentado a 11.1 V). No se sabe la variante exacta.
- Conclusión: a PWM 255 el carro debería andar entre ~650 y ~800 mm/s, **el doble que hoy a 120**. Es el margen más grande que hay. Falta medirlo en el carro: no se puede confiar en estirar una línea desde un solo punto.
- Ganancia estimada si las rectas de las vueltas 2-3 van a ~700 mm/s en vez de ~450: ~2.5 s por vuelta neto de rampas, ~5 s en dos vueltas, más parte de la vuelta 1.
- Límite real probable: tracción al acelerar (el 50:1 patinaba en salidas de curva, README §2.3), no el motor. Por eso el perfil necesita rampas de aceleración medidas.

## Medición mínima en el carro (lo que hace falta antes de la Fase 1)

Ya existe la hoja y los scripts: `twin/tools/calib/HOJA_MEDICION.md`, `calib_capture.py`, `calib_fit_motor.py`. La hoja completa son ~2 h; lo mínimo para las rectas es la sección 2-3 (velocidad vs PWM) añadiendo PWM 155, 200 y 255, y la 4-5 (aceleración y frenado). Unos 30-40 min.

## Resultado: perfil por curvatura (apuesta A), twin

Rama `fox/perfil-rectas` (`95171c9`), apagado por defecto. Preset `ruta_perfil` vs `ruta`, 42 seeds × 5 ruidos:
**−1.84 s** mediana pareada, fallas 15 vs 14 (1 solo-perfil). v1 daba −2.6 s pero +12 choques (llegaba a la meta a 780 mm/s y frenaba tarde antes de señales); v2 no acelera en la última sección y llega a las señales a 155. Falta: calibrar con las mediciones del carro, combinar con 100/155, y revisar `TURNS_PER_RACE` (4 por defecto en el .ino) antes de usarlo en el carro.

## Referencia: Team ANTi (WRO 2025, abierta 16 s, obstáculos 35 s con puntaje completo)

Leído de su repo (`atakanersoy/WRO2025_FE_ANTi`), cifras autodeclaradas:
- **No usan nada sofisticado:** PID sobre cámara + rumbo, PWM abierto con rampa, giros de 90° por giroscopio, sin mapa, sin pure pursuit, sin control de velocidad por encoder, no cortan esquinas. (Lo que dijo Gemini sobre MPC/splines no coincide con su código.)
- **Su ventaja es física y de flujo:** carro de ~130 g y 69 × 53 mm (el nuestro ~1.5 kg, 180 × 130), N20 de 1500 rpm casi directo con rueda de 21 mm (1.4 m/s operativa), y **nunca se detienen**: reaccionan a cada señal al verla, la vuelta 1 va igual de rápido que las demás.
- Lo aplicable a nosotros: (1) la vuelta 1 es nuestro mayor hueco (~27.7 s vs ~17.7 s de las otras): reaccionar al ver en lugar de frenar por asientos sin ver (apuestas C y D); (2) masa: menos peso = frenadas y curvas más cortas en el carro real; (3) un carro más angosto tiene más pasillo para trazar alrededor de las señales.
