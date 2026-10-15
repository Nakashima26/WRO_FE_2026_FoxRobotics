# Handoff: sesión de Grok bot (mejora de tiempos)

Base de partida: `dfe95f7` (Modo ruta + arranque ciego con el cajón movido).
Resultado: **ningún cambio fue adoptado**. Grok no dejó commits; su trabajo quedó sin commitear en worktrees.
Las cifras de este documento las reportó Grok y **no están reproducidas aquí** salvo que se indique.

## 1. Dónde quedó el trabajo

Respaldado, sin tocar los worktrees, en ramas locales `backup/grok-*` (un commit cada una sobre el HEAD de su worktree):

| Rama de respaldo | Worktree | Contenido |
|---|---|---|
| `backup/grok-algo` | `FoxRobotics-wt-algo` | Localizador de la Pi: compuerta de saltos de sonar (V1b), reaceptar tras rechazos 3σ, compuerta contra medida esperada (V1c), filtro espejo del ESP en la medida esperada (V4a/V4b), `twin/esp_diag.py` de diagnóstico (solo lectura), `ACK_SONAR_RAW` en el .ino. Todo detrás de banderas, False = comportamiento anterior. |
| `backup/grok-opt` | `FoxRobotics-wt-opt` | Solo banco de pruebas del twin: `FOX_SERVO_SLEW`, `FOX_SONAR_SIGMA`, `FOX_SONAR_REL`, `FOX_SONAR_DROP`, `FOX_SONAR_OUTL` por variable de entorno. Sin efecto por defecto. |
| `backup/grok-vel` | `FoxRobotics-wt-vel` | `ROUTE_UPLOAD_SLOW` (config + route_mode) y banderas del giro PI en el .ino (`GIRO_PI_SIN_FRENO`, `GIRO_PI_ARMADO`), todas apagadas por defecto. |
| `backup/grok-optvel` | `FoxRobotics-wt-optvel` | Igual que `vel` más lo de `opt` en una sola base. |
| `backup/grok-arco` | `FoxRobotics-wt-arco` | Reescritura grande y vieja (base `eff6d4a`, 1 oct): centerline, controller, obstacle_memory, wro_field, map_view, chassis_twin. Probablemente experimento aparte; no mezclar sin revisar. |
| (sin cambios) | `FoxRobotics-wt-robo` | Limpio. |

## 2. Diagnóstico de tiempo (reportado por Grok)

El carro va unos 28 s por corrida por debajo del tope de 155. Mayores frenos:

1. Sin ver todavía la señal: ~11 s (tope 100).
2. Tope de 120 en la vuelta 1: ~5.2 s, casi todo en las esquinas (~3.3 s).
3. No saber bien dónde está: ~4.3 s.
4. Subidas de ruta: ~3.4 s; 174 de 564 se tiran porque llega otra más nueva (duran un frame).
5. Salida por dentro (19 de 42 seeds): 4.5 s en vez de 2.7 s (gira 90°, 620 mm de lado vs 200 mm).

Recortar la ruta en sí solo da 3 a 5 %.

## 3. Candidato principal: `ROUTE_LAP1_PWM` 120 → 155 (con `ROUTE_SLOW_PWM` = 100)

Es un cambio de una línea en `src/RASPI/cam/pure_pursuit/config.py` (hoy `ROUTE_LAP1_PWM = 120`, `ROUTE_SLOW_PWM = 100`).

Reportado por Grok, 42 seeds del twin:
- Sin ruido: mediana 65.0 s → 63.7 s (-1.2 s), 41/42 limpias igual que la base.
- Con ruido a 375 °/s: 39/42 en ambas, mediana 64.70 → 63.94 s (-1.03 s).
- Con ruido a 300 °/s: 41/42 en ambas, 64.81 → 63.99 s (-0.92 s).
- No agrega fallas a la lista de seeds que ya fallaban con la base.

Críticas abiertas del propio equipo:
- Con una sola corrida de ruido por seed, "falla las mismas seeds" es evidencia débil. Pidieron 5 corridas de ruido por seed, base y candidato (42 × 5 × 2 = 420 corridas), comparando **tasas** de fallas.
- Para que el ruido sea pareado debe salir de semilla + sensor + tick, no de un solo flujo. Sin esto no se puede atribuir una falla a un par concreto.

Rechazado: subir también el tope de 100 (sin ver la señal) llega a 58.2 s (-7 s) pero choca en las seeds 16 y 33, porque el mismo 100 lo usan los frenos de seguridad. Idea pendiente: separar "todavía no he visto la señal" de los frenos de seguridad. Causa del seed 16: la ruta cambia de lado al confirmar rojo y el planificador acepta pasar a 49 mm; además el filtro descarta el ToF frontal por exceso de confianza en la pose (eso lo ataca `backup/grok-algo`).

## 4. Riesgos para la prueba en el carro real

- **TERMINANDO** (.ino, `TERMINANDO_MS = 1000`): 1000 ms a PWM 155 girando hacia la isla y ~109 mm de inercia. La base ya queda a 2.2 mm de la isla con servo a 375 °/s. La inercia depende de τ = 0.25 s, **sin calibrar**. Si en el carro real τ es mayor, es choque. No depende de 100/155. Propuesta: bajar PWM o duración de TERMINANDO.
- Pendiente de confirmar en el reglamento: si esa fase cuenta en el tiempo y si tocar la isla tras cruzar la meta se penaliza.
- Todo es resultado del twin; ninguna cifra se midió en el carro.
- Seeds que fallan con ruido: s8, s14, s21 (375 °/s), s41 (300 °/s), s16 y s32 (colas previas). El seed 32 choca con la señal verde en la base.

## 5. Siguiente

1. Reproducir 100/155 con 5 corridas de ruido por seed (ver `docs/` y `twin/`), tasa de fallas base vs candidato.
2. Hacer el ruido por (semilla, sensor, tick) en el twin.
3. Medir holgura mínima de TERMINANDO en 42 seeds y decidir cuánto bajarlo.
4. Si todo cuadra, adoptar `ROUTE_LAP1_PWM = 155` y probarlo en el carro con velocidad escalonada.
