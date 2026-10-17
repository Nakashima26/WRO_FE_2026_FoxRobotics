# HANDOFF — U de 180° y estacionamiento paralelo (2026-10-16)

Rama `claude/park-uturn-ef38b9`. Todo medido en el twin con `TWIN_MOTOR=fisico`
(N20 1.5:1: k 3.03, pwm0 19, τ 0.057 s). El twin **no** está calibrado contra el carro.

## Cómo se mide

- **Carrera completa en modo ruta**: preset `ruta_perfil`, `FOX_PARK_POS=random`,
  42 semillas (1-41 y 3170839), ruido 1 (más 2 y 3 para validar). Scripts en
  `src/RASPI/cam/runs/` (están en .gitignore): `pk.sh <tag> "<ruidos>" '<json>' 6` y
  `pkan.py runs/<tag>/n1`.
- **Corrida OK** = `race_finished`, sin choque, y huella ≤ 5 mm fuera de la caja del cajón.
- **Escenario corto**: `twin/tools/prepark.py` (hw_nuevo, sin modo ruta). Son 100
  escenarios en unos 4 min. No incluye la U. Lo usé para afinar la maniobra de reversa.
  Wrapper: `runs/ppsw.sh <tag> <PARK_AJ=V,..>`.

## Diagnóstico inicial

1. **En modo ruta la U nunca armaba**: la Pi manda `prio=1` siempre y `vigilarUturn` lo
   trataba como una esquiva. Esto **no es un bug**: el modo ruta se diseñó solo para las
   3 vueltas, sin estacionar. Estacionar en ruta es una función nueva y queda apagada por
   defecto (ver flags).
2. **El twin cortaba la carrera** cuando el carro se detenía con tc ≥ 12, incluso en los
   settles del estacionamiento. Se arregló en `sim.py` (no corta con `est == "E"`).
3. **Los UART saturados frenan el loop al estacionar.** Cada V2 (40 Hz) recibe un ACK de
   316 B, que son 27 ms a 115200 y más del 100 % de Serial2. Además, el log periódico
   (~170 B por loop) satura Serial USB. En ESTACIONANDO cada loop tardaba 30-60 ms, así
   que el escaneo del cajón no armaba (`arm=0`) y las fases cortaban tarde. En el carro
   pasa igual porque usa el mismo baud. Con esto, `prepark.py` daba **0/100**.
4. **Patrón A**: la U (radio mínimo ~13 cm, desplazamiento 2R) arrancaba a 32-37 cm de la
   pared, terminaba a ~6 cm y el escaneo chocaba con la barrera (fuera ~200 mm).
   En ruta fueron 10 de 41.
5. **Patrón B**: la contravuelta (fase 8) terminaba 22-36° torcida y con
   ATRAS PEGADO (distB ≈ 6), y luego chocaba con el cajón. Con el motor físico la
   maniobra de reversa ya no cuadraba (swing de 54-57°).
6. **Patrón C**: en ruta la U disparaba con el carro a ±30-40° (todavía siguiendo la ruta).
   El ToF lateral leía 200 (fuera de rango), así que el radio y la referencia de 180°
   salían mal.

## Aceptado (en el código)

| Cambio | Default | Efecto medido |
|---|---|---|
| `ACK_E_MIN_MS` (ms mínimos entre ACK en ESTACIONANDO*) | **100** | junto con LOG: prepark 0 → 27/100; sin los dos, 0/100 aunque la geometría esté bien |
| `LOG_MIN_MS` (log periódico, solo en ESTACIONANDO*) | **100** | la carrera no cambia porque solo aplica al estacionar |
| `PARK_ANG_IN_DEG` 63 → **70** | 70 | prepark 27 → 96/100 (con 67 da 80, con 73 da 97, con 76 da 76) |
| `PARK_POSTE2_MM` 40 → **60** | 60 | con ANG 70: 100/100 (con 80 da 0/100: es un precipicio, no subirlo) |
| `PARK_UTURN_RUTA_ALONG_MM` (U por pose del tapete en ruta; también habilita estacionar en ruta) | 0 = apagado | con 600 la U dispara a ~0.6 m del centro de la recta |
| `PARK_UTURN_ALEJAR_CM` (fases 26/27: alejarse de la pared del giro a 35° y enderezar antes de la U) | 0 = apagado | ruta 28 → 35/41 |
| En ruta: referencia de la U = rumbo del carril (`rutaErrCarrilDeg`) y lateral del tapete (`rutaExtTapeteCm`) | con ALEJAR | ruta 35 → **40/41** |
| `ACK_SALTA_VIEJOS` (no responde un V2 viejo si hay otro en cola) | 0 | quedó superado por ACK_E_MIN_MS |

Defaults con la configuración recomendada en ruta:
`PARK_MODO=PARK_PARALELO, PARK_UTURN_RUTA_ALONG_MM=600, PARK_UTURN_ALEJAR_CM=60`.

| Config (ruta, 42 semillas) | OK | Choque | Tiempo tc12 → fin (OK) |
|---|---|---|---|
| solo arreglo prio (motor viejo) | 0 | 19 | – |
| + along 600 + ACK_SALTA 1 (motor viejo) | 13 | 28 | ~14 s |
| + ACK/LOG 100, ANG 70, POSTE2 60 (físico) | 28 | 14 | ~14 s |
| + ALEJAR 60 + carril del tapete (físico) | **40** | 2 | ~16 s |

En las 42 hay una semilla (s32) que choca con una señal antes de tc12, ajena al
estacionamiento.

**Validación (ALEJAR 60, ruido 2 y 3):** n2 39/40, n3 38/40, sin contar las semillas que
chocan antes de tc12 (s16, s32). Fallas restantes: s18, donde la U pega con la señal E/T4
en ambos ruidos (la U pasa por encima de una señal de la recta), y s17 n3 (U a 37 cm, patrón A residual).
Tiempo de tc12 al fin: ~15.5-16 s.

## Rechazado (probado y peor)

- `PARK_POSTE2_MM` 80 (ruta, motor viejo): 13 → 1. `PARK_RECTO_MAX_MM` 50: 13 → 7;
  180: 0. `PARK_PARED_CM` 20: 3.
- En prepark (físico): POSTE2 100: 0/100. RECTO_MAX 60 o 140: 0/100. ANG_IN 55: 0/100.
  ANG 70 + POSTE2 80: 0/100.
- `PARK_UTURN_RUTA_ANG_MAX` = 15 (esperar a ir recto para disparar la U): 28 → 20/41,
  porque la U salía tarde o nunca (6 llegaron a max_time). Queda en 0.
- Una U hacia el lado interior: la fase 0 ignora lecturas > `PARK_PARED_MAX_CM` = 55.

## Pedido: arrancar a mitad de carrera (solo la U y la última recta)

- `prepark.py` ya hace esto en modo **legacy**: arranca en la salida de la esquina 12
  con tc = 12. Lo usé para la maniobra de reversa. Estaba roto (0/100) por la saturación
  de UART, no por el harness. Su arranque no pasa por la U de ruta.
- Para el **modo ruta** no hay arranque a mitad de carrera. Haría falta sembrar el estado
  de la Pi (RouteMode localizado, `rtx/rty/rth`, conteo de vueltas) y el del ESP (tc,
  `rutaSecSalida`, odometría). Es un cambio grande en `sim.py`/`pi_link`. No lo hice:
  la carrera completa tarda ~25 s de reloj por semilla, unos 10 min por banco de 42
  con 6 procesos.
- **Hecho (rama `claude/park-u-preset`)**: arranque sembrado en modo ruta. Cualquier carrera
  completa en modo ruta con `log_dir` graba `ruta_seed.json` al salir del giro 12, con el carro
  ya recto (|rumbo − carril| ≤ 8°). Guarda la pose verdadera en el instante de la odometría del
  ACK, px/py/yaw/ang/od del ESP, el od de tc=12, la pose del localizador y el shift/park_w del
  arranque ciego. Con `FOX_RUTA_SEED=<dir con s<seed>/>` (por ejemplo `runs/full/n1`), `Sim`
  arranca ahí:
  - ESP: `silRutaSeed` en el 1er loop, antes del 1er ACK (tc=12, U pendiente, odometría).
  - Pi: `_seed_pi` (mapa con las señales del campo, pose y yaw_fix, RouteMode con el avance
    resuelto). La ruta se sube de nuevo al ESP (~0.7 s).

  Uso: `FOX_RUTA_SEED=runs/full/n1 runs/pkw.sh seed/n1 1 "$EX" <seed>`, con el mismo preset y
  el mismo `FOX_PARK_POS`.

  **Validación** (ruta_perfil + PARALELO, ALONG 600, ALEJAR 60, físico, ruido 1; semillas 1-3,
  5-7, 17 y 18): 8/8 iguales. Siete dan OK; s18 choca en ambos casos con la señal Red E/T4, 1.6
  contra 1.7 s después de la U. Tiempo de la U al TERMINADO: 11.4 contra 11.4 s (s1).

  **Velocidad**: 7 semillas con 6 procesos tardan 48 s en carrera completa y 9.8 s sembradas
  (**~5×**). Una semilla sola: 26.7 → 7.8 s.

  **Límite**: el arranque viene de una carrera completa de la base. Sirve mientras el cambio no
  toque las vueltas 1-3. Si las toca, hay que volver a grabar las semillas con carreras completas.
  El carro arranca detenido (en la carrera llega a ~370 mm/s), así que la U sale ~1 s más tarde
  en reloj simulado.

  **Grabación más estricta**: además de |rumbo − carril| ≤ 8°, la semilla solo se graba con el
  rumbo estable (≤ 2° entre la odometría del ACK y el frame) y el localizador a ≤ 50 mm / 5° de
  la verdad. Sin eso, 4 de 41 semillas (s9, s21, s35, s38) arrancaban 60-80 mm / 10° corridas y
  chocaban donde la carrera completa no. Con el filtro quedan 28 de 42 semillas (s18 no entra).

## Park más rápido (rama `claude/park-u-preset`)

Constantes nuevas `PARK_AJ` (0 = como antes):
- `PARK_UTURN_ALEJAR_PWM` = 160 (fases 26/27; antes PARK_RETORNO_PWM_MIN 80).
- `PARK_UTURN_PWM` = 160 (tope de la rampa de la U, fase 24).
- `PARK_SCAN_RAPIDO_PWM` = 200 y `PARK_SCAN_RAPIDO_MM` = 400: escaneo del paralelo (fase 0,
  sub 0) rápido mientras `alongSalidaMm() > pka + 400`. `pka` = along de la madera del cajón más
  adelante en la carrera, que la Pi manda en el sufijo de ruta (`_stall_far_along`). En
  ESTACIONANDO el ESP usa la última transformación y el último pka.
- `PARK_UTURN_ENDEREZA` (1 = siempre fase 27, 0 = nunca, 2 = solo si ya avanzó ≥ 30 mm): no
  arregla s18 y quedó en 1.

Banco sembrado (28 semillas, físico, ruido 1), de la U al TERMINADO: base 13.25 s → 10.53 s,
28/28 OK. Por encima de 160 la U y el alejar no ganan nada (200/240 dan lo mismo). Con
`PARK_SCAN_RAPIDO_MM` 250, s7 pasa los postes a 410 mm/s sin verlos y sale del tapete: 400 es el
margen. `PARK_UTURN_RUTA_ALONG_MM` 450 agrega 2 choques (s2, s27) con señales de la recta.

**Carreras completas (42 semillas)**: 40/42, los mismos 2 fallos que la base (s18 en la U y s32
antes del park). De la U al fin: 14.05 → 11.00 s; carrera 78.0 → 75.0 s.

**s18** (CW, cajón N, Red N/T1 por dentro): la U se dispara en along 616 con el carro todavía a
−43° (sale de la curva de la señal) y la U choca con Red E/T4. Ni ENDEREZA 0/2 ni ALONG 450
lo arreglan.

## Baud 921600 con el park (rama `claude/park-baud-921600`, merge de fox/perfil-isla sobre park-u-preset)

921600 en el twin: `TWIN_FW_PARAMS="serial2_us_per_char=10.85,serial_us_per_char=10.85"
TWIN_PI_UART_US_PER_BYTE=10.85`. En el carro: `PI_BAUD`/`USB_BAUD` del .ino y `BAUDRATE` de config.py.

Carreras completas (42 semillas, ruido 1, ruta_perfil + PARALELO/ALONG600/ALEJAR60, físico, park random):

| config | OK | de la U al fin | carrera (media OK) | fallos |
|---|---|---|---|---|
| (a) 115200, ACK_E_MIN_MS/LOG_MIN_MS 100 | 40/42 | 11.00 s | 74.97 s | s18 (U → Red E/T4), s32 (señal W/T2 a 8 s) |
| (b) 921600, flags 100 | 40/42 | 10.61 s | 73.28 s | s18 (igual), s31 (señal W/T2 a 8 s) |
| (c) 921600, flags 0 | 40/42 | 10.67 s | 73.34 s | iguales a (b) |

prepark.py (100 escenarios, hw_nuevo, físico):

| | flags 100 | flags 0 |
|---|---|---|
| 115200 | 97/100 | **0/100** (el escaneo nunca arma) |
| 921600 | 100/100 | 100/100 |

**A 921600, `ACK_E_MIN_MS` y `LOG_MIN_MS` ya no hacen falta**: (b) y (c) dan lo mismo (0.06 s de
diferencia, ruido). A 115200 siguen siendo imprescindibles (0/100 sin ellos). Se dejan en 100: no
cuestan nada a 921600 y protegen si el carro queda a 115200. Los fallos s31/s32 cambian con el baud,
pero son choques con señales a los 8 s (primera vuelta), no del park.

## Pedido: ¿planear el paralelo con la pose del modo ruta?

**Es viable, pero no con la pose del tapete sola.** El cajón deja ~70 mm de holgura lateral (200 − 130)
y ~90 mm longitudinal (270 − 180). El plan necesita un error de pose ≲ 30 mm. La pose
de la Pi en ruta deriva (en hw_nuevo se midieron 150-430 mm sin latas). Con eso, un
Reeds-Shepp hacia el cajón **desde el tapete** fallaría.

Lo que sí funciona es un plan **local**:
- Medir los postes del cajón con el ToF lateral (ya lo hace la fase 0, con ~10 mm).
- Calcular dos arcos y una recta desde (d lateral medido, rumbo, posición del poste 2).
- Seguirlos con odometría, que sobre ~400 mm da ~10 mm de error.

Hoy la maniobra fija es sensible justo a lo que el plan local corregiría:
- El ángulo de entrada: 63 → 70 cambia 27 → 96/100.
- El avance tras el poste: 60 → 80 cambia 100 → 0/100.
- La distancia lateral de llegada (d = 26-30 cm contra los 14 que asume la maniobra).

Hoy el twin da 97-100/100 en prepark y 40/41 en ruta. La ganancia en tasa sería
pequeña. La ganancia real es la **robustez** ante el motor real y el ToF real (el
precipicio de POSTE2 y ANG muestra lo frágil que es la maniobra fija). La pose del tapete sirve
para el disparo de la U, el rumbo de carril y el lateral (ya se usa).
Recomendación: si el carro real falla por ángulo o avance, reemplazar fases 1-4/14
por el plan local de dos arcos. No antes.

## Riesgos / no verificado

- Nada se probó en el carro. Los dos flags de UART son reales en el hardware (mismo baud),
  pero el efecto en el carro no está medido.
- ANG_IN 70 / POSTE2 60 se ajustaron con el motor físico del twin. La meseta es estrecha
  (ANG 70-73; POSTE2 80 rompe).
- Tiempo: con ALEJAR el estacionamiento tarda ~16 s desde tc12, contra ~14 s.
  Aún no se ajustó el tiempo.
