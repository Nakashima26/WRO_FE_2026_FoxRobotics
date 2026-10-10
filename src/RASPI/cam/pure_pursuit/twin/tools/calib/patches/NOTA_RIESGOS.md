# Parches propuestos para PurePursuit.ino (NO aplicados)

Base: `fox/v2-medidas` (67bcacc). Ambos archivos son diffs `git apply` con finales de linea CRLF (como el .ino):

1. `looptime_instrument.patch` — instrumentacion + quitar el 2o `mpu.update()`.
2. `us_async.patch` — pings HC-SR04 por interrupcion. Se aplica DESPUES del 1.

```
git apply --check tools/calib/patches/looptime_instrument.patch     # desde la raiz del repo
git apply tools/calib/patches/looptime_instrument.patch
git apply tools/calib/patches/us_async.patch                        # opcional
# si core.autocrlf convirtio el .ino a LF:  git apply --ignore-whitespace ...
```
Verificado: `git apply --check` limpio sobre fox/v2-medidas y sobre fox/tof-rate (720ea35; no se toco
initToF: los hunks estan en ~l.2568, 2790, 3146, 3196, 3287, 3290), y el .ino parcheado compila en
el SIL del twin (zig c++) con las combinaciones FOX_LOOPSTAT 0/1, FOX_LOOPSTAT_ACK 1, FOX_MPU_ONCE 0/1,
FOX_LOOP_PERIOD_US 20000. **No compilado para ESP32** (sin arduino-cli aqui); el bloque asincrono
solo se reviso con `-fsyntax-only` contra un mock de Arduino.

## 1. looptime_instrument.patch
Que hace: `loopStat()` al inicio de `loop()` mide el periodo entre entradas consecutivas (todo
incluido) y cada 1 s imprime por USB `[LOOP] n= min= mean= max= >10ms= >20ms= >40ms=` (us). El 2o
`mpu.update()` (l.3287) pasa a `#if !FOX_MPU_ONCE`; `actualizarGyro()` se conserva. Macros (todas
con default seguro): `FOX_LOOPSTAT` (1), `FOX_LOOPSTAT_ACK` (0), `FOX_LOOP_PERIOD_US` (0),
`FOX_MPU_ONCE` (1). En el SIL del twin el ahorro es 500 us/loop (mpu_update_us=500 modelado); en el
carro seran ~1.5-2 ms a 100 kHz (por verificar con el LOOPTIME de CalibFox).

Riesgos:
- Se imprime por `Serial` (USB), no por Serial2: no cambia el largo del ACK ni los tiempos de UART con la
  Pi. `FOX_LOOPSTAT_ACK=1` agrega `,lpm=,lpx=` al ACK:V2; el parser de la Pi no se reviso para campos
  nuevos y el comentario del .ino (l.2786) avisa que el largo del ACK mueve los tiempos: dejarlo en 0.
- El `Serial.print` de la linea comienza con `\n` para no pegarse a la linea de debug en curso; los
  parsers de logs que asuman una linea por loop veran 1 linea extra por segundo.
- Quitar el 2o `mpu.update()`: la 2a `actualizarGyro()` reutiliza el mismo gz (muestra de hace unos ms).
  El yaw integrado cambia en segundo orden y el EMA de `gyroRate` (0.7/0.3 por llamada) se aplica
  igual de veces que antes, pero sobre una muestra repetida: la derivada de GIRO_RAPIDO (`GR_KD*gyroRate`)
  y el gate REREF_RATE_MAX pueden reaccionar algo mas lento. Probar: 4 esquinas con
  `FOX_MPU_ONCE=0` y `=1`, comparar `yaw=` final y numero de giros; revertir con `-DFOX_MPU_ONCE=0`.
- Hipotesis NO medida: los `Serial.print` de debug por loop (115200 baud, ~5 ms por 60 caracteres cuando
  el buffer TX se llena) pueden dominar el periodo tanto como los pulseIn. El `[LOOP] max` y `LOOPTIME`
  de CalibFox permiten separar ambos efectos.

## 2. us_async.patch  (`-DFOX_US_ASYNC=1`; default 0 = identico a hoy)
Que hace: `usService(mask)` dispara UN ping por llamada (L, R, F, de uno en uno como hoy, saltando los
apagados por `parkSoloExterior`), el eco se mide por interrupcion (`attachInterruptArg`, CHANGE) y el
timeout es el mismo de `pulseIn` (7 ms). El bloque principal de `loop()` lee con `usRead()` el ultimo
valor. Los demas `leerDistancia()` (INICIO, estacionamiento, setup) siguen bloqueantes y frescos.

Riesgos (el mayor primero):
- **Toda la logica por-loop esta afinada a un periodo de loop largo** (EMA `alpha=0.85` por loop,
  `esquinaDebounce`, `lateralOpenStreak`, `contadorFront`, `medianaFront` de 3 muestras, dt de PIDs en
  ms). Sin los 7-21 ms de `pulseIn` el loop corre mucho mas rapido y esos contadores se cumplen en
  menos tiempo real: el comportamiento cambia. Por eso hay que fijar `-DFOX_LOOP_PERIOD_US=<media medida
  con [LOOP]>` (p. ej. el `mean` del patch 1) y volver a validar esquinas y parking.
- Los valores se repiten entre pings (cada sensor se refresca en >= 3 pings, hasta ~21 ms sin eco):
  `medianaFront` rechaza menos picos y el EMA ve muestras repetidas; la latencia de cada lectura sube
  hasta un ciclo.
- No compilado para ESP32. `micros()`/`digitalRead()` en ISR siguen el patron de `encIsr`; los pines
  ECHO 32/33/35 admiten interrupcion. Requiere arduino-esp32 >= 2.0 (`attachInterruptArg`).
- Un `leerDistancia()` bloqueante sobre un canal mientras el asincrono esta "busy" genera un eco que el
  asincrono tambien ve: da una lectura valida pero se acopla en el tiempo; no se observo en pruebas
  porque no se pudo probar con hardware.
- El SIL del twin no modela esta ruta (`!defined(FOX_SIL)`): el twin sigue viendo pings bloqueantes.
