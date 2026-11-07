# Hoja de medición — calibración del twin con el carro v2

Fecha: ____  Batería (V en reposo / al final): ____ / ____  Superficie: ____  Operador: ____

Antes de nada, una sola vez:
1. Cargar `src/ESP32/CalibFox/CalibFox.ino` (`arduino-cli compile/upload --fqbn esp32:esp32:esp32`, librerías `MPU6050_tockn` y `VL53L1X` de Pololu; `-DCALIB_MPU=0 -DCALIB_TOF=0` si no las tienes). La Pi NO participa.
2. Ruedas motrices en el suelo (la carga cambia la velocidad), pista lisa y recta de >= 2.5 m, batería cargada. Anotar el voltaje: el PWM es lazo abierto y v depende de Vbat.
3. Desde `src/RASPI/cam`: `$env:PYTHONUTF8=1; $env:PYTHONPATH="."`, y
   `python -m pure_pursuit.twin.tools.calib.calib_capture --port COM5 --out runs/calib/<fecha> --vbat <V> <subcomando>`.
   Ensayo en seco sin carro: `--port mock`.
4. Seguridad: cualquier tecla enviada por el puerto aborta la corrida; `STOP` frena (coast). Tope de 20 s por corrida. CalibFox espera 300 ms antes de invertir el puente H (TB6612).

Orden recomendado (el orden importa: 1 escala a todo lo demás):

**La medición 1 (cuentas/mm) va SIEMPRE primero: escala todas las velocidades.**

| # | Medición | Subcomando | Repeticiones | Tiempo aprox. |
|---|----------|-----------|--------------|---------------|
| 1 | cuentas/mm (rodar con regla) | `enc --dist 1000 --reps 5` | 5 | 8 min |
| 2 | velocidad vs PWM (adelante y reversa) | `ramp --reps 3` | 3 pares | 10 min |
| 3 | zona muerta fina | `deadband --reps 3` | 3 pares | 5 min |
| 4 | escalón desde reposo (tau_drive) | `step --pwms 60,90,120 --reps 3` | 3 por PWM, +/- | 12 min |
| 5 | coast (tau_coast) y distancia al cortar | `coast --pwms 100,120 --reps 3` (y `--brake` aparte) | 3 | 10 min |
| 6 | velocidad con volante (scrub) | `lock --pwm 100 --servos 90,50,30 --reps 3` | 3 | 8 min |
| 7 | servo → rueda y radio de giro | `steer --servos 60,70,80,100,110,120,130,150 --reps 2` + transportador | 2 | 25 min |
| 8 | duración del loop() | `looptime --n 500` (con HC-SR04 y MPU conectados) | 3 corridas | 5 min |
| 9 | ToF: tasa, status, alcance | `tof --sensor L --dists 1000,1300,1500` (y F, R, B) | 1 por variante | 25 min |
| 10 | radio mínimo y velocidad a tope | incluido en 7 (servo 30/160 o 19/161) | 3 | +5 min |

Total ≈ 2 h 10 min sin imprevistos.

## 1. Cuentas por mm (encoder)
- Marca en el suelo dos líneas a **1000 mm** exactos (cinta métrica, no a ojo). Alinear el centro de la huella de una rueda motriz con la 1ª.
- El script pide empujar el carro a mano en línea recta hasta la 2ª marca (sin patinar), luego devolverlo a la 1ª.
- Anotar: cuentas ida: ____ ____ ____ ____ ____ | neto al volver: ____ (debe ser ~0; si no, se pierden cuentas) | `enc_err`: ____
- Hipotesis a verificar (no medida): el 20.73 del firmware (PurePursuit.ino l.434) supone 7 PPR x4 x50 x2 / (pi*43) del encoder del N20. Si el encoder del Pololu es el tipico de 12 CPR en cuadratura, son 12 x 50 x 2 / (pi*43) = ~8.9 cuentas/mm: con 20.73 la odometria (od, px, py, parking) saldria ~2.3 veces demasiado larga. calib_fit_motor avisa si lo medido difiere mas de 10 % de 20.73.
- Criterio de calidad: dispersión < 1 %. Si `enc_err > 0`, el ISR pierde flancos (pull-ups en GPIO34/39, ruido del motor).

## 2-3. Velocidad vs PWM y zona muerta
- Ruedas en el suelo, volante recto (SERVO 90). Cada escalón dura 800 ms (≥ 3 tau); si el script avisa "sin estabilizar", subir `--hold`.
- Anotar por corrida: Vbat inicial/final, superficie, si el carro se desvió. Qué mirar: ¿es lineal v(PWM)? El twin supone `v = 3.5·(PWM−25)` mm/s.
- Repeticiones: 3 pares adelante/atrás (el par devuelve el carro al punto de partida). Si Vbat cae > 0.4 V entre repeticiones, recargar.

## 4-5. tau_drive y tau_coast
- `step`: PWM 60, 90, 120 desde reposo, 2.5 s. Hace + y − alternados para volver a la salida.
- `coast`: corre a PWM 100 1.5 s, corta con A1=A2=LOW y registra 2 s. Medir con cinta cuánto rodó (compara con `stop_mm`): ____ mm. Repetir con `--brake` (como `setMotor(0)` del .ino) y anotar ambas distancias.

## 6. Scrub
- Misma velocidad (PWM 100) con servo 90, 50 y 30 (o los topes reales); el carro hace círculos, dejar espacio de ~0.6 m de radio.

## 7 y 10. Servo → rueda, radio mínimo
Método A (radio, recomendado): pegar un marcador (plumón) en el **centro del eje trasero** y circular sobre papel; medir el radio de ese círculo (o el diámetro) en mm. Una fila por vuelta.
Método B (transportador/foto): con el carro en el aire y el servo fijo (`SERVO <deg>` desde `calib_capture raw`), medir el ángulo de la rueda interior (y exterior) respecto al eje longitudinal en una foto cenital. Anotar en `steer.csv` como `servo_deg,inner_deg,outer_deg`.

| servo (deg) | R medido (mm) vuelta 1 | vuelta 2 | rueda int. (deg) | rueda ext. (deg) | ¿roza/zumba el servo? |
|-------------|-----|-----|-----|-----|-----|
| 160 / 30 (topes) | | | | | |
| 150 / 40 | | | | | |
| 130 | | | | | |
| 110 | | | | | |
| 100 | | | | | |
| 80 | | | | | |
| 70 | | | | | |
| 60 | | | | | |
| 19 / 161 (tope v2 ±71°) | | | | | |

- Medir también el **centro**: SERVO 90 y verificar que avanza recto 2 m; si no, anotar el servo que lo hace recto: ____.
- Después correr `calib_fit_steer.py steer.csv`: da ganancia izq/der, R mín, y `ppServoGain` para que rueda = steer_deg (hoy 1.0).

## 8. Duración del loop()
- Conectar los 3 HC-SR04 y el MPU. Correr `looptime --n 500` 3 veces. Anotar `total` min/media/p99/max (µs) y `timeouts` de cada ping.
- Referencia a verificar: `pulseIn` bloquea hasta 7 ms por sensor (PurePursuit.ino l.2579) y `mpu.update()` va dos veces (l.3196, l.3287). Es un modelo del peor caso; el valor real se mide con `tools/calib/patches/looptime_instrument.patch (ver NOTA_RIESGOS.md)`.

## 9. ToF (VL53L1X)
- Blanco plano claro perpendicular al sensor a 1.0, 1.3 y 1.5 m (metro). Variantes: `SHORT:20` (presupuesto 20 ms), `SHORT:DEF` (50 ms por defecto de la librería), `LONG:33`, `LONG:DEF`.
- Anotar por variante: `rate_hz` (la lib de Pololu fija 50 ms en init: startContinuous(33) real ≈ 20 Hz salvo que se fije el presupuesto antes), % de lecturas con status 0, histograma de `range_status`, mm medio vs real.
- Repetir con luz de la sala encendida y con pared negra de la pista (el twin supone `black_wall_max_mm`=250).

## Después de medir
```
python -m pure_pursuit.twin.tools.calib.calib_fit_motor runs/calib/<fecha>      # -> calib_params.patch, calib_ino_consts.patch
python -m pure_pursuit.twin.tools.calib.calib_fit_steer runs/calib/<fecha>/steer.csv
git apply --check runs/calib/<fecha>/calib_params.patch && git apply runs/calib/<fecha>/calib_params.patch
python tools/paridad.py   # twin vs .ino deben seguir alineados (ODOM_*, ENC_CUENTAS_POR_MM)
```
Los parches de `.ino` no se aplican solos; revisar antes (cambiar ODOM_*/ENC_* afecta la odometría de parking).

