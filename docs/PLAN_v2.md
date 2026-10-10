# Plan v2 — carro v2 + twin (actualizado 2026-10-10)

Reglas de trabajo: este plan se actualiza al cerrar cada tarea; no se pregunta entre pasos salvo que haga falta
que el usuario vea el carro o la simulación. El código lo escriben subagentes Haiku con especificación precisa
(archivo, función, comportamiento, comando de prueba) y el planner verifica ejecutando las pruebas. Corridas
pesadas en la Mac (`ssh jesse@192.168.68.59`, solo `~/Projects/fox_v2_*`), una vez por celda, seeds fijas, nunca
Mac contra Windows. Todo en worktrees; no editar los de otras sesiones (`t15*`, `t16*`, `t17*`).
Contexto completo en `docs/HANDOFF.md` ("Estado 2026-10-10").

## Hecho y verificado (rama `fox/seed-3170839-red-object`)
- [x] Geometría v2 del GLB en `config.py` (largo/ancho/voladizos/vía, montajes de sensores, giro 44.54° equivalente).
- [x] Servo ±71° → topes 19..161 en `.ino` y twin; `paridad.py` resuelve `SERVO_MIN_DEG/MAX_DEG`.
- [x] Cámara Pi 5 a 40 fps (`vision.open_camera`, `camera_tuning/`, `cam_web.py`), `PI_FPS=40`.
- [x] Firmware v2: pinout ToF de los sketches probados, BNO085 por UART-RVC (acumula delta por paquete), defaults `FOX_*`=1 fuera de SIL.
- [x] ToF: Short 20 ms L/R/trasero, Long 33 ms frontal; twin con ToF por sensor; `hw_nuevo` a 40 fps.
- [x] Twin sobre la rama integrada: suite 133 pasan / 1 falla previa (`test_corner_hint`); 41 seeds sin excepciones.
- [x] U de 180°: `EXT_OBJ` 40→28 (ajustable), `ESPERA_MM` disponible en 0.
- [x] Planificador de ruta offline (`route_planner.py`) + tests; herramientas de calibración (`CalibFox.ino`, `twin/tools/calib/`).

## En curso
- [ ] Probar el planificador en el twin (Mac): P0 control, P1 `ROUTE_PLANNER=True`, P2 P1 + lookahead constante.
  Criterio: sin excepciones, ruta usada (no línea vacía), margen a latas ≥ al de P0, jitter (Δsteer std, cambios de signo/s) menor.
  Límite: el twin no modela el bloqueo de ~0.2 s del replan.

## Siguiente (orden)
1. [ ] Integrar el planificador al runtime si P1/P2 funcionan: lookahead constante sobre la ruta, replan en hilo al
   confirmar un asiento nuevo (votos ≥3), histéresis, sanity-check con el BEV. Medir tiempo de replan en la Pi 5.
2. [ ] CCW: contar cuántas de las 120 seeds llegan a la U por sentido; revisar los 2 choques CCW que no son de la U
   (cajón magenta / pared exterior, s112); revisar `blocks_turn` sin espejo CCW (twin_plan ~:235).
3. [ ] U cuando arranca a ≤35 cm de la pared exterior (radio ya mínimo): arrancarla antes / más lejos de la pared.
4. [ ] `calibration.py`: usar `vision.open_camera` (40 fps + BGRx) en vez de su pipeline de 30 fps.
5. [ ] Aplicar los parches de `twin/tools/calib/patches/` (medir `loop()`, `pulseIn` asíncrono) solo con datos del carro.
6. [ ] Decidir el umbral de señal del ToF del twin (hoy 250 mm de pared negra → lecturas inválidas lejos): por sensor o por material.
7. [ ] Choque físico del twin (rebote/empuje de pilares en vez de terminar la carrera) y jitter fotométrico de la cámara
   (`camera.py`), contrastado con el corpus real (`tools/vision_corpus.py`, `detcheck_all.py`).

## Bloqueado: necesita el carro físico
- [ ] Compilar `PurePursuit.ino` para ESP32 (`arduino-cli`, core `esp32:esp32` ≥3.0, libs MPU6050_tockn y VL53L1X de Pololu).
- [ ] Verificar signo del yaw del BNO (`BNO_YAW_SIGN`), GPIO 0/15 como XSHUT sin afectar el arranque.
- [ ] Medir (HOJA_MEDICION.md, ~2 h 10 min): cuentas/mm (¿20.73 o ~8.9?), velocidad vs PWM (PWM 120), zona muerta, tau,
  scrub, servo→rueda y radio mínimo (¿53° es solo la interior?), duración de `loop()`, ToF Short vs 50 ms.
- [ ] Posición real de la cámara (altura, adelanto, inclinación) y recalibrar `bev_calib.npz`; luego pose de cámara del twin.
- [ ] Umbral de rosa real (`pick_color.py`) en el cajón; decidir `ppServoGain` con el mapeo servo→rueda medido.
- [ ] fps reales del pipeline completo en la Pi 5 (si <40: `FOX_PI_FPS` y `pi_period_s` del twin).

## Descartado (con datos en HANDOFF): STM32, Gazebo, Nav2/SLAM completos, LQR; cambiar `WHEELBASE_PX`; espera extra antes de la U.
