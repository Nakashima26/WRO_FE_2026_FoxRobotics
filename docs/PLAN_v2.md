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

## Planificador en el twin (2026-10-10): NO supera al control; se detuvo a pedido del usuario
41 seeds, `hw_nuevo --solo-cajon`, Mac. Terminan: P0 control 11 (tc 6.0) · P1 (margen pilar 25/pared 50) 0 (tc 0.8, 26 choques <10 s)
· Q1 (100/80) 3 (tc 3.4) · Q2 (150/100) 0: `LinAlgError: Matrix is not positive definite` en `route_planner._solve_qp:217`
(el QP deja de ser factible). Causa del fallo de P1 (verificada en seed 1): ruta a 38 mm del pilar T1 de la sección del
cajón + salida del cajón con 8–14° de rumbo desalineado → el frente (147.6 mm) lo toca; 26/41 igual. Rama `fox/plan-twin`
(WRO_v2_plan, e3ca8c6: márgenes parametrizables `ROUTE_*_MARGIN_MM`, sin fusionar). Sin probar: P2 con margen bueno,
replans por carrera, error lateral, tiempo por vuelta. Si se retoma: capturar la excepción del QP con fallback a `_build_line`,
empalme de rumbo tras la salida del cajón y margen ~100 mm. El planificador sigue APAGADO (`ROUTE_PLANNER=False`).

## Línea A*/DP por carta (2026-10-12)
- [x] Lattice Frenet + A* (recta) + DP cost-to-go (esquina): `racing_line_astar.py`.
- [x] CLI `twin/tools/line_card.py` (métrica vs centro vs slice de `plan_lap`).
- [x] Tests carta 26 + seed 3170839 sección N (`test_racing_line_astar.py`).
- [x] `plan_lap_joint` (DP anillo) > `_build_line` en seed 3170839; `--lap --mode joint`.
- [ ] Enganche DigitalMap/PP (T11) con la polilínea joint; QP global sigue apagado.

## Siguiente (orden)
1. [ ] Seguir A*/DP carta-por-carta (encadenar → anillo) antes de reintentar el QP global en runtime.
2. [ ] Integrar el planificador al runtime si la línea por carta encaja: lookahead constante sobre la ruta, replan en hilo al
   confirmar un asiento nuevo (votos ≥3), histéresis, sanity-check con el BEV. Medir tiempo de replan en la Pi 5.
3. [ ] CCW: contar cuántas de las 120 seeds llegan a la U por sentido; revisar los 2 choques CCW que no son de la U
   (cajón magenta / pared exterior, s112); revisar `blocks_turn` sin espejo CCW (twin_plan ~:235).
4. [ ] U cuando arranca a ≤35 cm de la pared exterior (radio ya mínimo): arrancarla antes / más lejos de la pared.
5. [ ] `calibration.py`: usar `vision.open_camera` (40 fps + BGRx) en vez de su pipeline de 30 fps.
6. [ ] Aplicar los parches de `twin/tools/calib/patches/` (medir `loop()`, `pulseIn` asíncrono) solo con datos del carro.
7. [ ] Decidir el umbral de señal del ToF del twin (hoy 250 mm de pared negra → lecturas inválidas lejos): por sensor o por material.
8. [ ] Choque físico del twin (rebote/empuje de pilares en vez de terminar la carrera) y jitter fotométrico de la cámara
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
