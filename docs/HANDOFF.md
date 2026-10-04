# Handoff — digital twin Fox (rama `digital-twin`)

Punto de entrada para una sesión nueva. El detalle histórico está en `docs/twin_plan.md`
(léelo por secciones con grep, no completo: es largo).

## Cómo trabajar (preferencia del usuario)
- El hilo principal es el **planner**: arma la lista de tareas, delega la ejecución a subagentes,
  verifica lo que reportan en el código/logs antes de aceptarlo, fusiona y documenta.
- Subagentes en **worktree aislado** cuando tocan los mismos archivos que otro; tope de **10 jobs**
  cada uno (22 hilos de CPU compartidos); presupuesto de iteraciones explícito.
- Cada cambio que se queda: commit + entrada en `docs/twin_plan.md`. No commitear `runs/` ni `_build/`.
- Leer logs con grep/resúmenes, nunca completos (tokens).
- Español; términos técnicos en inglés.

## Entorno (Windows)
- venv: `.venv-sim` en la raíz (numpy, opencv-python con GUI, matplotlib, imageio, pytest, ziglang).
- Desde `src/RASPI/cam/`: `PYTHONUTF8=1 ../../../.venv-sim/Scripts/python -m pure_pursuit.chassis_twin --preset hw_nuevo --seed 3170839 [--no-show --log-dir runs/x] < /dev/null`
- Firmware SIL se compila con zig (~5 min la 1ª vez por cambio del .ino; caché por hash).
- Herramientas: `pure_pursuit/twin/tools/` (all.sh = 21 seeds con drive.py, summ3.py, salida.py = harness salida del cajón, triage.py, parkcheck.py…). Salidas en `src/RASPI/cam/runs/` (ignorado).
- Tests: `../../../.venv-sim/Scripts/python -m pytest pure_pursuit/twin/tests -q` (falla conocida previa: `test_corner_hint::test_v2_carries_hint_when_enabled`).

## Presets
- `hw_nuevo` = hardware de competencia: 180×130 mm, batalla 113, rueda 46.32°, cámara 45° a ~97 mm,
  encoder en motor, 4 ToF (L/R/F/B), IMU BNO085, Pi 5 (~28 fps), corrección del mapa con sonares.
  **Todo el trabajo nuevo se mide aquí.**
- `giro_rapido` = carro actual (PWM sin encoder, MPU6050, 14 fps). Solo referencia.
- Dimensiones/montajes: fuente única en `config.py` (bloque GEOMETRÍA DEL VEHÍCULO).

## Estado (2026-10-04, HEAD 1250703 + lo que entreguen los agentes abajo)
Carrera completa hw_nuevo 28 fps, 21 seeds (1-20 + 3170839): suma tc 177/252, 11 llegan a 12 giros,
0 terminan limpios (≤5 mm fuera del cajón, sin contacto). Salida del cajón resuelta (harness 40/41).

Hecho y fusionado: fix del verde en esquina (CW), mapa digital (pose eje trasero, proyección,
rumbo de arranque real, corrección con sonares), encoder+4 ToF, BNO085, Pi 5 + constantes en frames
independientes de fps, arranque al ras del borde interior del cajón, salida del cajón sin reversa.

## Trabajo en vuelo al cerrar la sesión anterior (revisar primero)
Dos worktrees en `.claude/worktrees/` (ver `git worktree list`):
- `agent-a6501da593f9e5146` (rama `worktree-agent-a6501da593f9e5146`, base cf8f97d — anterior a la salida nueva):
  **T15b estacionamiento en PARALELO**. Fases de tiempo → distancia de encoder + barrido de parámetros
  (mejor combo vista: POSTE2_M100 / ENDEREZA10 / FINAL_CM4 / ANG_IN_D50). Harness `twin/tools/prepark.py`
  arranca con tc=12 y **se salta la vuelta en U**: en la carrera (seed 6) la media vuelta choca el cajón.
  Pendiente: harness desde tc=11; la centerline de la Pi no está adaptada a la aproximación al cajón.
- `agent-a47a3f8910d0a9fc8` (rama `t15c-esquinas`, base 1250703): **T15c esquinas**. Generalizar
  "lata en la boca de la recta siguiente" a ambos sentidos/colores (falta el espejo CCW: seed 11,
  rojo S/T3); caso rojo por DENTRO que retrasa el giro en la vuelta 1 (seed 3170839, esquina S→W);
  línea del mapa sin validar contra la lata viva (runtime_nuevo.py ~1847 `path_points = mapped`);
  hitbox del carro en el mapa (hoy es un punto); seeds 1,3,4,15 chocan rojo T2 de la recta del cajón.
Ambos agentes se detuvieron; todo lo que tenían quedó commiteado en sus ramas (nada sin commitear):
- `worktree-agent-a6501da593f9e5146`: `ae1e3ed` (harness pre_park tc=12 + paralelo ruta A por encoder,
  completo) y `203d36c` WIP (estaba metiendo: empate de media vuelta, hook SIL tc=11, modo "curva" del
  harness — incompleto, sin validar).
- `t15c-esquinas`: `9cbc6ad` WIP (digital_map/config: lata en la boca para ambos sentidos, `huella.py`
  hitbox, harness `twin/tools/esquina.py` + `esq_cmp.py` — incompleto, sin validar; estaba por lanzar
  la validación). Sin entrada T15c en twin_plan.md todavía.
Para cada uno: ver `git -C <worktree> log` y `diff --stat`; si hay commits útiles, fusionar a
`digital-twin` (conflictos esperables en PurePursuit.ino/runtime_nuevo.py/twin_plan.md), correr tests,
validar con `tools/all.sh` y `tools/salida.py all`, y borrar el worktree.

## Siguiente (en orden)
1. **T16 rendimiento** (plan en twin_plan.md): hoy ~7 s reales por s simulado. Modo "percepción oráculo"
   (saltar render+visión, detecciones desde la verdad del twin con ruido/latencia/FOV/oclusión) para
   iterar lógica ×5-10; perfilar render + process_frame + física/ctypes; validar final con visión completa.
2. Terminar T15b (estacionamiento paralelo incl. vuelta en U) y T15c con el simulador rápido.
3. T12 escenarios explícitos por cartas (`--scenario`, `--noise-seed`, enumerador) y corregir
   `REQUIRED_CARDS` en wro_field.py:226 (debe ser 26, 27, 32, 33) después de guardar los escenarios de las seeds actuales.
4. T11 mapa congelado tras vuelta 1 + ruta fija vueltas 2-3 (visión solo como sanity check).
5. T14 driver BNO085 en el firmware real; recalibrar `bev_calib.npz` al mover la cámara.

## Pendiente de medir en el carro real
Altura real del lente y grosor de la LiPo; montajes de US/ToF; PPR del encoder y pull-ups (GPIO 34/39);
GPIO15 libre (XSHUT ToF frontal); VL53L0X vs VL53L1X; radio real de la contravuelta de la salida
(`INICIO_R_CONTRA_MM`); fps reales en la Pi 5.
