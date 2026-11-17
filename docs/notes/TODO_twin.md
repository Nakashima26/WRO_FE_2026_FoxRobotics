# TODO: Digital Twin Analysis & Fix

## Fase 1: Análisis ✅ COMPLETADO

### Hallazgos clave del análisis graph + docs + código:

1. **FPS/Timing**: Pi 5 corre a 40fps (`PI_FPS=40`), tests lockeados a 14fps, `hw_nuevo` preset sobreescrito a 40fps con `pi_period_s=0.025`. `cam_latency_s=0.04` está hardcodeado.
2. **REQUIRED_CARDS bug**: `(22,23,28,29)` debería ser `(26,27,32,33)` según app oficial → cambia significado de TODAS las seeds.
3. **`blocks_turn()` early exit**: En seed 3170839, la salida temprana `if along>1900 and lat<-80` suelta prio demasiado pronto cuando el verde está en 1a columna de la recta siguiente.
4. **Proyección BEV→campo**: Le falta `BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM` (100mm) + `pie→centro lata` → deriva de ~120mm en pose.
5. **ppServoGain=1.0**: El ESP manda steer_deg como servo_deg pero 1° servo ≈ 0.627° rueda → el pure pursuit opera a ~63% de ganancia.
6. **Camara params**: `CAMERA_HEIGHT_MM=97`, `CAMERA_FWD_MM=140` son SUPUESTOS no medidos.
7. **ToF black wall**: Umbral 250mm da lecturas inválidas en pared negra.
8. **DigitalMap no se resetea en GO**: Vota desarmado.

## Fase 2: Planificación (en curso)

## Fase 3: Implementación
- [x] **Fix #1 (HIGH)**: REQUIRED_CARDS en wro_field.py → (26, 27, 32, 33)
- [ ] **Fix #2 (HIGH)**: blocks_turn() early exit en digital_map.py
- [ ] **Fix #3 (MEDIUM)**: Proyección BEV→campo + offsets
- [ ] **Fix #4 (MEDIUM)**: DigitalMap reset en GO
- [ ] **Fix #5 (LOW)**: Documentar params no medidos como warnings al arrancar
- [ ] **Fix #6 (LOW)**: Camara params: agregar notas "SUPUESTO" en logs de arranque

## Fase 4: Validación
- [ ] Correr tests existentes (`pytest twin/tests`)
- [ ] Verificar seed 3170839 con fix #2
- [ ] Verificar seeds 1-6 con fix #1
- [ ] Comparar paridad twin↔real
