# Herramientas del twin (scratch promovido)

Desde `src/RASPI/cam/`. Las salidas van a `runs/` (ignorado por git).

- `all.sh <tag> [jobs]`: corre `drive.py` con `hw_nuevo` en seeds 1-20 + 3170839 (`SEEDS=`, `EXTRA=` para cambiarlo) y resume con `summ3.py`.
- `drive.py <seed> <dir> [preset] [extra_json]`: una corrida con logs completos (`pi.log`, `fw_debug.log`, `trace.csv`, `field.txt`).
- `summ3.py <dir> <seeds…>`: tabla seed/stop/tc/choque/fuera_mm; limpio = terminado sin choque y ≤ 5 mm fuera del cajón.
- `salida.py all <tag> [jobs]`: harness corto de la salida del cajón (T15a): CW/CCW × cartas de la recta del cajón + las 21 semillas, 14 s; holguras de la huella y lado de paso de cada señal de la recta. `EXTRA='{"src":"head"}'` = base con el .ino de HEAD.
- `paridad.py [preset] [--ino PATH]` (`PYTHONPATH=.`): tabla twin <-> carro (.ino, config.py, preset resuelto, IMU, periodo de la Pi, verdad inyectada al mapa, servo). Cada diferencia necesita motivo en `ACEPTADAS`; exit 1 si alguna no lo tiene, 2 si el preset nombra algo que no existe. Solo lee.
- `triage.py`, `align.py`, `parkcheck.py`, `hderr.py`, `odoerr.py`: diagnóstico (falla por categoría, pose del mapa vs real, huella final en el cajón, error de rumbo y de odometría).
