# Herramientas del twin (scratch promovido)

Desde `src/RASPI/cam/`. Las salidas van a `runs/` (ignorado por git).

- `all.sh <tag> [jobs]`: corre `drive.py` con `hw_nuevo` en seeds 1-20 + 3170839 (`SEEDS=`, `EXTRA=` para cambiarlo) y resume con `summ3.py`.
- `drive.py <seed> <dir> [preset] [extra_json]`: una corrida con logs completos (`pi.log`, `fw_debug.log`, `trace.csv`, `field.txt`).
- `summ3.py <dir> <seeds…>`: tabla seed/stop/tc/choque/fuera_mm; limpio = terminado sin choque y ≤ 5 mm fuera del cajón.
- `triage.py`, `align.py`, `parkcheck.py`, `hderr.py`, `odoerr.py`: diagnóstico (falla por categoría, pose del mapa vs real, huella final en el cajón, error de rumbo y de odometría).
