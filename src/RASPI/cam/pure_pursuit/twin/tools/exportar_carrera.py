"""Exporta la config del mejor twin (preset + bench JSON) al carro real.

Escribe pure_pursuit/carrera.json (lo carga runtime_nuevo.py) y un sketch
ESP32/PurePursuitCarrera/PurePursuitCarrera.ino ya parcheado con los overrides.

Uso (desde src/RASPI/cam):
  PYTHONPATH=. ../../../.venv/bin/python pure_pursuit/twin/tools/exportar_carrera.py <preset> <bench.json> [--pi-baud N]
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

from pure_pursuit.twin.sim import resolve_preset
from pure_pursuit.twin.firmware.build import load_ino_text, _apply_overrides

# Solo existen en el twin (compensación de la cámara simulada); el carro real no las usa.
TWIN_ONLY = {"PARK_PINK_RATIO_MIN"}

# La carrera completa estaciona: U + paralelo en modo ruta (docs/HANDOFF_PARKING.md). Pisan el
# PARK_NINGUNO de los presets de ruta; el bench JSON puede cambiarlos.
FW_PARK_CARRERA = {"PARK_MODO": "PARK_PARALELO", "PARK_UTURN_RUTA_ALONG_MM": "600",
                   "PARK_UTURN_ALEJAR_CM": "60"}

_SRC = Path(__file__).resolve().parents[5]          # .../src
_PI_DIR = Path(__file__).resolve().parents[2]       # .../pure_pursuit
_SKETCH_BASE = _SRC / "ESP32" / "PurePursuit"
_SKETCH_CARRERA = _SRC / "ESP32" / "PurePursuitCarrera"


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("preset")
    p.add_argument("bench_json", type=Path)
    p.add_argument("--pi-baud", type=int, default=None,
                   help="baud Pi<->ESP (BAUDRATE en carrera.json, PI_BAUD y USB_BAUD en el .ino)")
    args = p.parse_args()

    bench = json.loads(args.bench_json.read_text(encoding="utf-8"))
    cfg = resolve_preset(args.preset, fw_overrides=bench.get("fw"), pi_overrides=bench.get("pi"))
    fuente = args.bench_json.name

    # Pi: carrera.json, claves que runtime_nuevo.py aplica sobre config.C.
    pi = {k: v for k, v in cfg["pi_overrides"].items() if k not in TWIN_ONLY}
    if args.pi_baud is not None:
        pi["BAUDRATE"] = args.pi_baud
    pi["_preset"] = args.preset
    pi["_fuente"] = fuente
    pi_path = _PI_DIR / "carrera.json"
    pi_path.write_text(json.dumps(pi, indent=1, sort_keys=True) + "\n", encoding="utf-8")

    # ESP: sketch con los overrides de fw y los defines (#ifndef en el .ino dejan que ganen).
    fw = {**cfg["fw_overrides"], **FW_PARK_CARRERA, **(bench.get("fw") or {})}
    defines = dict(cfg.get("fw_defines", {}))
    if args.pi_baud is not None:
        defines["PI_BAUD"] = str(args.pi_baud)
        defines["USB_BAUD"] = str(args.pi_baud)
    patched = _apply_overrides(load_ino_text("worktree"), fw)
    cabecera = (f"// GENERADO por exportar_carrera.py desde {args.preset} + {fuente}. "
                "No editar: editar PurePursuit.ino y regenerar.\n")
    defs = "".join(f"#define {k} {v}\n" for k, v in defines.items())

    _SKETCH_CARRERA.mkdir(parents=True, exist_ok=True)
    ino_path = _SKETCH_CARRERA / "PurePursuitCarrera.ino"
    ino_path.write_text(cabecera + defs + patched, encoding="utf-8")
    for extra in _SKETCH_BASE.iterdir():
        if extra.suffix in (".h", ".cpp"):
            shutil.copy(extra, _SKETCH_CARRERA / extra.name)

    print(f"carrera: preset={args.preset} fuente={fuente}")
    print(f"  pi: {len(pi) - 2} claves -> {pi_path}")
    print(f"  fw: {len(fw)} overrides, {len(defines)} defines -> {ino_path}")


if __name__ == "__main__":
    main()
