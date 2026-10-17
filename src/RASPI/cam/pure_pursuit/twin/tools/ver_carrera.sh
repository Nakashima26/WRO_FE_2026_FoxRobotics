#!/usr/bin/env bash
# Abre el twin (ventana "Fox") con la carrera exportada: 3 vueltas + U + paralelo, engrane 1.25, 564 g.
# Uso: pure_pursuit/twin/tools/ver_carrera.sh [semilla]   (espacio=iniciar, n=otra semilla, r=reinicio, esc=salir)
set -euo pipefail
CAM="$(cd "$(dirname "$0")/../../.." && pwd)"
PY="${PY:-python3}"
cd "$CAM"
# Igual que exportar_carrera.py: FW_PARK_CARRERA y luego el JSON del banco.
ARGS=$("$PY" -c '
import json, shlex, sys
from pure_pursuit.twin.tools.exportar_carrera import FW_PARK_CARRERA
d = json.load(open(sys.argv[1]))
a = []
for k, v in {**FW_PARK_CARRERA, **(d.get("fw") or {})}.items(): a += ["--fw", f"{k}={v}"]
for k, v in (d.get("pi") or {}).items(): a += ["--pi", f"{k}={v!r}"]
print(" ".join(shlex.quote(x) for x in a))' pure_pursuit/twin/tools/carrera_bench.json)
export TWIN_MOTOR=fisico TWIN_GEAR=1.25 TWIN_MASS_KG=0.564 TWIN_PI_UART_US_PER_BYTE=21.7 \
       TWIN_FW_PARAMS="serial2_us_per_char=21.7,serial_us_per_char=21.7" FOX_PARK_POS=random PYTHONPATH=.
eval "\"$PY\" -m pure_pursuit.chassis_twin --seed ${1:-3} --preset ruta_perfil --speed 1.5 $ARGS"
