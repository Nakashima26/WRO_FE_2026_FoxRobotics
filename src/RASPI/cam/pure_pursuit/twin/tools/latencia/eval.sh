#!/bin/bash
# Banco con la latencia medida en la Pi 5 real (cuadro x4.28 y DP x5.8 sobre el tiempo de CPU de la Mac,
# primer plan 1.95 s). uso (cwd = src/RASPI/cam):
#   eval.sh <tag> [json_pi_extra] [P]   -> 48 carreras (semillas 1-24 x ruidos 1,2) y resumen
# COSTO=0 apaga la latencia (twin normal). FOX_PY = python del venv. Salida en $LAT_OUT (runs/latencia.noindex).
D=$(cd "$(dirname "$0")" && pwd)
tag=$1; ex=${2:-'{}'}; P=${3:-5}
if [ "${COSTO:-1}" = 0 ]; then ff=0; fd=0; first=0; else ff=4.28; fd=5.8; first=1.95; fi
for nz in 1 2; do for s in $(seq 1 24); do echo "$nz $s"; done; done | \
  TWIN_PI_EXTRA="$ex" TWIN_COST_FIRST=$first xargs -P "$P" -L 1 bash -c "\"$D/uno.sh\" $tag $ff $fd \$0 \$1"
"${FOX_PY:-python3}" "$D/resumen.py" "$tag"
