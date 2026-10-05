#!/bin/bash
# uso: pure_pursuit/twin/tools/all.sh <tag> [jobs]  (desde src/RASPI/cam) -> hw_nuevo seeds 1-20 + 3170839 con drive.py
# SOLO_CAJON=1: pasa --solo-cajon a drive.py (un contacto con la pared exterior no corta la carrera).
# TWIN_NOISE_SEED=k: mismo campo de cada seed, otro ruido de sensores (sin definir = el de siempre).
#   Para el ensamble de K semillas de ruido usar tools/ens.sh.
# PY=<python>: intérprete (default el .venv-sim de Windows; en la Mac el .venv del usuario).
tag=$1; J=${2:-10}
EX="${EXTRA:-null}"
SEEDS="${SEEDS:-$(seq 1 20) 3170839}"
SC="${SOLO_CAJON:+--solo-cajon}"
PY="${PY:-../../../.venv-sim/Scripts/python}"
mkdir -p runs/$tag
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
[ -n "$TWIN_NOISE_SEED" ] && export TWIN_NOISE_SEED
printf "%s\n" $SEEDS | xargs -P $J -I{} bash -c "PYTHONUTF8=1 PYTHONPATH=. '$PY' pure_pursuit/twin/tools/drive.py {} runs/$tag/s{} hw_nuevo '$EX' $SC < /dev/null > runs/$tag/s{}.out 2>&1"
PYTHONUTF8=1 "$PY" pure_pursuit/twin/tools/summ3.py runs/$tag $SEEDS
