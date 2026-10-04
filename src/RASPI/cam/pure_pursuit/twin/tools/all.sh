#!/bin/bash
# uso: pure_pursuit/twin/tools/all.sh <tag> [jobs]  (desde src/RASPI/cam) -> hw_nuevo seeds 1-20 + 3170839 con drive.py
tag=$1; J=${2:-10}
EX="${EXTRA:-null}"
SEEDS="${SEEDS:-$(seq 1 20) 3170839}"
mkdir -p runs/$tag
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
printf "%s\n" $SEEDS | xargs -P $J -I{} bash -c "PYTHONUTF8=1 PYTHONPATH=. ../../../.venv-sim/Scripts/python pure_pursuit/twin/tools/drive.py {} runs/$tag/s{} hw_nuevo '$EX' < /dev/null > runs/$tag/s{}.out 2>&1"
PYTHONUTF8=1 ../../../.venv-sim/Scripts/python pure_pursuit/twin/tools/summ3.py runs/$tag $SEEDS
