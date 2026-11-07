#!/bin/bash
# uso: bench.sh <worktree> <nombre> <preset_base> <preset_cand> [extra_cand_json]
SP="$(cd "$(dirname "$0")" && pwd)"
cd $1/src/RASPI/cam || exit 1
N=$2; PB=$3; PC=$4; EXC=${5:-null}
SEEDS="$(seq 1 41) 3170839"; J=${JOBS:-12}
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUTF8=1 PYTHONPATH=.
for nz in 1 2 3 4 5; do
  # BASE_REF=<runs de referencia>: enlaza su base_n* en vez de correrla o copiarla (~1 GB menos)
  [ -n "$BASE_REF" ] && [ ! -e runs/$N/base_n$nz ] && mkdir -p runs/$N && ln -s "$BASE_REF/base_n$nz" runs/$N/base_n$nz
  mkdir -p runs/$N/base_n$nz runs/$N/cand_n$nz
  printf "%s\n" $SEEDS | xargs -P $J -n1 "$SP/wk.sh" $nz $N/base_n$nz $PB null
  printf "%s\n" $SEEDS | xargs -P $J -n1 "$SP/wk.sh" $nz $N/cand_n$nz $PC "$EXC"
  echo "done n$nz $(date +%T)"
done
echo ALL_DONE
