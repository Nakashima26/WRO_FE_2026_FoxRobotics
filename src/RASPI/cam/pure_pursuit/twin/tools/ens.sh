#!/bin/bash
# Ensamble de ruido de sensores (T16ens). Desde src/RASPI/cam:
#   pure_pursuit/twin/tools/ens.sh <tag> <K> [jobs]
#       carrera hw_nuevo: SEEDS (default 1-20 + 3170839) x TWIN_NOISE_SEED 1..K
#       -> runs/<tag>/s<seed>_n<k>/ (+ s<seed>_n<k>.out), como all.sh/drive.py.
#   PREPARK=1 pure_pursuit/twin/tools/ens.sh <tag> <K> [jobs]
#       prepark.py (100 escenarios) x --noise-seed 1..K -> runs/<tag>/n<k>/<escenario>/
#       (+ runs/<tag>/n<k>/summary.json, runs/<tag>/n<k>.out). PREPARK_ARGS: más opciones
#       de prepark.py (--fw, --par, --perts, --max-time...).
# Mismas variables que all.sh: SEEDS, SOLO_CAJON=1 (--solo-cajon), EXTRA (json de drive.py), PY.
# K0=k0: corre solo las réplicas k0..K, para agrandar un ensamble ya corrido con 1..k0-1
#   (el twin es determinista en un host: repetirlas daría lo mismo); suma la pared a wall.txt.
# Al final resume con ens.py runs/<tag>; comparar dos ensambles con ens_cmp.py A B.
# Comparar siempre ensambles del MISMO host (Mac contra Mac).
tag=$1; K=$2; J=${3:-8}; K0=${K0:-1}
if [ -z "$tag" ] || [ -z "$K" ]; then
  echo "uso: [PREPARK=1] $0 <tag> <K> [jobs]" >&2; exit 2
fi
EX="${EXTRA:-null}"
SEEDS="${SEEDS:-$(seq 1 20) 3170839}"
SC="${SOLO_CAJON:+--solo-cajon}"
PY="${PY:-../../../.venv-sim/Scripts/python}"
mkdir -p "runs/$tag"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUTF8=1 PYTHONPATH=.
t0=$(date +%s)
if [ -n "$PREPARK" ]; then
  for k in $(seq "$K0" "$K"); do
    "$PY" pure_pursuit/twin/tools/prepark.py "$tag/n$k" --jobs "$J" --noise-seed "$k" $SC $PREPARK_ARGS \
      < /dev/null > "runs/$tag/n$k.out" 2>&1
    echo "n$k: $(tail -n 1 "runs/$tag/n$k.out")"
  done
else
  # Una compilación del firmware antes de lanzar los jobs (si no, cada uno compila la suya).
  "$PY" -c "import json, sys
from pure_pursuit.twin import sim as S
from pure_pursuit.twin.firmware import build as B
e = json.loads(sys.argv[1]) or {}
c = S.resolve_preset('hw_nuevo', fw_overrides=e.get('fw'), fw_defines=e.get('def'))
B.build(overrides=c['fw_overrides'], defines=c.get('fw_defines'), source=c['fw_source'])" "$EX" < /dev/null || exit 1
  for s in $SEEDS; do for k in $(seq "$K0" "$K"); do echo "$s $k"; done; done \
    | xargs -P "$J" -n 2 bash -c "TWIN_NOISE_SEED=\$2 '$PY' pure_pursuit/twin/tools/drive.py \$1 runs/$tag/s\$1_n\$2 hw_nuevo '$EX' $SC < /dev/null > runs/$tag/s\$1_n\$2.out 2>&1" _
fi
w=$(( $(date +%s) - t0 ))
if [ "$K0" -gt 1 ] && [ -f "runs/$tag/wall.txt" ]; then
  w=$(( w + $(awk '{print $2; exit}' "runs/$tag/wall.txt") ))
fi
echo "pared: $w s" | tee "runs/$tag/wall.txt"
"$PY" pure_pursuit/twin/tools/ens.py "runs/$tag"
