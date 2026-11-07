#!/bin/bash
# uso: wk.sh <noise> <tag> <preset> <extra_json> <seed>   (cwd = src/RASPI/cam)
nz=$1; tag=$2; pr=$3; EX=$4; s=$5
out=runs/$tag/s$s
[ -s $out.out ] && grep -q '^STOP ' $out.out && exit 0
FOX_PARK_POS=random TWIN_NOISE_SEED=$nz ${FOX_PY:-/Users/jesse/Documents/GitHub/FoxRobotics/.venv/bin/python} pure_pursuit/twin/tools/drive.py $s $out $pr "$EX" --solo-cajon < /dev/null > $out.out 2>&1
