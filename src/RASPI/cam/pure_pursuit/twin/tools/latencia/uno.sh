#!/bin/bash
# uso: uno.sh <tag> <FF> <FD> <ruido> <semilla>   (cwd = src/RASPI/cam). Una carrera completa del twin con latencia.
D=$(cd "$(dirname "$0")" && pwd); OUT=${LAT_OUT:-runs/latencia.noindex}
tag=$1; ff=$2; fd=$3; nz=$4; s=$5; out=$OUT/$tag/n$nz/s$s
mkdir -p "$(dirname "$out")"; [ -s "$out.out" ] && grep -q '^COSTO ' "$out.out" && exit 0
PYTHONPATH=. TWIN_GEAR=1.25 FOX_PARK_POS=random TWIN_NOISE_SEED=$nz OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUTF8=1 \
TWIN_FW_PARAMS="serial2_us_per_char=21.7,serial_us_per_char=21.7" TWIN_PI_UART_US_PER_BYTE=21.7 TWIN_COST_FRAME=$ff TWIN_COST_DP=$fd \
"${FOX_PY:-python3}" "$D/costo.py" "$s" "$out" < /dev/null > "$out.out" 2>&1
