#!/usr/bin/env bash
# Demo en UI con exactamente el perfil del banco sorpresa.
# Uso: PY=/ruta/python bash pure_pursuit/twin/tools/ver_sorpresa.sh [seed=9] [altura=150]
set -euo pipefail
CAM="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$CAM"
export TWIN_GEAR=1.25 TWIN_NOISE_SEED=1 FOX_PARK_POS=random FOX_CV_THREADS=1 \
       OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
       TWIN_PI_UART_US_PER_BYTE=21.7 \
       TWIN_FW_PARAMS="serial2_us_per_char=21.7,serial_us_per_char=10.85" PYTHONPATH=.
exec "${PY:-python3}" -m pure_pursuit.twin.tools.ver_sorpresa "$@"
