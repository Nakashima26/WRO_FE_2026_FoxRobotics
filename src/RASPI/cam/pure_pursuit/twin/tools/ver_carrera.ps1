# Windows: igual que ver_carrera.sh. Abre el twin (ventana "Fox") con la carrera exportada:
# 3 vueltas + U + paralelo, engrane 1.25, 564 g. espacio=iniciar, n=otra semilla, r=reinicio, esc=salir.
# Uso (PowerShell, desde cualquier carpeta):  .\src\RASPI\cam\pure_pursuit\twin\tools\ver_carrera.ps1 [semilla]
# Requiere .venv-sim en la raíz del repo con opencv-python (NO headless: sin GUI no abre ventana) y ziglang:
#   py -3.12 -m venv .venv-sim; .venv-sim\Scripts\python -m pip install numpy opencv-python matplotlib imageio pytest ziglang
param([int]$Semilla = 3)
$ErrorActionPreference = "Stop"
$cam  = (Resolve-Path "$PSScriptRoot\..\..\..").Path
$repo = (Resolve-Path "$cam\..\..\..").Path
$py   = if ($env:PY) { $env:PY } else { "$repo\.venv-sim\Scripts\python.exe" }
Set-Location $cam
$env:PYTHONUTF8 = "1"; $env:PYTHONPATH = "."
$env:TWIN_MOTOR = "fisico"; $env:TWIN_GEAR = "1.25"; $env:TWIN_MASS_KG = "0.564"
$env:TWIN_PI_UART_US_PER_BYTE = "21.7"; $env:TWIN_FW_PARAMS = "serial2_us_per_char=21.7,serial_us_per_char=21.7"
$env:FOX_PARK_POS = "random"
# build.py elegiría el g++ de MinGW (usa flags de clang): forzar zig.
if (-not $env:FOX_CXX) { $env:FOX_CXX = "$py -m ziglang c++" }
# Mismos argumentos que ver_carrera.sh: FW_PARK_CARRERA y luego el JSON del banco, uno por línea.
$code = @'
import json, sys
from pure_pursuit.twin.tools.exportar_carrera import FW_PARK_CARRERA
d = json.load(open(sys.argv[1], encoding="utf-8"))
for k, v in {**FW_PARK_CARRERA, **(d.get("fw") or {})}.items(): print("--fw"); print(f"{k}={v}")
for k, v in (d.get("pi") or {}).items(): print("--pi"); print(f"{k}={v!r}")
'@
$extra = & $py -c $code "pure_pursuit/twin/tools/carrera_bench.json"
& $py -m pure_pursuit.chassis_twin --seed $Semilla --preset ruta_perfil --speed 1.5 @extra
