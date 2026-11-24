@echo off
rem Dashboard del carro + flasheo de la ESP32 por USB desde esta compu.
cd /d "%~dp0.."
python scripts\fox_pc.py %*
pause
