# cxv23 — cancelación de bancos originales (inválidos por saturación)

Tipo de prueba: no aplica (operación de limpieza, no es un banco). **Los bancos cancelados no son evidencia de rendimiento.**
Fecha: 2026-11-13 ~19:05 (hora del Pacífico, UTC-8). Pedido por codex-v23.

Método: se identificó PID/PPID/comando/cwd de cada proceso; solo se envió TERM a PIDs concretos cuyo comando contenía el tag exacto. Sin pkill/killall. No se tocaron cxv23_r*, rv23/chain.sh (PID 197309 sigue vivo) ni procesos de otros workers (p. ej. cxpark_b1b_1113). Logs/resultados existentes conservados; nada relanzado.

| tag | estado | motivo / detalle |
|---|---|---|
| cxv23_b01 | ya terminado | sin procesos activos (run.log RC=0) |
| cxv23_b02 | **cancelado** | PID 199125 (`cxv23_b02_post.sh`) y 199126 (`flock twin.lock ... cxv23_b02_run.sh`), en espera del lock para re-ejecutar; no había corrida en curso |
| cxv23_b03 | **cancelado** | PID 209320 (`cxv23_b03_wait.sh`) + hijo `sleep` 221068 (terminó solo); bucle de espera, no llegó a correr |
| cxv23_b04 | ya terminado | sin procesos activos (RC=0) |
| cxv23_b05 | **cancelado** | PID 217243 (`flock twin.lock ... cxv23_b05_run.sh`), en espera del lock para re-ejecutar; no había corrida en curso |
| cxv23_g14a | ya terminado | sin procesos activos |
| cxv23_g14b | ya terminado | sin procesos activos |
| cxv23_g14c | ya terminado | sin procesos activos |
| cxv23_los1 | ya terminado | sin procesos activos |
| cxv23_los2 | ya terminado | sin procesos activos |

Verificación posterior: no queda ningún proceso con cxv23_b0*/g14*/los* en su comando.
Nota: los procesos cancelados de b02/b05 no habían arrancado su run.sh, así que no sobrescribieron los run.log anteriores.

