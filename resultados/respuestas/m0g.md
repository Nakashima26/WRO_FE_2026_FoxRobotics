# Respuesta Grok — m0g

**Qué se hizo**
- Worktree nuevo `/workspace/fox/m0g_wt`: `git fetch origin && git checkout --detach origin/grok/t/m0g` → commit e46552b (commit vacío encima de 6be7aca).
- Tipo de prueba: **carrera completa desde el cajón** (`costo.py` usa `--solo-cajon`: salida del estacionamiento, 3 vueltas y estacionamiento, `FOX_PARK_POS=random`) en el **banco NUEVO con latencia** (`latencia/uno.sh` + `costo.py`, FF 2.8 / FD 3.8, primer plan 1.95 s). `TWIN_PI_EXTRA='{}'`, así que el plan es síncrono (`ROUTE_PLAN_ASYNC=false` en carrera_bench.json); sin `TWIN_VBAT` ni `TWIN_FW_OVER`. Ruido 1-2 × semillas 1-48 = 96 carreras, `xargs -P 7`.
- Toda la tanda corrió bajo `flock /workspace/fox/twin.lock` (y `/tmp/fox-grok-bancos.lock`), sin otros bancos activos (0 procesos uno.sh/costo.py ajenos al inicio, carga inicial ~0.9).

**Resultado**
- `m0g: 85/96 ok  media 34.01  mediana 33.60  mejor 29.50  fallas: n1/s11 n1/s23 n1/s8 n2/s14 n2/s15 n2/s26 n2/s34 n2/s4 n2/s43 n2/s6 n2/s8`
- Reloj total: **107.8 s**. Las 96 carreras tienen STOP y COSTO; media de media_ms del COSTO: 9.2 ms (tandas anteriores ~8.8-9.0). Carga máxima 6.9 y sin OOM/MemoryError.
- Validez: **VÁLIDA**.

**Problemas (se resolvieron antes de la tanda válida)**
- Se había reiniciado la caja: faltaba el venv compartido `/workspace/fox/perfil-isla/.venv`, el destino del symlink de todos los worktrees. Se recreó con `src/RASPI/cam/requirements-sim.txt` (numpy, opencv-python-headless, matplotlib, imageio, pytest) + `ziglang`.
- Al principio no había compilador C++ para el firmware SIL y las 96 carreras fallaron en 5 s. Se probó g++ del sistema y no compila los shims `sil/Arduino.h` (`std::arduino_min`), que son solo para clang. Se desinstaló y quedó el camino de siempre: `ziglang` en el venv (clang). Esos dos intentos fallidos se descartaron (guardados en `/workspace/fox/m0g_intento1/` y `m0g_intento2/`). En la tanda válida el firmware se precompiló con una carrera de calentamiento fuera del cronómetro.
- Comparar solo contra tandas del mismo tipo: carrera completa desde el cajón, banco con latencia, plan síncrono y sin VBAT. **No** es comparable con v1q0/v1q20/v1q30 (VBAT 12.5 + ROUTE_PLAN_ASYNC) ni con las de estacionamiento con TWIN_FW_OVER.

**Qué mandar después**
- Si m0g es la nueva referencia: un candidato con los mismos parámetros del banco (`TWIN_PI_EXTRA='{}'`, sin VBAT) para comparar contra 85/96, media 34.01. O repetir m0g una vez para ver la varianza entre corridas.
- Para revisar las fallas, empezar por n2 (8 de las 11 fallas).
