# Respuesta Grok — pfx100g

**Qué se hizo**
- Worktree nuevo `/workspace/fox/pfx100g_wt`: `git fetch origin && git checkout --detach origin/grok/t/pfx100g` → commit 292989b (parche de prueba encima de 664e69c, ROUTE_PREFIX_PWM 100).
- Tipo de prueba: **carrera completa desde el cajón** (`--solo-cajon`, FOX_PARK_POS=random) en el **banco NUEVO con latencia** (uno.sh + costo.py, FF 2.8 / FD 3.8, primer plan 1.95 s, `TWIN_PI_EXTRA='{}'` → plan síncrono, sin VBAT ni FW_OVER). Ruido 1-2 × semillas 1-48 = 96 carreras, `xargs -P 7`. Mismo banco que m0g.
- Todo bajo `flock /workspace/fox/twin.lock`, sin otros bancos activos; firmware SIL precompilado con una carrera de calentamiento fuera del cronómetro. No hizo falta instalar dependencias.

**Resultado**
- `pfx100g: 86/96 ok  media 34.37  mediana 34.25  mejor 30.42  fallas: n1/s11 n1/s28 n1/s32 n1/s35 n1/s39 n1/s46 n1/s8 n2/s37 n2/s43 n2/s8`
- Reloj total: **118.3 s**. 96/96 con STOP y COSTO; media de media_ms: 10.2 ms. Sin OOM, carga máx 6.25.
- Validez: **VÁLIDA**, con una reserva: reloj y costo un poco más altos que m0g (107.8 s / 9.2 ms) con el mismo banco. Puede ser actividad de fondo del host de la caja.
- Fallas por causa (n1 s11 y n2 s43 terminan con race_finished, pero resumen.py las cuenta como falla):
  - collision (cajón magenta): n1 s8, n1 s28, n1 s39, n1 s46, n2 s8, n2 s37
  - collision (isla): n1 s32
  - collision (señal Red E/X2): n1 s35
- Comparado con m0g (mismo tipo y mismo banco): 86/96 vs 85/96 ok, media 34.37 vs 34.01, mediana 34.25 vs 33.60. Una tanda sola no separa esto del ruido. n1/s8, n1/s11 y n2/s43 fallan en las dos.

**Qué mandar después**
- Para confirmar el +1 ok, repetir pfx100g y m0g (o hacer varias repeticiones) y comparar promedios.
- Archivo completo: `resultados/pfx100g.md` en grok/resultados.
