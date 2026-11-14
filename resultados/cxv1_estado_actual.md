# cxv1_estado_actual — estado (solo lectura, sin lanzar nada)

Hora: 2026-11-13 19:04 PST. Tipo: carrera completa desde el cajón en el banco con latencia (uno.sh/costo.py, FF 2.8 / FD 3.8, primer plan 1.95 s, VBAT 12.5, 96 carreras, P=7). **Sin resultados ni comparación de tiempos** (no ha corrido ninguna carrera cxv1).

## Cinco originales cxv1_*_1113a (sin r1)

| tanda | PID run.sh | estado ahora | último log (mtime) |
|---|---|---|---|
| cxv1_lock3_1113a | 184286 | **no existe** (cancelada 18:51 PST, SIGTERM) | 18:40 PST |
| cxv1_debt_base_1113a | 160738 | **no existe** (cancelada 18:51) | 18:25 |
| cxv1_debt_1113a | 163723 | **no existe** (cancelada 18:51) | 18:25 |
| cxv1_stale_base_1113a | 164896 | **no existe** (cancelada 18:51) | 18:26 |
| cxv1_stale_1113a | 164502 | **no existe** (cancelada 18:51) | 18:25 |

Cancelación solicitada: **hecha y efectiva**. Las cinco tenían 10–26 min de vida al cancelar, estaban en su bucle de espera y quedaron en 0/96 (inválidas, sin resultados parciales). Hoy no queda ningún proceso `cxv1_*_1113a_run.sh` vivo. (El informe de cancelación cxv1_estado_cancelacion.md sigue pendiente de push en pend/.)

## Única cola r1

- Script: `/workspace/fox/cxv1_r1_seq.sh`, PID **197778** (bash), vida ~14–15 min (inicio 18:48:57 PST). Log: `/workspace/fox/cxv1_r1_seq.log` (única línea: "esperando a que terminen las tandas originales").
- Hijo **203667** `flock 8` (vida ~11–12 min): ya pasó la espera de originales; **está bloqueado esperando /workspace/fox/twin.lock**.
- Orden de espera en twin.lock (/proc/locks): titular **197028** `flock twin.lock ./v1kB_run.sh` (~15 min) → en cola **197584** `./v1k0_run.sh` → en cola **203667** (cxv1 r1).
- v1kB tiene el lock pero aún no corre: su script espera a que no haya ningún `latencia/uno.sh`. El banco que sí corre ahora es **cxpark_b1b_1113** (run.sh PID 171724, ~34 min, lanzado fuera de twin.lock; xargs -P 7 arrancó ~19:03 PST, 7 uno.sh / 8 costo.py activos).
- Después del lock, la cola r1 aún tomará /tmp/fox-grok-bancos.lock (hoy no existe, libre) y esperará carga<10 sin otros uno.sh/costo.py antes de cada una de sus 6 tandas (lock_base, lock3, debt_base, debt, stale_base, stale). Ninguna tanda r1 ha empezado (sin _ENV/_WALL).

## Caja

- Load: 8.4 / 20.4 / 39.9 (1/5/15 min, 8 CPU) — bajando, pero sube de nuevo con cxpark_b1b_1113.
- Memoria: 16002 MB total, ~4.6 GB usados, ~11.4 GB disponibles, sin swap.

## Resumen de espera

cxv1 r1 → **no espera originales** (ya no existen) → **espera twin.lock** (detrás de v1kB y v1k0) → v1kB espera a que termine el banco ajeno cxpark_b1b_1113. No se lanzó nada, no se tocaron bancos ajenos.
