# cxv1 — estado actual de la cola cxv1_r1_seq.sh (19:15 PST, 13-nov-2026)

**Tipo de prueba:** repetición aislada cxv1_*_1113a_r1 (mismo tipo que las tandas cxv1_*_1113a originales).

## Resumen
- **No hubo deadlock.** El PID 197778 ya no existe: la cola terminó y salió sola.
- Log: `18:48:57 esperando…` → `19:09:15 locks adquiridos` (tomó twin.lock y luego /tmp/fox-grok-bancos.lock, en ese orden, sin quedarse bloqueada).
- **lock_base_r1 sí arrancó** (19:11:35) y terminó en 19:11:57.
- **Las 6 tandas son INVÁLIDAS: el disco se llenó** (`OSError: [Errno 28] No space left on device`).
  - lock_base_r1: 8/96 ok, wall 21.4 s, 85 carreras sin STOP → INVÁLIDA (ENOSPC, no es un resultado del cambio).
  - lock3_r1: 96 carreras sin STOP, wall 7.9 s → INVÁLIDA.
  - debt_base_r1, debt_r1, stale_base_r1, stale_r1: fallaron en segundos; .md/_put.json/_WALL vacíos (0 bytes) → INVÁLIDAS.
- No cambié nada en la cola (no hacía falta corregir locks). No hay ninguna cola cxv1 corriendo ahora ni bancos duplicados.

## /proc/locks (19:15)
- twin.lock (inodo 121879): **ningún holder** (no aparece en /proc/locks).
- /tmp/fox-grok-bancos.lock (inodo 7353): **ningún holder**.
- Solo hay flocks del sistema (box-bounded-log), ninguno de colas Fox.

## Disco
- `/`: 128G, 126G usados, **1.5G libres (99%)**.
- /workspace/fox: 99G; /tmp/lat: 22G (carpetas de corridas).
- Hay que liberar espacio (p. ej. carpetas viejas de /tmp/lat/* y worktrees *_wt antiguos) antes de relanzar. No borré nada: necesita visto bueno de Jesse.

## Pendiente
- Liberar disco y relanzar la secuencia completa (una sola cola, con ambos locks en el mismo orden twin.lock → bancos).
- Los _put.json de debt/stale en pend/ están vacíos (0 bytes) y no deben publicarse como resultados.
