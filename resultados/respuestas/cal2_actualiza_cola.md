# Respuesta: cal2_actualiza_cola

_2026-11-25 12:21 PST (Tijuana)_

Hecho:

- **s20cal2** y **cal2_288** seguían en espera de twin.lock (nunca arrancaron, sin worktree en uso ni carreras). Las saqué de la cola: terminé solo sus procesos en espera (el script y su `flock`), sin tocar ningún proceso en ejecución. Su estado quedó como `OMITIDA` (escala de reversa común 0.82 sustituida por TWIN_REV_RADII_MM=180,162).
- No cambié ni moví ninguna otra tanda.

Estado de las cuatro válidas (comandos originales, sin cambios):

| Tanda | Estado |
|---|---|
| s20cal2_base | en cola, esperando twin.lock (worktree listo en b2779476) |
| s20cal2_radios | en cola, esperando twin.lock (23ee18d4) |
| cal2_288_base | en cola, esperando twin.lock (2238e31a) |
| cal2_288_radios | en cola, esperando twin.lock (c82a302f) |

Ahora mismo corre otra tanda (ciego25) y hay varias más en espera del mismo lock; el orden entre tandas en espera no está garantizado (flock no es FIFO). Los resultados de cada una llegarán a `grok/resultados:resultados/<tanda>.md` y su respuesta en `resultados/respuestas/<tanda>.md` cuando terminen.

Nada más que enviar por ahora.
