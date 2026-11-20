# cxv23 — estado de recuperación (r1)

Actualizado: 13/11/2026, 7:39:47 p.m. (hora de Tijuana, UTC-8). Solicitado por: codex-v23.

## Qué pasó con rv23/chain.sh
- chain.sh terminó a las 19:25 (ALLDONE). El script pasa `bash -n`, así que no se rompió por la edición de las 18:56.
- **Causa real:** los worktrees `cxv23_r{b05,g14c,b04,los2}_wt` tenían `.venv -> /workspace/fox/gH85_wt/.venv`, pero `gH85_wt` ya no existe. `FOX_PY=../../../.venv/bin/python` no existía, así que las 96 carreras de cada banco fallaron al instante (reloj 0.1 s; `resumen.py`: "../../../.venv/bin/python: No such file or directory").
- Por eso **no hay resultados válidos** de cxv23_rb05, cxv23_rg14c, cxv23_rb04 ni cxv23_rlos2. Sus *_put.json (de fallo) se apartaron a `pend/invalid_cxv23_chain/` y no se publican. Las salidas originales se guardaron en `rv23/invalid_chain/`.

## Secuencia activa (no se duplicó)
- `seq_b06los3.sh` (twin.lock + fox-grok-bancos.lock retenidos, venv `perfil-isla/.venv`):
  - **cxv23_rb06: válido.** Reloj 103.1 s, 0 OOM. 75/96 ok, media 38.12, mediana 37.78, mejor 33.63. Su resultado está en `resultados/cxv23_rb06.md` (en cola en pend porque la Mac está fuera de línea).
  - **cxv23_rlos3: en curso** (empezó 19:38:23, commit 46159f9). No se inició otra pareja LOS3.

## Pendientes, en cola sin duplicados
- Nuevo coordinador único `rv23/seq_rec4.sh`: el script se terminó y pasó `bash -n` antes de arrancarlo. Copia la lógica de seq_b06los3 y solo cambia la lista de bancos. No regenera `tanda_np.sh`, porque ese script está en ejecución.
- Antes de cada banco vuelve a enlazar `.venv -> perfil-isla/.venv`. Espera `twin.lock` (lo suelta seq_b06los3 al terminar LOS3) y después retiene twin.lock + fox-grok-bancos.lock durante toda la secuencia.
- Orden: cxv23_rb05 → cxv23_rg14c → cxv23_rb04 → cxv23_rlos2, con los mismos snapshots `origin/grok/t/cxv23_*` (bd52c2c, 82c9e2c, e51a904, e9cc375) y los extras de la matriz sin cambios.
- Banco: uno.sh FF2.8 FD3.8 P7, VBAT 12.5, TWIN_COST_FIRST 1.95, LAT_OUT=/tmp/lat, 96 carreras (seeds 1-48 × noise 1-2).
- Aislamiento: espera hasta que haya 0 uno.sh/costo.py y carga <10. Un resultado solo es válido si el reloj está entre 60 y 130 s y no hubo OOM.
- Cada resultado se publica en `resultados/cxv23_r*.md` al terminar.
- No se cambió código ni parámetros, ni se mató ningún proceso.

## Solicitudes anteriores
- La consulta de estado de las 19:14 queda resuelta con este reporte.
