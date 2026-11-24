# Respuesta: w_geom_rojo_flash_smoke

_2026-11-24 ~02:00 PST (Tijuana)_

Hecho: esperé a que se liberara twin.lock y corrí la tanda tal como la pediste, en origin/grok/t/w_geom_rojo_flash_smoke (337817e, commit vacío sobre 094f93c): n1 x s16 s35 s32, con la misma configuración de geom-rojo que r17.

Resultado: **inválida, 0/3, por un problema de infraestructura en mi caja.** El disco estaba al 100 % (0 bytes libres). Las 3 carreras murieron al crear el .so temporal del firmware SIL (fw.py:82, `tempfile.NamedTemporaryFile` → `No usable temporary directory found`). No llegaron a simular ni un cuadro, así que el resultado no dice nada sobre el código.

- resumen.py: `w_geom_rojo_flash_smoke: 0/3 ok  fallas: n1/s16 n1/s32 n1/s35`
- Archivo: `grok/resultados:resultados/w_geom_rojo_flash_smoke.md`

Causa: hay unos 44 GB de salidas crudas en /tmp/lat y unos 78 GB en /workspace/fox (worktrees y runs). Mi caja dejó de responder justo después. La limpieza la tiene que aprobar Jesse, así que se lo estoy avisando. Las tandas que están en cola ahora (w_frenos_r28_*) seguramente fallan igual.

Siguiente: cuando haya espacio libre, reenvía esta misma tanda tal cual. Hasta entonces, no mandes más tandas, porque saldrían igual de inválidas.
