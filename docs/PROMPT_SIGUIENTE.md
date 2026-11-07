Repo `C:\Users\jbanda\Documents\GitHub\WRO_FE_2026_FoxRobotics`, digital twin del robot Fox (WRO FE 2026).
Trabaja desde la rama `t15b2` (worktree `.claude/worktrees/t15b2`, HEAD 0e3f6f1 o posterior).

Lee primero `docs/HANDOFF.md` completo (estado, reglas duras, Mac). Lee `docs/twin_plan.md` solo por secciones
con grep/tail (es largo). En cada comando bash antepone
`export PATH="/c/Users/jbanda/AppData/Local/Programs/Git/usr/bin:/c/Users/jbanda/AppData/Local/Programs/Git/mingw64/bin:/c/Users/jbanda/AppData/Local/Programs/Git/cmd:$PATH";`

## Tu rol
Eres el ORQUESTADOR. No editas firmware ni corres lotes tú mismo; delegas en subagentes (Sonnet para trabajo con
juicio, Haiku solo para tareas mecánicas) y verificas en la fuente lo que reportan antes de aceptarlo o contármelo.

**Fase 0 — planeación (antes de delegar nada):**
1. Decide qué hacer con el trabajo sin commitear que está auditado en HANDOFF (por defecto: descartar; los
   comentarios de diagnóstico se pueden conservar). Muéstrame la decisión antes de borrar nada.
2. Aclara el ruido del twin, porque bloquea a los tres frentes. Puede ser una tarea corta previa o la
   primera tarea del frente C. Mide: misma seed ×3 en serie, ×3 en paralelo y en Mac vs Windows. Busca qué
   depende del reloj real o de la carga (`pi_frame_ms`, `FOX_CV_THREADS`, `time.perf_counter` dentro de la
   lógica). Si se puede hacer determinista sin cambiar el comportamiento, mejor.
3. Arma un baseline común en la Mac: 21 seeds (1-20 + 3170839) con `SOLO_CAJON=1`, más prepark 100, desde el
   mismo HEAD. Los tres frentes comparan contra él.
4. Para cada frente escribe la hipótesis inicial, qué archivos o zonas puede tocar (para que los merges sean
   limpios), el criterio de éxito y el presupuesto de iteraciones. Muéstrame el plan en pocas líneas y arranca.

**Fase 1 — tres subagentes en paralelo**, cada uno en su propio worktree creado a mano desde t15b2
(`git worktree add .claude/worktrees/<x> -b <x> t15b2`; no uses `isolation: worktree`, que parte de `main`):

- **A) PARALELO 100/100.** Llevar `PARK_PARALELO` a 100/100 en `prepark.py --solo-cajon` (hoy 98: fallan
  `CW_c2_p4` y `CW_c3_p4`) y que estacionen limpio todas las seeds de carrera que llegan a tc=12 (hoy
  6/8/18/3170839 sí; cajón en 2, 7, 14, 19, 20). Ver los patrones 2 y 3 en HANDOFF. La U se queda dentro de la
  esquina 13 (regla 9.23, decisión mía). Solo zona de estacionamiento del .ino.
- **B) PARK_PARALELO_REV (estacionar en reversa).** Hoy 0/100 limpios: el costado roza el poste a
  Ang≈150-165° del swing C con el servo fijo (160/30, no se cambia). Buscar una secuencia de maniobra viable
  con `runs/geo4.py` / `geo2.py` (planeador geométrico) antes de tocar firmware. Fases 30-38 del .ino.
  Si no hay solución geométrica con este radio, que lo demuestre con números y pare.
- **C) Todas las seeds a 12 esquinas.** Revisar por qué las 21 seeds se comportan raro y lograr que todas
  completen las 12 esquinas, SIN clavarse en el parking (eso es de A y B). Hoy 12/21 chocan con señal: 7 en
  SIGUIENDO con esquive activo pero sin margen, siempre la primera señal. Hay 23 giros con REVERSA, con
  `distExt` 36-81 cm: averiguar por qué llega tan separado de la pared exterior, no subir `HUG_CM`.
  Considera la rama WIP `t15c-esquinas`. Zonas: esquive, giros, Pi (`runtime_nuevo.py`, `digital_map.py`),
  no parking.

Cada prompt de subagente debe incluir:
- Las reglas duras de HANDOFF: causa raíz con trace.csv y líneas de log antes de editar; seeds que cambian
  ×3; métricas por esquina; no tocar zonas ajenas.
- El bloque Mac de HANDOFF, con el permiso de ssh dado explícitamente (yo lo autorizo para lotes).
- Usar ssh sin bucles de `sleep`.
- Commit con entrada en twin_plan.md, sin push. Nunca matar python.exe ajenos.

**Fase 2 — por cada fix que un frente quiera commitear:** auditoría por otro subagente en contexto fresco,
solo lectura, que revise si el razonamiento tiene evidencia o es tanteo y si el resultado supera al baseline
fuera del ruido. Solo entonces merge a t15b2 y validación conjunta (prepark 100 + 21 seeds).

Reglas de trabajo:
- Español, términos técnicos en inglés. Directo, sin rodeos.
- Tablas solo para datos cortos; cita la línea de log decisiva.
- Si un dato verificado contradice algo que dijiste, gana el dato y corrígete explícitamente.
- Avísame de cada hallazgo o cambio de plan; no esperes al final.
- Yo hago push desde GitHub Desktop.

> 2026-10-10: leer primero docs/HANDOFF.md (Estado 2026-10-10) y docs/PLAN_v2.md; el estado de 2026-10-05 de abajo es histórico.

