# geo_scripts — búsqueda de scripts del planeador geométrico de estacionamiento

Solicitado por: claude-code (webhook 2026-11-27 23:37 PST, UTC-8)
Tipo: búsqueda de archivos (sin correr nada). No es un batch del twin.

## Resultado: NO se encontró ninguno

No existe ningún `geo.py`, `geo2.py`, `geo3.py`, `geo4.py`, `geo5.py` ni `geo5b..geo5g.py` en ninguna de las ubicaciones a las que tengo acceso. Por eso `resultados/geo_scripts/` no se creó (no hay nada que copiar).

## Rutas / fuentes revisadas

1. **Disco completo de mi computadora (box Linux)**: `find / -name 'geo*.py'` (raíz) + `/workspace`, `/tmp`, `/home` (disco aparte). Solo aparecieron librerías de terceros (fontTools, openpyxl) y:
   - `/workspace/fox/runs/sweep/segtrace/geom.py`, `geom_agg.py`, `geom_agg2.py` (2026-11-25 23:04). **No son del estacionamiento**: recalculan longitud de camino libre en trazas de la pista (hybrid-A* con R_min = 113/tan(44.54°)). No se copiaron.
2. **Todos los worktrees bajo /workspace/fox** (`*_wt`, `wt-senal`, `am2*`, `vel`, `opt`, etc.) incluyendo carpetas `runs/` ignoradas por git.
3. **Historial git de TODAS las ramas** de Nakashima26/WRO_FE_2026_FoxRobotics (`git log --all --name-only` tras `git fetch`): ningún archivo `geo*.py` en ningún commit.
4. **Rama `t15b2`** y **`t15b-merge`**: sin geo*.py. Lo relacionado con estacionamiento que sí tienen: `src/RASPI/cam/pure_pursuit/salida_cajon.py`, `twin/prepark.py`, `twin/tools/prepark.py`, `twin/tools/parkcheck.py`, `twin/tools/park_viz.py` (solo t15b2), `twin/tests/test_salida_cajon.py`.
5. **Rama `t15rev`**: **no existe** en origin.
6. Búsqueda por contenido de .py con `109` y `270` en /workspace y /tmp: solo coincide `pure_pursuit/config.py` de cada worktree (configuración, no simulador de arcos).

## No accesible

- **`C:\001Escul`** y cualquier disco de Windows: no tengo acceso a ninguna PC Windows (solo a mi box Linux y a la MacBook de Jesse, que ahora está **offline**). Si los scripts están en esa PC, es casi seguro que solo existen ahí (nunca se commitearon).
