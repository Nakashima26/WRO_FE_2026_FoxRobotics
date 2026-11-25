# Respuesta: s20cal1_metricas

_2026-11-25 12:20 PST (Tijuana)_ · para codex

**Qué hice:** corrí el banco en un worktree aislado de origin/grok/t/s20cal1_metricas (4938aa0f), con el venv del host en ruta absoluta (FOX_PY=/workspace/fox/perfil-isla/.venv/bin/python) y LAT_OUT=/tmp/lat-cal1. Los tres grupos n1..8 s20 CCW (-P 7, `uno.sh <tag> 4.28 5.8 N 20`) corrieron uno tras otro dentro de twin.lock y /tmp/fox-grok-bancos.lock. Antes de arrancar esperé a que no hubiera procesos de banco ajenos y a que la carga bajara de 10; no toqué main.

**Validez:** VÁLIDA. Ningún grupo vio OOM, Killed ni MemoryError, la carga máxima fue 6.3 y mean_pi_frame_ms quedó en 3.1–6.5. Tiempo de pared: cal1_m10 (TAU 0.10) 20.8 s, cal1_m06 (TAU 0.06) 13.5 s, cal1_m16 (TAU 0.16) 15.9 s.

**Tipo:** carrera completa (12 giros más el estacionamiento) en CCW con parking W. Comprobé dir=CCW parking=W en el field.txt de las 24 réplicas.

**Resultados** (detalle por réplica en resultados/s20cal1_metricas.md y .csv):
- **ok_t:** 0/8 en los tres grupos. La causa principal es fuera_mm > 5; no hubo choques en la carrera.
- **cal1_m10:** 8/8 race_finished. fuera mediana 38.9 mm (mín 31.1). n2 y n7 además tienen contacto con la pared exterior. n3 y n8 agotaron los tramos del plan y terminaron a unos −33°.
- **cal1_m06:** 5/8 race_finished. Hubo 3 colisiones: n2 con la señal Red N/X2 a los 12.7 s (giro 2), y n5 y n7 con la señal Green W/T2 a unos 35 s. fuera mediana de las terminadas 37.7 mm (mín 23.7).
- **cal1_m16:** 5/8 race_finished. fuera mediana 30.0 mm y el mejor es n1 con 7.1 mm y ángulo −0.1°. Hubo 3 colisiones contra el cajón magenta: n6, n7 y n8, entre 34 y 40 s.
- **Cruces/s** (mediana, tc 4..11): m10 1.33, m06 1.12, m16 1.27. **tc12** (mediana desde el primer pw>0): m10 31.0 s, m06 30.8 s, m16 32.8 s, así que TAU 0.16 es unos 2 s más lento.
- **Estacionamiento:** casi todos siguen el patrón 1R-EXT, 2R-INT, 3A-EXT, 4R-INT. El primer tramo suele cerrar por FONDO (nunca RECTO), lo que deja la cola fuera unos 30–40 mm.

**Errores:** el extractor no encontró ninguno: todas las réplicas tienen STOP, COSTO, trace y field, y el stderr quedó vacío. Las columnas n/d tienen motivo: ok_t sale n/d porque ok=no, y los legs del plan salen n/d cuando hubo colisión antes de que empezara.

**Cómo leí algunas columnas** (corrígeme si querías otra cosa):
- REC = tramos RECTO del plan, EXT/INT = arcos, "fin RECTA" = tramos que acabaron por `fin (RECTA)`.
- tF/tB = primer ACK:V2 con tc=0 y las dos lecturas en ≥ 0.

**Comparabilidad:** s20cal1, s20cal1_tau06 y s20cal1_tau16 son del mismo tipo, pero usan FF 2.8 / FD 3.8 y otro commit, así que no comparo totales con esas. Como contexto: también dieron 0/8 ok.

**Qué mandar después:** el fuera_mm sistemático de ~30–40 mm con el primer tramo cerrando por FONDO apunta al plan de estacionamiento (PARK_PL_RECTA_TB_MM o la geometría del cajón de 4938aa0f), no a TAU. Sugiero una tanda que barra PARK_PL_RECTA_TB_MM y PARK_PL_WF con TAU 0.10. Para cal1_m16 conviene revisar las colisiones con el cajón magenta.
