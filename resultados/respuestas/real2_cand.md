# Respuesta: real2_cand

_2026-11-25 23:20 PST (Tijuana)_

Hecho: el box se reinició a las ~23:03 PST con la tanda en cola (se perdieron el .venv y toda la cola de twin.lock, incluida real2_base). Recreé el venv y relancé solo real2_cand a las 23:08 PST; terminó a las 23:18 PST. Corrí la tanda en origin/grok/t/real2_cand (c02d4566) con el comando que mandaste (TWIN_REAL2=1 + resto de env igual que real2_base): n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `real2_cand: 255/384 ok  media 39.24  mediana 38.91  mejor 32.92  fallas: n1/s11 n1/s13 n1/s19 n1/s21 n1/s29 n1/s33 n1/s34 n1/s35 n1/s38 n1/s42 n1/s45 n1/s47 n1/`
- Paradas: collision 4, race_finished 378, stuck 2. De las 129 fallas, 123 son race_finished sin choques pero con el robot fuera del cajón de estacionamiento >5 mm (mediana 10.0 mm, rango 5.03-374 mm); las otras 6 son 4 collision + 2 stuck.
- Reloj total: 620.9 s. INVÁLIDA por reloj (sin OOM y carga ok, pero el reloj quedó muy por encima de la referencia): corrió bajo twin.lock (espera de lock 0 s, no incluida en el reloj; 0 procesos de banco ajenos al inicio), 620.9 s para 384 carreras (referencia ~400-450 s, ~100-110 s por 96; la última válida del mismo tipo, vis2_base, tardó 551.6 s con mean_pi_frame_ms media 4.45), carga máx 8.40, mem máx 4638 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 4.15-7.90 (media 4.79) (+~8% vs vis2_base). Causa probable: el box se reinició ~23:03 PST (se perdió el .venv y murió la cola de twin.lock); el venv se recreó con numpy 2.5.5, opencv-python-headless 5.1.0 y ziglang (compilador C++), así que el entorno no es idéntico al de tandas anteriores. Los tiempos de carrera (total_time_s) son tiempo simulado con COST_FIJO=1, pero trátalo como no comparable en rendimiento del banco.
- Tipo de prueba: carrera completa FF 4.28/FD 5.8, banco nuevo con latencia, 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y semillas.

- Comparación (mismo tipo: carrera completa FF 4.28/FD 5.8, mismas semillas n1-8 x s1-48; referencia directa real2_base: mismo código sin TWIN_REAL2; vis2_base/regr_main como contexto):
  - vs vis2_base: 378/384 ok, media 37.21, mediana 36.73, mejor 32.63 -> real2_cand 255/384 ok, media 39.24 (+2.03), mediana 38.91 (+2.18), mejor 32.92; fallas 129 vs 6; fallas en ambas: n3/s27 n4/s36

Nota: real2_cand tiene el mismo árbol que real2_base (el commit c02d4566 es vacío sobre f8b9101d), así que la diferencia contra real2_base es solo el efecto de TWIN_REAL2=1 en el gemelo, no un cambio de código del robot. Contra vis2_base/regr_main se mezcla cambio de gemelo con cambio de código.

Ojo: en este árbol TWIN_REAL2=1 activa la reversa lenta medida (params.py, igual que TWIN_REV_CURVA_REAL) y el scrub de INICIO 0.8 (fw.py), pero NO activa TWIN_FIELD_JSON (acomodo de corrida real); esa env no se pasó. Preguntas: ¿quieres una tanda extra con TWIN_FIELD_JSON también? ¿Relanzo real2_base (murió en el reinicio y no se ha publicado) para tener la comparación directa?

Lectura: con TWIN_REAL2=1 el problema es casi todo precisión de estacionamiento (reversa más lenta + scrub INICIO); las vueltas también salen ~2 s más lentas que vis2_base. Qué mandar después: real2_base relanzada como base directa; si TWIN_REAL2=1 queda como gemelo de referencia, las variantes de código contra real2_cand con este mismo comando y semillas.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/real2_cand.md`.
