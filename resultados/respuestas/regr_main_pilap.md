# Respuesta: regr_main_pilap

_2026-11-25 22:50 PST (Tijuana)_

Recibida la tanda regr_main_pilap (origin/grok/t/regr_main_pilap = 09c29176, mismo código que main 52f42d03; commit vacío). Worktree listo en /workspace/fox/regr_main_pilap_wt y script lanzado: está en cola esperando twin.lock (hay ~12 tandas delante: vis2_*, l1s_post_v3, regr_*). Corre n1-8 x s1-48 (384 carreras), -P 7, con tu comando exacto (FW MOTOR_CURVE_LIVE=false; PI: ROUTE_MOTOR_CURVE_LIVE=false, L0_CTE_PWM=null, L1_COALESCE_UPLOAD=false, L23_CLEAR_CURVE_LEAD_REDUCTION_S=0.0, L23_SIGN_MM_L3=null).

Tipo de prueba: carrera completa (latencia/uno.sh, FF 4.28 / FD 5.8). Cuando termine subo resultados/regr_main_pilap.md y agrego aquí el resultado con comparación contra vis2_base y las regr_* hermanas.

## Actualización 2026-11-25 23:58 PST (Tijuana)


Hecho: corrí la tanda en origin/grok/t/regr_main_pilap (09c29176) con el comando que mandaste: n1-8 x s1-48 (384 carreras), -P 7, bajo twin.lock.

- resumen.py: `regr_main_pilap: 370/384 ok  media 37.23  mediana 36.85  mejor 32.63  fallas: n1/s26 n2/s48 n3/s27 n3/s3 n3/s44 n4/s20 n4/s36 n5/s16 n5/s3 n5/s36 n6/s10 n6/s23 `
- Paradas: collision 6, race_finished 375, stuck 3
- Reloj total: 576.2 s. INVÁLIDA (revisar carga/OOM): corrió bajo twin.lock (espera de lock 2197 s, no incluida en el reloj; 0 procesos de banco ajenos al inicio), 576.2 s para 384 carreras (referencia ~400-440 s sin carga, proporcional a ~100-110 s por 96), carga máx 8.35, mem máx 4548 MB, OOM/Killed/MemoryError: 0, carreras sin STOP: 0, mean_pi_frame_ms 3.95-7.44 (media 4.65).
- Tipo de prueba: carrera completa, regr_main_pilap (main 52f42d03; FW MOTOR_CURVE_LIVE=false; PI: ROUTE_MOTOR_CURVE_LIVE=false, L0_CTE_PWM=null, L1_COALESCE_UPLOAD=false, L23_CLEAR_CURVE_LEAD_REDUCTION_S=0.0, L23_SIGN_MM_L3=null), 384 carreras (n1-8 x s1-48); solo comparable con tandas de ese mismo tipo y mismo set de semillas.

- Comparación (mismo tipo: carrera completa FF 4.28/FD 5.8, mismas semillas n1-8 x s1-48; referencia vis2_base; regr_* hermanas si ya estaban publicadas):
  - vs vis2_base: 378/384 ok, media 37.21, mediana 36.73, mejor 32.63 -> regr_main_pilap 370/384 ok, media 37.23 (+0.02), mediana 36.85 (+0.12), mejor 32.63; fallas 14 vs 6; fallas en ambas: n3/s27 n4/s20 n4/s36 n5/s3 n5/s36

Qué mandar después: si aún no está, la tanda regr_main (main 52f42d03 con los overrides PI de vuelta activos, mismo comando y semillas) para aislar el efecto de L0/L1/L23; luego activar uno a uno (L0, L1, L23) para ver cuál explica la diferencia.

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/regr_main_pilap.md`.
