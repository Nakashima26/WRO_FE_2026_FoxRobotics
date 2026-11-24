# Respuesta: w_geom_rojo_r15b

_2026-11-24 00:16 PST (Tijuana)_

Hecho: corrí la tanda en origin/grok/t/w_geom_rojo_r15b (9f0bbc1, stickyEarly solo si pka>=100) como la pediste: 8 carreras (n1-2 x s5 s32 s35 s36), TWIN_MOTOR_REAL=1, TWIN_VBAT=11.5, cámara desde calibración, TWIN_FW_OVER PARK_GEOM_EXT=1 + PARK_GEOM_ROJO=1, TWIN_PI_EXTRA ROUTE_PARK_GEOM_EXT=1, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7. Corrió sola bajo twin.lock y no faltó ninguna dependencia del venv.

- resumen.py: `w_geom_rojo_r15b: 7/8 ok  media 45.02  mediana 45.01  mejor 43.80  fallas: n2/s36`
- Reloj total: 12.8 s. La doy por **válida**: carga máx 1.75, sin OOM/Killed, media_ms (COSTO) de 9.5-10.5 ms en 6 de 8 carreras. n2/s36 da 22 ms porque se cortó a los 6.6 s, y n1/s35 da 15.4 ms.
- Tipo de prueba: carrera completa con estacionamiento (mismo tipo y misma configuración que w_geom_rojo_r14 y w_geom_rojo_r15_s5).

Comparación en las carreras que coinciden:
- **n2/s36 falla**: choca con la señal verde de S/T1 a los 6.6 s, con 1 giro. Es el mismo choque temprano que r14 tuvo en n1/s36, ahora con el otro ruido. Pasa antes del estacionamiento, así que no lo atribuyo al parche.
- s35 (n1 y n2) termina bien y baja 0.74 y 1.19 s frente a r14. n1/s36, que en r14 chocaba temprano, ahora termina bien (44.38 s).
- s5/s32 frente a r15_s5: siguen ok, pero suben n1/s5 (+1.02 s), n1/s32 (+0.51 s) y n2/s32 (+0.69 s); n2/s5 baja 0.30 s. El parche apunta a s5 (pka=30) y no se ve una mejora clara ahí. ¿Esperabas que s5 bajara de tiempo?

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/w_geom_rojo_r15b.md`.

Siguiente: si el cambio de pka>=100 es el candidato, manda la tanda completa n1-2 x s1-48 con este mismo env para compararla 1 a 1 contra r14 (89/96, media 42.28). Si quieres aislar el choque temprano de s36 (que cambia de n1 a n2), manda aparte un lote solo de s36 con más ruidos (n1-8).
