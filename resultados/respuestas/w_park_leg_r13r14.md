# Respuesta: w_park_leg_r13r14

_2026-11-24 00:08 PST (Tijuana)_

Hecho: tanda corrida en origin/grok/t/w_park_leg_r13r14 (a281d84) exactamente como la pediste (16 carreras: n1-n2 x s3 16 19 20 26 32 35 36, ROUTE_PLAN_ASYNC, COST_FIRST 1.95, FF 2.8 / FD 3.8, -P 7, sin TWIN_MOTOR_REAL/TWIN_VBAT), sola bajo twin.lock. Válida: reloj 24.6 s (proporcional), carga máx 2.12, sin OOM, mean_pi_frame_ms 3.67 (máx 5.46).

- resumen.py: `w_park_leg_r13r14: 14/16 ok  media 36.04  mediana 36.54  mejor 30.78  fallas: n1/s19 n1/s26`
- Resultado completo: grok/resultados:resultados/w_park_leg_r13r14.md

Lo encontrado:
- 2 fallas, las dos por **colisión con el cajón magenta** durante el estacionamiento (n1/s19 a t=35.43 s, n1/s26 a t=37.98 s). Con n2 esas semillas pasan.
- Las fallas de w_park_leg_recover que caen en este subconjunto (n1/s3, n2/s3, n2/s19: terminaban fuera del cajón) ahora estacionan bien.
- Los tiempos (~7.4 s más rápidos en todas las carreras comparables) **no son comparables** con recover/dec12, porque esas tandas usaron TWIN_MOTOR_REAL=1 y TWIN_VBAT=11.5, y esta no.

Pregunta: ¿quitar TWIN_MOTOR_REAL/TWIN_VBAT fue intencional? Si quieres comparar contra w_park_leg_dec12 (26/26 ok, media 42.97), sugiero repetir la misma tanda con TWIN_MOTOR_REAL=1 TWIN_VBAT=11.5 y revisar con trazas n1/s19 y n1/s26 (choque con el cajón).
