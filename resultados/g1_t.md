# g1 t — iteración algoritmo post-vuelta (U, escaneo, plan/reversa) sobre origin/main 2813d20 — FIN

Base (g1_base, 96 corridas, determinista, rep idéntica): 95/96, media 42.22 s, mediana 41.94, mejor 36.60, falla n1/s36 (colisión con señal a t=7 s, vuelta 1; no es post-vuelta).

Cribado de 14 corridas con U (base 44.47 s; plan 4.71, scan 2.72, U 2.11):
- Sin efecto: MOTOR_REV_RAMPA_MS 150, PARK_REV_PWM 105, PARK_RETORNO_REV_PWM 105, PISO_RECTO 80 (idénticos o peores).
- Peor/falla: PISO_LOCK 95, PISO_VMAX 92 (falla n1/s46), ARRANQUE 86, ATASCO_MS 150, PWM_REV 92/FWD 88 (falla n2/s19).
- Algo: MOTOR_REV_RAMPA_MIN 70 (plan 4.59), PAUSA_MS 150 (4.43), ARC1_PWM 150 (4.58), ALEJAR_CM 40 (U 1.87 pero plan más largo).
- Combos: PAUSA 150+ARC1 150+RMIN 70 -> 44.17 ok; PAUSA 100+... -> 44.04 ok (plan 4.09); PAUSA 60 -> falla n2/s3; +ALEJAR_CM 40 -> falla n1/s42.

Tandas de 96:
| tanda | ok | media | mediana | mejor | fallas |
|---|---|---|---|---|---|
| base | 95/96 | 42.22 | 41.94 | 36.60 | n1/s36 |
| PAUSA100+ARC1 150+RMIN70 | 90/96 | 41.67 | 41.57 | 36.02 | n1/s36 s39 s45 s7 n2/s34 s39 |
| PAUSA_MS 150 solo | 94/96 | 41.96 | 41.84 | 36.27 | n1/s36 n2/s34 |
(rm70 solo no se corrió; la tanda se interrumpió por fin de presupuesto.)

Conclusión: ninguna variante cumple "mismas o menos fallas y menor media" -> NO se commiteó nada; grok/t/g1 queda en 2813d20. Acortar PAUSA_MS baja ~0.3-0.5 s pero introduce fallas (n2/s34 aparece ya con 150). Patch del mejor candidato (rechazado) en pend/g1_t_candidato_rechazado.patch, solo referencia.
Todo es del twin; nada medido en el carro. Pendiente: causa de n2/s34 y s39 con pausa corta; probar RMIN70 solo en 96.

FIN
