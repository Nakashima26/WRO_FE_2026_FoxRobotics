# s20cal1_metricas

_2026-11-25 12:14 PST (Tijuana)_ · commit 4938aa0f (origin/grok/t/s20cal1_metricas, worktree aislado; main sin tocar)

**Tipo de prueba:** carrera completa del twin con latencia (latencia/uno.sh → costo.py → drive.py `ruta_perfil --solo-cajon`, 3 vueltas / 12 giros + estacionamiento), FF 4.28 / FD 5.8, campo CCW con parking W (FOX_PARK_POS=random), semilla s20, ruido n1..8, -P 7. Tres grupos secuenciales bajo twin.lock: TWIN_SERVO_TAU 0.10 (cal1_m10), 0.06 (cal1_m06), 0.16 (cal1_m16). Env: TOF_SHORT_K 0.5, MOTOR_CURVA_REAL 1, CAMERA_FROM_CALIB 1, MOTOR_REAL 1, VBAT 11.5, COST_FIJO 1, LENS_CALIB lens_calib.npz, PARK_GAP 291, STALL_TAIL 35, REV_WHEEL_SCALE 0.65, COST_FIRST 1.95, PI_EXTRA ROUTE_MOTOR_CURVE_LIVE=false, FW_OVER MOTOR_CURVE_LIVE=false. LAT_OUT=/tmp/lat-cal1.

## Validez

TANDA VÁLIDA · procesos de banco ajenos al inicio: 0 · OOM/Killed/MemoryError: 0
- cal1_m10 (TAU 0.10): VÁLIDA: pared 20.8 s para 8 carreras (carrera más larga wall_time_s 12.5 s), carga máx 5.35, mem máx 4479 MB, mean_pi_frame_ms 3.24-3.65.
- cal1_m06 (TAU 0.06): VÁLIDA: pared 13.5 s para 8 carreras (carrera más larga wall_time_s 9.0 s), carga máx 5.53, mem máx 4088 MB, mean_pi_frame_ms 3.25-6.51.
- cal1_m16 (TAU 0.16): VÁLIDA: pared 15.9 s para 8 carreras (carrera más larga wall_time_s 9.5 s), carga máx 6.29, mem máx 4436 MB, mean_pi_frame_ms 3.06-3.31.

## Resumen

- cal1_m10: 0/8 ok
- cal1_m06: 0/8 ok
- cal1_m16: 0/8 ok

resumen.py:

    cal1_m10: 0/8 ok  fallas: n1/s20 n2/s20 n3/s20 n4/s20 n5/s20 n6/s20 n7/s20 n8/s20
    cal1_m06: 0/8 ok  fallas: n1/s20 n2/s20 n3/s20 n4/s20 n5/s20 n6/s20 n7/s20 n8/s20
    cal1_m16: 0/8 ok  fallas: n1/s20 n2/s20 n3/s20 n4/s20 n5/s20 n6/s20 n7/s20 n8/s20

## Por réplica

Definiciones (todo sale de las trazas; n/d = no existe en la traza):
- ok / ok_t / fuera_mm: `optimizar_park.ok_t` y `fuera_mm` (PYTHONPATH=.); ok = parada normal sin colisiones/contactos, fuera ≤ 5 mm y margen de pared ≥ 0. ok_t = total_time_s si ok (n/d si ok=no, así lo define ok_t); total_s = total_time_s de STOP siempre.
- STOP / causa: stop_reason de la línea STOP; causa = colisiones (causa@t), contactos ignorados, fuera > 5 mm, margen de pared < 0, stop distinto de race_finished.
- ang_final: heading de la última fila de trace.csv − 180°, normalizado a [−180, 180) (parking W; dir/park verificados en field.txt).
- tF/tB: primer ACK:V2 con tc=0 y ambas lecturas ≥ 0 (−1 = sin lectura).
- tcK: t del primer ACK con tc ≥ K menos t del primer ACK con pw > 0 (s, acumulado).
- cruces/s: ACK sin repetidos consecutivos, tc 4..11, pw > 0; lado L si srv < 70, R si srv > 80 (zona 70–80 omitida); cruces = cambios de lado / (t_último − t_primero filtrado). Entre paréntesis: cruces/segundos.
- legs park (fw_debug.log, líneas `PARK plan k ADELANTE|REVERSA EXT|INT|RECTO`): A/R = adelante/reversa; nREC = tramos RECTO, nEXT/nINT = arcos; fin RECTA = tramos que acabaron por `fin (RECTA)`; fin de tramos = causas `PARK plan k fin (…)`; TERMINADO = motivo de `ESTACIONAMIENTO … TERMINADO`.

| tag | TAU | réplica | ok | ok_t s | total_s | fuera_mm | STOP | causa | giros | dir/park | ang_final° | tF | tB | tc1 | tc2 | tc3 | tc4 | tc5 | tc6 | tc7 | tc8 | tc9 | tc10 | tc11 | tc12 | cruces/s | legs park (REC/EXT/INT) | nREC | nEXT | nINT | fin RECTA | fin de tramos | TERMINADO | pi_ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| cal1_m10 | 0.10 | n1 | no | n/d | 39.528 | 35.9 | race_finished | fuera 35.9mm>5 | 12 | CCW/W | -2.9 | 123 | 34 | 3.52 | 6.46 | 10.46 | 12.68 | 15.23 | 17.63 | 20.30 | 21.90 | 24.12 | 26.64 | 29.22 | 30.73 | 1.332 (24/18.0s) | 1R-EXT 2R-INT 3A-EXT 4R-INT | 0 | 2 | 2 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:TH | PLAN completo | 3.24 |
| cal1_m10 | 0.10 | n2 | no | n/d | 40.665 | 39.3 | race_finished | contacto_ign:pared exterior; fuera 39.3mm>5; pared min_clr -38.32 | 12 | CCW/W | -2.4 | 123 | 35 | 3.84 | 7.04 | 11.12 | 13.45 | 16.01 | 18.52 | 21.23 | 22.91 | 25.16 | 27.62 | 30.15 | 31.62 | 1.213 (22/18.1s) | 1R-EXT 2R-INT 3A-EXT 4R-INT | 0 | 2 | 2 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:TH | PLAN completo | 3.25 |
| cal1_m10 | 0.10 | n3 | no | n/d | 40.286 | 51.8 | race_finished | fuera 51.8mm>5 | 12 | CCW/W | -32.6 | 117 | 33 | 3.73 | 6.69 | 10.71 | 13.07 | 15.59 | 18.00 | 20.67 | 22.28 | 24.62 | 27.05 | 29.43 | 30.88 | 1.238 (22/17.8s) | 1R-EXT 2R-INT 3A-EXT 4R-INT 5A-EXT 6R-INT 7A-EXT 8R-INT 9A-EXT 10R-INT | 0 | 5 | 5 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:ATRAS 5:HOLGURA 6:ATRAS 7:HOLGURA 8:ATRAS 9:HOLGURA 10:ATRAS | PLAN legs agotados | 3.53 |
| cal1_m10 | 0.10 | n4 | no | n/d | 39.903 | 40.6 | race_finished | fuera 40.6mm>5 | 12 | CCW/W | -2.0 | 113 | 34 | 3.87 | 6.86 | 10.85 | 13.16 | 15.68 | 18.08 | 20.72 | 22.31 | 24.59 | 27.11 | 29.65 | 31.11 | 1.340 (24/17.9s) | 1R-EXT 2R-INT 3A-EXT 4R-INT | 0 | 2 | 2 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:TH | PLAN completo | 3.46 |
| cal1_m10 | 0.10 | n5 | no | n/d | 39.649 | 38.5 | race_finished | fuera 38.5mm>5 | 12 | CCW/W | -2.7 | 126 | 31 | 3.50 | 6.41 | 10.45 | 12.73 | 15.28 | 17.68 | 20.36 | 22.01 | 24.20 | 26.70 | 29.34 | 30.83 | 1.328 (24/18.1s) | 1R-EXT 2R-INT 3A-EXT 4R-INT | 0 | 2 | 2 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:TH | PLAN completo | 3.30 |
| cal1_m10 | 0.10 | n6 | no | n/d | 39.508 | 31.1 | race_finished | fuera 31.1mm>5 | 12 | CCW/W | -2.9 | 131 | 40 | 3.50 | 6.39 | 10.35 | 12.65 | 15.18 | 17.57 | 20.22 | 21.80 | 23.99 | 26.51 | 29.06 | 30.54 | 1.344 (24/17.9s) | 1R-EXT 2R-INT 3A-EXT 4R-INT | 0 | 2 | 2 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:TH | PLAN completo | 3.29 |
| cal1_m10 | 0.10 | n7 | no | n/d | 40.789 | 38.0 | race_finished | contacto_ign:pared exterior; fuera 38.0mm>5; pared min_clr -42.06 | 12 | CCW/W | -2.0 | 129 | 40 | 3.99 | 7.10 | 11.07 | 13.35 | 15.87 | 18.25 | 20.92 | 22.58 | 24.83 | 27.35 | 29.87 | 31.40 | 1.332 (24/18.0s) | 1R-EXT 2R-INT 3A-EXT 4R-INT | 0 | 2 | 2 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:TH | PLAN completo | 3.65 |
| cal1_m10 | 0.10 | n8 | no | n/d | 40.911 | 51.1 | race_finished | fuera 51.1mm>5 | 12 | CCW/W | -34.2 | 128 | 35 | 3.62 | 6.65 | 10.75 | 13.12 | 15.67 | 18.13 | 20.83 | 22.60 | 24.91 | 27.34 | 29.74 | 31.23 | 1.217 (22/18.1s) | 1R-EXT 2R-INT 3A-EXT 4R-INT 5A-EXT 6R-INT 7A-EXT 8R-INT 9A-EXT 10R-INT | 0 | 5 | 5 | 1 | 1:RECTA 2:HOLGURA 3:HOLGURA 4:ATRAS 5:HOLGURA 6:ATRAS 7:HOLGURA 8:ATRAS 9:HOLGURA 10:ATRAS | PLAN legs agotados | 3.26 |
| cal1_m06 | 0.06 | n1 | no | n/d | 39.139 | 41.9 | race_finished | fuera 41.9mm>5 | 12 | CCW/W | -3.4 | 123 | 34 | 3.97 | 6.83 | 10.79 | 13.11 | 15.56 | 17.94 | 20.55 | 22.09 | 24.31 | 26.74 | 29.08 | 30.61 | 0.916 (16/17.5s) | 1R-EXT 2R-INT 3A-EXT 4R-INT | 0 | 2 | 2 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:TH | PLAN completo | 3.45 |
| cal1_m06 | 0.06 | n2 | no | n/d | 11.902 | 1572.0 | collision | colisión:señal Red N/X2@12.7s; contacto_ign:pared exterior; fuera 1572.0mm>5; pared min_clr -105.33; stop=collision | 2 | CCW/W | 125.7 | 123 | 35 | 3.77 | 7.04 | n/d | n/d | n/d | n/d | n/d | n/d | n/d | n/d | n/d | n/d | n/d | n/d | 0 | 0 | 0 | 0 | n/d | n/d | 6.51 |
| cal1_m06 | 0.06 | n3 | no | n/d | 39.134 | 37.7 | race_finished | fuera 37.7mm>5 | 12 | CCW/W | -2.7 | 117 | 33 | 3.95 | 6.93 | 10.91 | 13.26 | 15.75 | 18.15 | 20.73 | 22.25 | 24.44 | 26.84 | 29.18 | 30.65 | 0.921 (16/17.4s) | 1R-EXT 2R-INT 3A-EXT 4R-INT | 0 | 2 | 2 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:TH | PLAN completo | 3.44 |
| cal1_m06 | 0.06 | n4 | no | n/d | 39.666 | 34.5 | race_finished | fuera 34.5mm>5 | 12 | CCW/W | -1.9 | 113 | 34 | 3.89 | 6.87 | 10.84 | 13.16 | 15.65 | 18.02 | 20.57 | 22.13 | 24.44 | 26.93 | 29.27 | 30.82 | 1.134 (20/17.6s) | 1R-EXT 2R-INT 3A-EXT 4R-INT | 0 | 2 | 2 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:TH | PLAN completo | 3.47 |
| cal1_m06 | 0.06 | n5 | no | n/d | 35.032 | 386.7 | collision | colisión:señal Green W/T2@35.7s; contacto_ign:pared exterior; fuera 386.7mm>5; pared min_clr -75.7; stop=collision | 12 | CCW/W | -71.7 | 126 | 31 | 4.47 | 5.47 | 11.68 | 13.92 | 16.45 | 18.79 | 21.37 | 22.94 | 25.20 | 27.60 | 30.09 | 31.62 | 1.132 (20/17.7s) | n/d | 0 | 0 | 0 | 0 | n/d | n/d | 4.08 |
| cal1_m06 | 0.06 | n6 | no | n/d | 39.963 | 23.7 | race_finished | fuera 23.7mm>5 | 12 | CCW/W | -3.0 | 131 | 40 | 3.81 | 6.69 | 10.64 | 12.92 | 15.44 | 17.84 | 20.39 | 21.91 | 24.16 | 26.71 | 29.17 | 30.76 | 1.123 (20/17.8s) | 1R-EXT 2R-INT 3A-EXT 4R-INT 5A-EXT 6R-INT | 0 | 3 | 3 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:ATRAS 5:FRENTE 6:TH | PLAN completo | 3.37 |
| cal1_m06 | 0.06 | n7 | no | n/d | 34.164 | 368.8 | collision | colisión:señal Green W/T2@34.9s; contacto_ign:pared exterior; fuera 368.8mm>5; pared min_clr -71.59; stop=collision | 12 | CCW/W | -60.9 | 129 | 40 | 3.78 | 6.71 | 10.63 | 13.06 | 15.59 | 17.96 | 20.54 | 22.10 | 24.36 | 26.79 | 29.29 | 30.80 | 1.016 (18/17.7s) | n/d | 0 | 0 | 0 | 0 | n/d | n/d | 4.11 |
| cal1_m06 | 0.06 | n8 | no | n/d | 39.070 | 39.6 | race_finished | fuera 39.6mm>5 | 12 | CCW/W | -2.0 | 128 | 35 | 3.43 | 6.45 | 10.51 | 12.89 | 15.38 | 17.75 | 20.33 | 21.90 | 24.24 | 26.67 | 28.98 | 30.49 | 1.252 (22/17.6s) | 1R-EXT 2R-INT 3A-EXT 4R-INT | 0 | 2 | 2 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:TH | PLAN completo | 3.25 |
| cal1_m16 | 0.16 | n1 | no | n/d | 44.233 | 7.1 | race_finished | fuera 7.1mm>5 | 12 | CCW/W | -0.1 | 123 | 34 | 4.35 | 7.87 | 12.02 | 14.30 | 16.89 | 19.41 | 22.32 | 24.12 | 26.64 | 29.01 | 31.85 | 33.21 | 1.377 (26/18.9s) | 1R-EXT 2R-INT 3A-EXT 4R-INT 5A-EXT 6R-INT 7A-EXT 8R-INT 9A-EXT 10R-EXT | 0 | 6 | 4 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:ATRAS 5:HOLGURA 6:ATRAS 7:HOLGURA 8:ATRAS 9:FRENTE 10:TH | PLAN completo | 3.20 |
| cal1_m16 | 0.16 | n2 | no | n/d | 41.876 | 31.5 | race_finished | fuera 31.5mm>5 | 12 | CCW/W | -4.4 | 123 | 35 | 3.84 | 7.24 | 11.55 | 13.91 | 16.49 | 19.07 | 22.01 | 23.79 | 26.22 | 28.73 | 31.42 | 32.87 | 1.268 (24/18.9s) | 1R-EXT 2R-INT 3A-EXT 4R-INT | 0 | 2 | 2 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:ATRAS | PLAN completo | 3.08 |
| cal1_m16 | 0.16 | n3 | no | n/d | 42.759 | 30.1 | race_finished | fuera 30.1mm>5 | 12 | CCW/W | -5.1 | 117 | 33 | 4.42 | 5.80 | 11.93 | 14.10 | 16.76 | 19.26 | 22.18 | 23.87 | 26.39 | 29.06 | 31.88 | 33.49 | 1.136 (22/19.4s) | 1R-EXT 2R-INT 3A-EXT 4R-INT | 0 | 2 | 2 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:ATRAS | PLAN completo | 3.19 |
| cal1_m16 | 0.16 | n4 | no | n/d | 44.155 | 14.7 | race_finished | fuera 14.7mm>5 | 12 | CCW/W | 4.0 | 113 | 34 | 4.36 | 7.83 | 12.07 | 14.44 | 16.99 | 19.66 | 22.72 | 24.49 | 26.98 | 29.50 | 32.36 | 33.80 | 1.138 (22/19.3s) | 1R-EXT 2R-INT 3A-EXT 4R-INT 5A-EXT 6R-INT 7A-EXT | 0 | 4 | 3 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:ATRAS 5:HOLGURA 6:ATRAS 7:TH | PLAN completo | 3.26 |
| cal1_m16 | 0.16 | n5 | no | n/d | 41.926 | 30.0 | race_finished | fuera 30.0mm>5 | 12 | CCW/W | -3.2 | 126 | 31 | 4.01 | 5.39 | 11.60 | 13.85 | 16.43 | 18.84 | 21.87 | 23.58 | 26.03 | 28.43 | 31.30 | 32.63 | 1.280 (24/18.7s) | 1R-EXT 2R-INT 3A-EXT 4R-INT | 0 | 2 | 2 | 0 | 1:FONDO 2:HOLGURA 3:HOLGURA 4:TH | PLAN completo | 3.18 |
| cal1_m16 | 0.16 | n6 | no | n/d | 33.721 | 175.5 | collision | colisión:cajón magenta@34.4s; fuera 175.5mm>5; stop=collision | 12 | CCW/W | 3.9 | 131 | 40 | 3.54 | 6.75 | 10.89 | 13.20 | 15.74 | 18.29 | 21.33 | 23.04 | 25.50 | 28.29 | 31.17 | 32.54 | 1.449 (28/19.3s) | n/d | 0 | 0 | 0 | 0 | n/d | n/d | 3.31 |
| cal1_m16 | 0.16 | n7 | no | n/d | 39.086 | 97.0 | collision | colisión:cajón magenta@39.8s; fuera 97.0mm>5; stop=collision | 12 | CCW/W | -28.8 | 129 | 40 | 3.90 | 7.05 | 11.21 | 13.52 | 16.10 | 18.69 | 21.63 | 23.39 | 25.92 | 28.38 | 31.16 | 32.54 | 1.159 (22/19.0s) | 1R-EXT 2R-INT 3A-EXT | 0 | 2 | 1 | 1 | 1:RECTA 2:HOLGURA 3:HOLGURA | n/d | 3.27 |
| cal1_m16 | 0.16 | n8 | no | n/d | 38.321 | 100.3 | collision | colisión:cajón magenta@39.0s; fuera 100.3mm>5; stop=collision | 12 | CCW/W | -32.4 | 128 | 35 | 3.48 | 6.62 | 10.82 | 13.17 | 15.72 | 18.27 | 21.18 | 22.86 | 25.35 | 27.78 | 30.46 | 31.85 | 1.394 (26/18.7s) | 1R-EXT 2R-INT 3A-EXT | 0 | 2 | 1 | 1 | 1:RECTA 2:HOLGURA 3:HOLGURA | n/d | 3.06 |

## Errores

- ninguno detectado

## Comparabilidad

Solo comparable con tandas del mismo tipo (carrera completa CCW parking W, s20, n1..8). Las tandas s20cal1 / s20cal1_sep / s20cal1_tau06 / s20cal1_tau16 usan el mismo env pero FF 2.8 / FD 3.8 (esta usa 4.28 / 5.8) y otro commit, así que no se comparan totales directamente aquí. CSV con las mismas filas: resultados/s20cal1_metricas.csv. No se suben trazas.
