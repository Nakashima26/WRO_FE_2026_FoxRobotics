# Respuesta: l1v_diag288

_2026-11-25 15:01 PST (Tijuana) — estado: TERMINADA_

Hecho: corrí la tanda en origin/grok/t/l1v_diag288 (63886cd9 (padre 0ee69022)) con tu comando (l1v_uno.sh con l1v_diag.py, FF 4.28 / FD 5.8, PI_EXTRA {}, FW_OVER {}): n1-6 x s1-48 (288 carreras), -P 7, bajo twin.lock. HEAD esperado 0ee69022: coincide.

- resumen.py: `l1v_diag288: 270/288 ok  media 35.83  mediana 35.37  mejor 31.89  fallas: n1/s23 n1/s32 n1/s47 n2/s13 n2/s23 n2/s36 n3/s12 n3/s27 n3/s48 n4/s15 n4/s16 n5/s3 n5/`
- l1v_diag.py summary (inicio): `Carreras con muestras: 288. Media por carrera en segundos.
tc0 | CTE/subida/espera | 0.60524 | 0.36392
tc0 | asiento | 0.00071 | 0.00008
tc0 | esquina | 0.15908 | 0.05711
tc0 | otro | 0.01031 | 0.00000
tc0 | pared sin ver | 0.00750 | 0.00165
tc0 | perfil restante | 1.20471 | 0.58217
tc0 | perfil restante + rampa | 0.00010 | 0.00000
tc0 | prefijo | 2.05264 | 1.43114
tc0 | rampa | 0.00219 | 0.00002
`
- Paradas: collision 7, race_finished 278, stuck 3
- Reloj total: 369.8 s. VÁLIDA: corrió bajo twin.lock (0 procesos de banco ajenos al inicio), 369.8 s para 288 carreras (referencia ~300-330 s sin carga, 3x la de 96), carga máx 7.71, mem máx 4665 MB, OOM/Killed/MemoryError: 0, mean_pi_frame_ms 3.18-7.20 (media 3.83).
- Tipo de prueba: carrera completa, banco diagnóstico sin variante (PI_EXTRA {}, FW_OVER {}), FF 4.28 / FD 5.8, 288 carreras (n1-6 x s1-48). Por la corrección l1v_ajuste, estos mismos STOP/COSTO se publican también como la0g (árbol idéntico a ae0f25ed), y son la base de l1v_cartas_288 (FF 4.28 / FD 5.8, FW_OVER {}).

Detalle por carrera (STOP y COSTO): `grok/resultados:resultados/l1v_diag288.md`.
