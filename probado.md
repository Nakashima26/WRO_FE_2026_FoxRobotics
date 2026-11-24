# Maniobra U de parking

Banco pareado de 96 casos (n1–2, s1–48) con calibración de cámara, motor realista a 11.5 V y coste fijo. Base `w_u_base`, candidato `w_u_rapida`. Se usaron cinco bancos en total.

| Ajuste | Pases estrictos | Colisiones | Δ medio de la U | Δ medio total |
| --- | ---: | ---: | ---: | ---: |
| Base | 95/96 | 1 | — | — |
| Reconsiderar escaneo directo cuando la Pi recupera la línea | 95/96 | 1 | 0 | 0 |
| PWM del arco 170 → 185 | 95/96 | 1 | −149 ms | +16 ms |
| Preparación general 45 → 43 cm y PWM 188 → 195 | 93/96 | 3 | −101 ms | ≈0 ms |
| Preparación corta con condición de pose | 95/96 | 1 | −6 ms | −6 ms |

La U se mide desde el disparo por pose hasta la entrega a escaneo. El tiempo total se compara entre pares estrictos, con parada normal, sin contactos y con la huella dentro de la tolerancia de 5 mm del cajón. El ajuste final conservó los tiempos de vuelta en 96/96 y mantuvo el único contacto de la base en n1/s36.

`PARK_UTURN_PREPARACION_CORTA` queda apagado por defecto en ambos sketches. La prueba se hizo con el perfil de carrera y el override de firmware `"PARK_UTURN_PREPARACION_CORTA": "true"`. Solo se aplica en la preparación CW con arco continuo habilitado y con un cajón temprano cuyo escaneo de regreso cabe antes de la esquina. El giro conserva su PWM y el predictor sigue autorizando la entrada al arco. Ambos sketches compilaron para `esp32:esp32:esp32` con la opción activa.

| Caso | U hasta escaneo | Tiempo total |
| --- | ---: | ---: |
| n1/s19 | 1.767 → 1.673 s | 39.934 → 39.459 s |
| n2/s19 | 1.673 → 1.595 s | 40.209 → 40.076 s |

La prueba de escaneo directo tuvo cero activaciones. En n1/s19, al pasar de largo la primera madera el eje seguía aproximadamente a 0.59 m de la pared, frente a los ~0.30 m de la línea de escaneo. El prototipo queda en los artefactos del banco. La preparación general introdujo contactos con el cajón durante el plan de entrada en n1/s46 y n2/s4. La condición final excluye ambos casos y recupera sus resultados base.
