# uparkx1: ¿por qué a veces hace la U antes de estacionarse? (commit a68fa06, rama grok/t/uparkx1)

**Tipo de prueba:** 3 vueltas + estacionamiento (`--solo-cajon`, `FOX_PARK_POS=random`, `ruta_perfil`, `grok_cfg.json` de la rama), ruidos 1, 2 y 3. Ninguna tanda anterior usó estos ruidos (las tandas de cajón fueron con 4/5/7/10), así que **no comparo tiempos con tandas anteriores**; solo comparo base contra parche sobre los mismos casos.

## Resumen

- **Qué decide U o directo:** `vigilarSinU()` en `PurePursuit.ino`. Si la Pi manda `pka >= PARK_SINU_PKA_MIN_MM` (100 mm), no se decide nada: siempre se estaciona directo. Si `pka < 100` (cajón al inicio de la recta), en `along >= PARK_SINU_DEC_MM` (-560) se exigen **tres** cosas para ir directo: `|ext - PARK_PARED_CM(28)| <= 5 cm`, `|err carril| <= 14°` y **`pka - PARK_SINU_B2_OFF_MM(80) - along >= PARK_SINU_CORRE_MM(330)`** (la recta que queda para armar el sonar antes de la madera). Si falla cualquiera, `parkSinUDec = 2` y se hace la U por pose (`along >= 500`).
- **Seeds 17 y 9:** pka de 305 a 389 mm, o sea mayor o igual a 100. Nunca entran a la decisión, así que siempre van directo.
- **Seeds 7 y 20:** pka entre -92 y -249, así que sí entran a la decisión. En los 6 casos con U, la pared y el ángulo **sí pasaban** (ext 28.0–30.3 cm, err de 2.8° a 6.6° en valor absoluto). **La U la causa solo el umbral de recta, `CORRE_MM = 330`:** seed 20 llega con corre de 228 a 296 mm (cajón en W a 1016 mm, más al inicio) y seed 7 con 326 a 385 mm. Seed 7 está justo en el borde: el ruido 2 da 326, se queda a 4 mm del umbral y hace U, mientras que los ruidos 1 y 3 dan 341 y 385 y van directo. Por eso "a veces" hace la U.
- **Cuánta recta hace falta de verdad:** el sonar se arma tras `PARK_ARMADO_MS` = 407 ms. En el caso más corto (seed 20 con ruido 2, corre 228) la madera se vio 0.67 s después de entrar al escaneo (along -306, dentro de la ventana pka-80 ± 130), así que 330 sobra.
- **Parche propuesto:** `PARK_SINU_CORRE_MM` de 330 a 200. Los 12 casos van directo y salen limpios. En la validación con 72 casos baja la suma en 21.5 s, sin cambiar la holgura mínima global ni ensuciar ningún caso. No hace falta tocar la ruta de la vuelta 3: ya deja el carro a unos 28–30 cm de la pared.

## Tabla de los 12 casos

Base es a68fa06 tal cual; el parche es `PARK_SINU_CORRE_MM=200`. El dato de decisión sale del log con un print extra de `pka` y `corre` (solo diagnóstico, no va en el parche). "Limpia" significa terminada, sin choques, sin señales tocadas, sin violaciones de lado y sin contactos ignorados con la pared exterior.

| seed | ruido | U (base) | tiempo base s | STOP base | pared mín mm | señal mín mm | decisión (base) | U (parche) | tiempo parche s | pared mín parche mm | limpia base/parche |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 7 | 1 | no | 40.255 | race_finished | 36.84 | 42.27 | DIRECTO ext=30.0 err=-6.7 pka=-129 corre=341 | no | 40.278 | 30.58 | sí/sí |
| 7 | 2 | **sí** | 43.837 | race_finished | 43.92 | 41.55 | CON U ext=30.3 err=-6.6 pka=-142 **corre=326** | no | 40.272 | 35.73 | sí/sí |
| 7 | 3 | no | 40.595 | race_finished | 28.12 | 48.4 | DIRECTO ext=29.9 err=-5.7 pka=-92 corre=385 | no | 39.937 | 22.16 | sí/sí |
| 17 | 1 | no | 38.378 | race_finished | 37.76 | 77.54 | pka=316 ≥ 100: sin decisión, directo | no | 38.378 | 37.76 | sí/sí |
| 17 | 2 | no | 37.726 | race_finished | 36.78 | 67.15 | pka=305 ≥ 100: sin decisión, directo | no | 37.726 | 36.78 | sí/sí |
| 17 | 3 | no | 38.482 | race_finished | 34.07 | 76.37 | pka=316 ≥ 100: sin decisión, directo | no | 38.482 | 34.07 | sí/sí |
| 9 | 1 | no | 39.469 | race_finished | 44.65 | 26.57 | pka=388 ≥ 100: sin decisión, directo | no | 39.469 | 44.65 | sí/sí |
| 9 | 2 | no | 41.234 | race_finished | 39.25 | 21.19 | pka=315 ≥ 100: sin decisión, directo | no | 41.234 | 39.25 | sí/sí |
| 9 | 3 | no | 38.079 | race_finished | 47.97 | 30.48 | pka=389 ≥ 100: sin decisión, directo | no | 38.079 | 47.97 | sí/sí |
| 20 | 1 | **sí** | 42.353 | race_finished | 34.53 | 74.05 | CON U ext=28.7 err=4.1 pka=-178 **corre=296** | no | 37.542 | 45.24 | sí/sí |
| 20 | 2 | **sí** | 41.603 | race_finished | 24.85 | 44.63 | CON U ext=28.0 err=2.8 pka=-249 **corre=228** | no | 37.351 | 31.12 | sí/sí |
| 20 | 3 | **sí** | 43.252 | race_finished | 28.59 | 64.57 | CON U ext=28.5 err=3.6 pka=-182 **corre=295** | no | 38.679 | 37.5 | sí/sí |

En estos 12, el parche quita las 4 U, y cada caso que se arregla ahorra entre 3.6 y 4.9 s. Las señales no cambian. La pared mínima en los casos cambiados sube o queda en 31.1 mm o más.

Probé también `CORRE_MM=250`. Con 250, seed 20 con ruido 2 (corre 228) sigue haciendo U y el resto va directo, así que 250 se queda corto.

## Validación (72 casos: seeds 1–24 × ruidos 1–3, mismo tipo de prueba)

| | limpias | con U | suma s | media s | pared mín mm | señal mín mm |
|---|---|---|---|---|---|---|
| base a68fa06 | 71/72 | 17 | 2965.099 | 41.182 | 18.03 | 13.13 |
| CORRE_MM=200 | 71/72 | 12 | 2943.578 | 40.883 | 18.03 | 13.13 |

En la comparación pareada la suma baja 21.521 s. Solo cambian 5 casos, todos de U a directo y todos limpios:

- seed 20, ruido 1: de 42.353 a 37.463 s (-4.89 s); pared mínima de 34.5 a 47.7 mm.
- seed 7, ruido 2: de 43.837 a 40.287 s (-3.55 s); pared mínima de 43.9 a 36.0 mm.
- seed 20, ruido 2: de 41.603 a 36.637 s (-4.97 s); pared mínima de 24.9 a 22.5 mm.
- seed 21, ruido 2: de 46.041 a 42.527 s (-3.51 s); pared mínima de 42.7 a 34.5 mm.
- seed 20, ruido 3: de 43.252 a 38.651 s (-4.60 s); pared mínima de 28.6 a 38.0 mm.

En la validación, el parche da 40.287 s para seed 7 con ruido 2 y 37.463 s para seed 20 con ruido 1. Esos casos difieren unas centésimas de la tabla de arriba porque la tabla salió del firmware con el print de diagnóstico, que añade tiempo de serial.

- Las 12 U que quedan con el parche (seeds 3, 5, 19 y 22) son legítimas: la ruta trae al carro a 69–77 cm de la pared y girado de 19° a 38°, porque la señal de la entrada obliga a pasar por adentro. Ahí no se puede ir directo, y la condición de pared y ángulo las sigue mandando a la U.
- El único caso no limpio es seed 23 con ruido 1: choca con la señal verde S/T2 a los 4.1 s, en la vuelta 1. Pasa igual en la base y no tiene que ver con el estacionamiento.
- El caso con menos recta que se probó directo fue seed 20 con ruido 2 (corre 228). Entre 200 y 228 mm no hay casos observados. Si se quiere más margen, 225 también arregla los 5 casos de esta validación.

## Parche propuesto

```diff
--- a/src/ESP32/PurePursuit/PurePursuit.ino
+++ b/src/ESP32/PurePursuit/PurePursuit.ino
@@ -1524,7 +1524,7 @@ PARK_AJ long         PARK_SINU_PKA_MIN_MM        = 100;
 bool                 parkDirecto                 = false; // true: estaciona de frente (sin U); false: tras la U
 int                  parkSinUDec                 = 0;     // para pka < PKA_MIN: 0 sin decidir, 1 directo, 2 con U
 PARK_AJ float        PARK_SINU_DEC_TOL_CM        = 5.0f;
-PARK_AJ long         PARK_SINU_CORRE_MM          = 330;
+PARK_AJ long         PARK_SINU_CORRE_MM          = 200;   // recta minima (pka - B2_OFF - along) para armar el sonar (PARK_ARMADO_MS) antes de la madera; 330 mandaba a la U casos con 228-326 mm que se estacionan directo limpios
 PARK_AJ float        PARK_SINU_DEC_ANG_DEG       = 14.0f;
 PARK_AJ long         PARK_SINU_DEC_MM            = -560;
```

El mismo cambio se puede probar sin tocar el código, con `"PARK_SINU_CORRE_MM": "200"` en `fw` de `grok_cfg.json`. No toqué main ni otras ramas, y el firmware de grok/t/uparkx1 quedó igual. Los `fw_debug.log` y `trace.csv` de los 12 casos están en la máquina de Grok, en `/tmp/uparkx1/{base,c330,c250,c200}/` y la validación en `/tmp/uparkx1/{vbase,vc200}/`. No los subo porque son carpetas grandes.
