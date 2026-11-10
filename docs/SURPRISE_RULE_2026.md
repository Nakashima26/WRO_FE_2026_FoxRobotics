# Regla sorpresa 2026: visión y memoria

Regla oficial, versión 9 de noviembre de 2026:
https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Extra-Task.pdf

Es opcional y se declara antes del intento. Una señal, que no puede ser la
primera, cambia de color y mide 50 o 150 mm, conservando su base de 50×50 mm.
Se obedece el color original (opuesto al visible). Cada paso correcto vale 5
puntos, hasta 15. Un paso por el lado equivocado termina el intento.

## Decisión de diseño

El cambio de conducción ocurre en la Pi; el protocolo V2 y el ESP permanecen
iguales. La detección HSV conserva el color físico. Antes de pasar detecciones
al mapa, al control inmediato y a ObstacleMemory, se interpreta la altura y se
obtiene el color de conducción. Se conserva el bbox original para proyectar el
pie; la altura usada como pista de proximidad por LOCK/far-hint se normaliza al
bloque de 100 mm. No se debe dar prioridad a uno por ser físicamente más alto.

La relación alto/ancho ayuda, pero no determina por sí sola la altura. La base
cuadrada se ve más ancha al verla de esquina, el techo cambia con la perspectiva,
y recortar un bloque alto puede hacerlo parecer bajo. Tampoco basta el alto en
píxeles sin distancia: un bloque de 50 mm cercano puede verse mayor que uno de
150 mm lejano.

La distorsión de lente se corrige en el frame como antes. La homografía del piso
solo se aplica al PIE, que está sobre ese plano. La altura se mide en la cámara,
sin warpear el cuerpo del bloque como si fuera parte del piso.

Se soportan dos fuentes de evidencia:

1. **Tabla real, preferida tras calibrar:** pares de distancia longitudinal del
   pie en BEV y altura en píxeles de un bloque NORMAL de 100 mm. Se interpola la
   altura normal esperada y se compara con la observada. No se extrapola.
2. **Geometría, por defecto sin tabla:** H del piso y centro óptico de la cámara
   dan una columna vertical proyectiva. Se resuelve la altura a partir de las
   filas de pie y cima. Su precisión depende de que el montaje y H sean reales.

En geometría, P(X,Y,z)=G(X,Y,1)+z*q, con q=-G(Cx,Cy,1)/Hcam. No requiere
entrenamiento ni cambia los rangos de rojo/verde. La inferencia con tabla usa
una altura equivalente; no promete una medición métrica exacta del bloque.

## Memoria y recortes

Las señales especiales requieren tres observaciones consistentes y apoyo de
piso bajo su base. Un normal bien medido (100±8 mm) puede publicarse de inmediato
para conservar la reacción durante la salida. Los normales conservan el umbral
de área anterior; solo se permite un blob más pequeño si es compatible con 50 mm.

La interpretación conserva color visible, color de conducción, altura confirmada
y asociación por asiento cuando la pose lo permite. ObstacleMemory recibe el
color de conducción y conserva posición y lado mientras el objeto deja de verse.
Un recorte no vuelve a convertir automáticamente la señal en normal. Si el pie
sale por abajo no se actualiza su posición con el borde de la imagen; la memoria
continúa con la estimación anterior. Para una especial recortada lateralmente se
omite la detección nueva y se usa también la memoria. La asociación por asiento
permite recordar la clasificación en vueltas posteriores. GO limpia esta memoria.

La compuerta de piso es conservadora; no resuelve toda oclusión posible. Con una
tabla real, una cima recortada sin clasificación previa no se infiere por el modelo
geométrico. Se espera una vista completa o se reutiliza una clasificación conocida.

## Activación explícita

El valor por defecto es `SURPRISE_RULE=False`; el modo normal conserva su banco.
Desde `src/RASPI/cam`, con el perfil de carrera habitual:

```sh
python -m pure_pursuit.runtime_nuevo --surprise-rule
python -m pure_pursuit.runtime_nuevo --no-surprise-rule
```

También se puede definir `"SURPRISE_RULE": true` en `carrera.json`.
La opción CLI tiene prioridad. Se rechaza arrancar sin H calibrada o con
`ROUTE_SEAT_ROI=True`, porque esa ruta auxiliar todavía solo reconoce el color
visible. La decisión se fija al construir el runtime; no se cambia durante el intento.

Los registros `[SURPRISE]` muestran color visible, color de conducción, clase de
altura, estimación y asiento. FrameResult conserva `visible_positions` y
`surprise_observations` para revisar las decisiones sin perder lo que vio la cámara.

## Calibración con imágenes reales

Fotografiar UN bloque normal de 100 mm rojo o verde en el piso, a varias distancias
y ángulos de interés. Deben ser frames limpios de 640×480, ya enderezados con la
misma calibración de lente, resolución y orientación que el runtime. Su base y
cima deben verse. Usar después los bloques de 50 y 150 mm para validar, no para
etiquetar como normales las imágenes de calibración.

```sh
python -m pure_pursuit.calibrar_sorpresa fotos/*.png --out altura_normal.json
```

El archivo contiene `SURPRISE_NORMAL_HEIGHT_SAMPLES`; copiar ese valor a
`carrera.json`. La distancia de la tabla sale del mismo BEV que se usará al
clasificar: se mide desde el origen BEV, no desde el lente. Se requieren al menos
tres distancias cubriendo 300 mm. El rango debe cubrir las distancias de operación;
una observación fuera del rango no crea una clasificación nueva por extrapolación.
No hay una tabla real inventada o precargada: falta tomar estas fotos en el carro.

## Twin y validación

Sign distingue altura, color visible y color original. El render representa la
altura individual. El sorteo sorpresa se hace después del campo normal y excluye
la primera posición del recorrido. El detector recibe imágenes, nunca el índice,
la altura verdadera ni el color original. El juez usa el original, suma hasta 15
puntos y corta en `wrong_sign_side`. Usa el cruce del eje trasero como las métricas
previas del twin; es una aproximación de evaluación, no un nuevo árbitro oficial.

```sh
python -m pure_pursuit.twin.tools.sorpresa --seed 7 --height 50 --out /tmp/s7-baja
python -m pure_pursuit.twin.tools.sorpresa --seed 7 --height 150 --out /tmp/s7-alta
python -m pure_pursuit.twin.tools.sorpresa --seed 7 --normal --out /tmp/s7-normal
```

Se mantiene el banco actual (engranaje, UART, etc.) mediante sus variables
`TWIN_GEAR`, `TWIN_FW_PARAMS`, `TWIN_PI_UART_US_PER_BYTE` y `TWIN_NOISE_SEED`.
`surprise.json` guarda interpretación, pasos, puntos, colisiones y resultado.
La puntuación extra es previa a una eventual reparación; este banco no ejecuta
acciones de reparación ni calcula el puntaje oficial total.

Estudio de 735 vistas sintéticas: tres alturas, siete distancias, cinco ángulos
del chasis y siete rumbos de observación. En detecciones sin recorte y con apoyo
de piso, la proporción alto/ancho se solapó entre clases. Comparando h con una
curva normal de 100 mm del mismo modelo, los intervalos fueron aproximadamente
0.50–0.61 (bajo), 0.985–1.05 (normal) y 1.51–1.54 (alto). Es evidencia del twin,
no una validación de la lente, luz ni bloques reales.

Las pruebas cubren ambos colores/sentidos, las tres alturas, distancia, recortes,
memoria entre vueltas/GO, calibración por píxeles, oclusión y juez con límite de
15 puntos y conservación de puntos antes de un fallo. La simulación de ToF sigue
siendo 2D: no demuestra si un sensor montado por encima de 50 mm verá el bloque
bajo. La clasificación nueva depende de la cámara, no de medir altura con ToF.

Este modo es experimental hasta verificar imágenes reales y comparar la carrera
y el estacionamiento completos. Reconocer correctamente la señal no garantiza
que todas las trayectorias o el estacionamiento mantengan el resultado anterior.

### Resultado local del prototipo (10 noviembre 2026)

- Suite: **390 passed, 2 skipped, 1 xfailed**.
- Modo normal: **48/48 carreras idénticas** a la referencia anterior, excluyendo
  tiempos de cómputo (semillas 1–24 × ruido 1–2).
- Sorpresa: 24 pistas × dos alturas, ruido 1, una posición elegible por pista,
  seleccionada de forma reproducible. **35/48** obtuvieron 15 puntos, **1/48**
  obtuvo 10 y **12/48** no llegaron a un paso puntuable. **0 pasos incorrectos**
  registrados por el juez de la señal especial.
- Esto NO significa 48 carreras completas: **35/48** completaron 12 giros y
  **6/48** terminaron toda la carrera/estacionamiento. Los controles normales
  de esas 24 pistas tienen 18/24 con 12 giros y 4/24 finalizadas.
- Regresiones conocidas al activar el prototipo: s7 con ambas alturas y s9 con
  50 mm chocan con el cajón al estacionar aunque sus controles terminan; s22 con
  150 mm choca con una señal normal después de 10 giros, frente a 12 del control.
  La inversión de color de la especial fue correcta en esos casos. Hay que
  resolver estas diferencias y validar el clasificador con imágenes reales
  antes de recomendar este modo para un intento oficial.
