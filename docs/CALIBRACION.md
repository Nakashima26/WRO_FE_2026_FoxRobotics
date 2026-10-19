# Calibración completa del carro (paso a paso)

Objetivo: que el gemelo se comporte como el carro real. Tú mides; los scripts
calculan; los resultados se los pasas a Claude (o los aplicas tú con los
comandos del paso 12).

**Reglas que valen para todo:**

- Haz los pasos **en orden**. Varios dependen del anterior (el 2 depende del 1,
  todas las velocidades dependen del 4).
- **Batería cargada** (≥ 12.4 V en reposo) y anota el voltaje al empezar cada
  bloque. El motor va en lazo abierto: con menos voltaje va más lento.
- Mide en el **mismo tapete de la pista** cuando el paso diga "en pista".
- Si un número sale fuera de lo que dice "qué esperar", **no sigas**: repite ese
  paso. Un número malo arrastra a todos los demás.
- Todo lo que generan los scripts queda en `runs/...`. No borres esas carpetas
  hasta que Claude te confirme que ya las usó.

Tiempo total aproximado: **3 h** (cámara 45 min, motor y servo 2 h, sensores 15 min).

---

## Material

| Cosa | Para qué |
|---|---|
| Tablero de ajedrez impreso **10 × 7 cuadros de 25 mm** (9 × 6 esquinas interiores), pegado a algo **rígido y plano** (cartón grueso, acrílico, tabla) | Paso 1 (lente) |
| 9 marcas (cinta, tapitas) y cinta métrica | Paso 2 (piso) |
| Cinta métrica de 3 m y cinta adhesiva | Pasos 4–6 |
| Un pasillo o tramo **recto y liso de ≥ 3 m** (mejor el tapete de la pista) | Pasos 5–6 |
| Plumón pegado al **centro del eje trasero** y papel grande (o el tapete con tiza) | Paso 7 |
| Una tabla lisa forrada con el **tapete de la pista** y un transportador o la app de nivel del celular | Paso 9 (agarre) |
| Laptop con cable USB al ESP32 | Pasos 4–10 |

---

## Parte A — Cámara (en la Raspberry Pi)

Todo en la Pi, desde `src/RASPI/cam`, con monitor (o VNC) para ver las ventanas.

### Paso 1. Lente (tablero de ajedrez)

**Qué arregla:** el lente gran angular curva la imagen (las rectas de las orillas
salen dobladas). Hoy no se corrige y por eso la calibración del piso tiene un
error de **6.1 px**; una buena da 1–2 px.

1. Genera el tablero (cualquier compu con el repo):
   `python -m pure_pursuit.calib_lente --tablero tablero.png`. Imprímelo en
   **A4 horizontal al 100 %**, sin "ajustar a página": la hoja impresa mide
   270 × 195 mm. Con una regla
   comprueba que **un cuadro mide 25.0 mm**. Si mide distinto, anota la medida
   real y úsala en `--square-mm`.
2. Pégalo plano sobre algo rígido. **Si el papel se ondula, la calibración sale
   mala.**
3. Corre:
   ```bash
   python -m pure_pursuit.calib_lente --square-mm 25
   ```
4. Mueve el **tablero** (la cámara se queda en el carro) y presiona **ESPACIO**
   cuando las esquinas salgan dibujadas de colores. Toma **20–25 fotos**:
   - cerca y lejos,
   - en el centro **y en las 4 esquinas y orillas de la imagen** (es donde más
     curva el lente; sin fotos ahí no se corrige ahí),
   - inclinado hacia los lados y hacia arriba/abajo (hasta ~45°).
   - Tablero **quieto** al presionar (foto movida = mala).
   - Si dice "muy parecida, mueve el tablero", cámbialo de lugar.
5. Presiona **C** para calcular.

**Qué esperar:** `rms` < **0.5 px** excelente, < **1.0 px** bien. Si sale > 1.0,
borra y repite (casi siempre es tablero doblado o fotos movidas).

6. Presiona **U** para ver la imagen enderezada: las rectas de las orillas deben
   verse rectas. Si sí, presiona **S** para guardar
   (`pure_pursuit/lens_calib.npz`).

> Desde ese momento **toda** la Pi (carrera, calibraciones) usa la imagen
> enderezada. Para desactivarlo sin borrar el archivo: `FOX_LENS_CALIB=0`.

### Paso 2. Piso (9 puntos) — rehacer después del lente

**Obligatorio después del paso 1**: la calibración vieja era para la imagen sin
enderezar y ya no sirve (la Pi lo avisa con `[BEV] AVISO` al arrancar).

1. Carro en el piso, llantas rectas. Marca los 9 puntos medidos desde el **eje
   DELANTERO** (lateral, adelante), en mm:

   ```
   lejos:  P7 (-220, 550)   P8 (0, 550)   P9 (220, 550)
   media:  P4 (-150, 300)   P5 (0, 300)   P6 (150, 300)
   cerca:  P1  (-80,  80)   P2 (0,  80)   P3  (80,  80)
   ```
   Mide con cinta desde el centro del eje delantero; negativo = izquierda.
   Usa marcas **pequeñas** (punto de plumón sobre cinta): se clickea el centro.
2. Corre `python -m pure_pursuit.calibrate`, presiona **C**, y haz clic en los 9
   puntos **en el orden que dice la pantalla** (la ventana Zoom ayuda;
   clic derecho deshace).
3. **Qué esperar:** error de reproyección < **2 px**. Si sale más, el script
   dice qué punto está mal: revisa esa marca y presiona **R**.
4. **S** para guardar.

### Paso 3. Posición real de la cámara (sale sola de los pasos 1 y 2)

```bash
python -m pure_pursuit.twin.tools.calib.calib_camara
```

Imprime la altura, inclinación, giro y posición reales de la cámara y las
líneas para `config.py` (`CAMERA_HEIGHT_MM`, `CAMERA_TILT_DEG`, ...). Hoy esos
valores son **supuestos** (97 mm y 140 mm).

**Qué esperar:** altura ~80–110 mm, inclinación ~40–50°, giro y roll < 3°. Si
dice que la cámara está torcida (> 3°), endereza el soporte y repite el paso 2.

Copia la salida (o la carpeta) para Claude.

---

## Parte B — Motor, encoder y servo (ESP32 con CalibFox)

### Preparación (una vez)

Dos formas; las dos usan el mismo script. Ya viene con los valores del carro v2
(BNO085 sin MPU6050, encoder B en GPIO 36, servo centro 75), así que compila
tal cual desde el Arduino IDE o con arduino-cli.

**A. Pi apagada, laptop con cable USB al ESP32 (por defecto).** Como las
pruebas de la rama `parking`. El cable tiene que dejar que el carro recorra
~2 m, así que alguien camina con la laptop.

```bash
arduino-cli upload --fqbn esp32:esp32:esp32 -p <PUERTO> src/ESP32/CalibFox   # después de compile
cd src/RASPI/cam && export PYTHONUTF8=1 PYTHONPATH=.
CAP="python -m pure_pursuit.twin.tools.calib.calib_capture --port <PUERTO> --out runs/calib/$(date +%Y%m%d) --maxpwm 255 --vbat <VOLTAJE>"
```
`<PUERTO>` en Mac es `/dev/cu.usbserial-0001` o `/dev/cu.SLAB_USBtoUART`
(`ls /dev/cu.*`). Si no aparece: cierra apps que agarran USB (Spotify lo hace),
desconecta y reconecta; si sigue sin aparecer, instala el driver CP210x de
Silicon Labs.

**B. Sin cable al carro: por el UART de la Pi.** Compila con
`-DCALIB_UART=1`, para el programa de carrera en la Pi y corre el mismo `CAP`
en la Pi con `--port /dev/ttyS0` (el `SERIAL_PORT` de `config.py`).

En las dos: prueba en seco con `--port mock`. **Seguridad:** cualquier tecla
aborta la corrida, y cada corrida tiene un tope de 20 s.

Todos los pasos de abajo escriben en la **misma carpeta** `runs/calib/<fecha>`.

### Paso 4. Encoder: cuentas por mm (SIEMPRE PRIMERO)

**Qué arregla:** la escala de toda la odometría y de todas las velocidades. Hay
una sospecha seria: el firmware supone **20.73 cuentas/mm**, pero si el encoder
es de 12 CPR la cifra real sería **~8.9**. En ese caso la odometría estaría
2.3 veces mal.

1. Dos marcas en el piso separadas **1000 mm exactos** (cinta métrica, no a ojo).
2. ```bash
   ${=CAP} enc --dist 1000 --reps 5
   ```
   (en bash es `$CAP` sin `${= }`).
3. El script te pide **empujar el carro a mano**, recto y sin que patine, de una
   marca a la otra, y luego regresarlo. Alinea siempre el mismo punto (el eje)
   con la marca.

**Qué esperar:** las 5 lecturas a menos de **1 %** entre sí, y "neto al volver"
cerca de 0. Si el neto no vuelve a ~0, se pierden cuentas (cable o ruido):
avísale a Claude antes de seguir.

### Paso 5. Velocidad contra PWM (rápido y lento)

Carro en el piso, llantas rectas, en el recto de ≥ 3 m. **El carro se mueve
solo**: cada repetición va adelante y regresa.

```bash
${=CAP} ramp --reps 3                                          # PWM 20→130, velocidades bajas
${=CAP} ramp --reps 3 --from 150 --to 255 --step 35 --hold 500 # PWM 150→255, velocidades de carrera
${=CAP} deadband --reps 3                                      # dónde empieza a moverse
${=CAP} step --pwms 90,160,230 --reps 3                        # qué tan rápido acelera
```

Si avisa "sin estabilizar", sube `--hold`. Si el carro se desvía mucho de lado,
endereza el servo (paso 7) y repite. Vuelve a medir el voltaje al final; si
bajó **más de 0.4 V**, recarga.

**Qué esperar:** a PWM 255, unos **800–870 mm/s** con la relación 1.25:1. Si
sale muy distinto, avisa: cambia todo el perfil de velocidad.

### Paso 6. Frenado y rodado libre

```bash
${=CAP} coast --pwms 120,230 --reps 3            # corta el motor y deja rodar
${=CAP} coast --pwms 120,230 --reps 3 --brake    # freno como en carrera
```

Después de cada corrida **mide con cinta** cuánto rodó desde el corte y
anótalo aparte. El script no lo pide; el encoder también lo mide, y la cinta
sirve para comprobarlo.

Anota (mm): coast 120 ____ ____ ____ | coast 230 ____ ____ ____ |
freno 120 ____ ____ ____ | freno 230 ____ ____ ____

### Paso 7. Servo: centro y curvatura

1. **Centro:** con `${=CAP} raw "SERVO 75"` (el centro de ServoCentro) el carro
   debe avanzar recto 2 m. Si se va de lado, prueba 73, 74, 76... hasta que vaya
   recto y anota ese valor.
2. **Curvas:**
   ```bash
   ${=CAP} steer --reps 2 --servos 5,20,40,55,65,85,95,110,130,150
   ```
   Con el plumón en el **centro del eje trasero**, el carro hace círculos con
   cada ángulo de servo. Mide el **diámetro** de cada círculo de lado a lado,
   divídelo entre 2 y escribe ese **radio en mm** cuando lo pida (queda en
   `steer.csv`).

**Qué esperar:** el radio mínimo, en los topes, ronda los **110–130 mm**. Si un
lado gira mucho menos que el otro, anótalo: el gemelo lo modela por separado.

### Paso 8. Pérdida de velocidad al girar (scrub)

```bash
${=CAP} lock --pwm 160 --servos 75,40,15 --reps 3
```

Deja **~60 cm** de radio libre: el carro va a dar vueltas en círculo.

### Paso 9. Agarre (plano inclinado, sin script)

**Qué arregla:** hoy el gemelo supone μ = 0.8. Con ese valor salen la
aceleración y frenada máximas y el límite de curva (`A_LAT`, que el
optimizador ya lleva a ~2770 mm/s²). Si el μ real fuera 0.4, el carro real
derraparía en las curvas.

1. Forra una tabla lisa con **el tapete de la pista**.
2. Pon el carro encima **apagado y con el motor desconectado de la batería**
   (que las llantas no puedan girar: si giran, mides la llanta rodando y no el
   agarre). Para el agarre lateral pon el carro **atravesado** (de lado), y repite
   también de frente.
3. Ve levantando la tabla **despacio** hasta que el carro empiece a deslizarse.
   Mide el ángulo en ese momento (transportador o app de nivel).
4. Repite 5 veces de lado y 5 de frente.

**Resultado:** μ = tan(ángulo). Ejemplos: 22° → 0.40, 30° → 0.58, 38° → 0.78.

Anota: ángulos de lado ____ ____ ____ ____ ____; de frente ____ ____ ____ ____ ____

> Si el tapete de la competencia es distinto al tuyo, repite este paso allá: es
> el número que más cambia entre sedes.

### Paso 10. Sensores (opcional, 15 min)

```bash
${=CAP} tof --sensor L --dists 1000,1300,1500   # repite con F, R, B
```

Usa una pared plana a la distancia que pida. Repite también contra la **pared
negra de la pista**: el gemelo supone que el ToF deja de ver la pared negra a
más de 250 mm.

---

## Paso 11. Mandar los resultados

Pásale a Claude:

1. La carpeta `runs/calib/<fecha>` completa (motor, encoder, servo y sensores).
2. Los archivos `pure_pursuit/lens_calib.npz` y `pure_pursuit/bev_calib.npz` de
   la Pi.
3. La salida del paso 3, los ángulos del paso 9, el servo de centro del paso 7 y
   los voltajes.

## Paso 12. Aplicar al gemelo (lo hace Claude; queda aquí por referencia)

```bash
python -m pure_pursuit.twin.tools.calib.calib_fit_motor runs/calib/<fecha>     # k, pwm0, tau, scrub, cuentas/mm
python -m pure_pursuit.twin.tools.calib.calib_fit_steer runs/calib/<fecha>/steer.csv
git apply runs/calib/<fecha>/calib_params.patch
```

Además hay que:

- poner `a_max_mm_s2 = μ_frente × 0.55 × 9810` en `twin/params.py`;
- dejar de usar `TWIN_MOTOR=fisico` en el banco (ese era el motor estimado de la
  hoja de datos; ya no hace falta con el medido);
- revisar que `A_LAT` no pase de **μ_lado × 9810 × 0.8** (20 % de margen);
- poner la pose de la cámara del paso 3 en `config.py` y correr el banco con
  `TWIN_LENS_CALIB=pure_pursuit/lens_calib.npz`;
- si las cuentas/mm cambian, actualizar también `ENC_CUENTAS_POR_MM` en el
  firmware de carrera y correr `python tools/paridad.py`.

Con el gemelo nuevo se vuelve a correr el banco **antes** de cambiar nada del
carro: los tiempos y los márgenes de es01 se van a mover.

## Paso 13. Validar contra el carro real

La calibración solo se comprueba comparando con corridas reales:

1. Con el **firmware de carrera** cargado de nuevo, corre 3–5 vueltas reales con
   logs (`pi.log` y la traza).
2. Apunta dónde quedó cada señal y en qué cajón arrancó.
3. Claude repite esa misma pista en el gemelo y compara tramo por tramo:
   tiempos por vuelta, velocidad en rectas, dónde frena y dónde se separan las
   poses.
4. Lo que no cuadre indica qué paso hay que repetir.

Meta: tiempo por vuelta a **±3 %** y pose a **±5 cm** del real.
