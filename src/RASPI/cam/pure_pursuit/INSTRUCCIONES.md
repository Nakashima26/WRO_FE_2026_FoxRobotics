 # Pure Pursuit — Instrucciones de uso

Todos los comandos se corren desde `src/RASPI/cam/`.

---

## 1. Calibrar la cámara (una sola vez, o si mueves el montaje)

La Pi Cam v2 tiene barrel distortion. La homografía asume pinhole: si puedes,
calibra intrínsecos **antes** del BEV. Los videos de corrida (HUD cámara|BEV)
**no sirven** para reestimar H — el BEV ya está warpeado y no hay marcadores.

### 1a. Intrínsecos (opcional, chessboard)

```bash
python -m pure_pursuit.calibrate_intrinsics
```

ESPACIO captura el tablero (mín. ~8 poses, bordes y cerca/lejos). C calcula, S guarda `cam_intrinsics.npz`.

### 1b. Homografía BEV (9 puntos en el suelo)

Coloca marcadores en las posiciones de `CALIB_REAL_MM` (grilla 3×3: cerca / media / lejos × izq / centro / der).

```bash
python -m pure_pursuit.calibrate
```

1. Ventana en vivo → `C` congela
2. Clic en los 9 marcadores en orden (subpíxel automático). Derecho = deshacer
3. Preview BEV + **grilla métrica** sobre la cámara: las líneas deben coincidir con el piso
4. `S` guarda `bev_calib.npz` (hasta pulsar S no se pisa el archivo)
5. `R` rehacer

Si existe `cam_intrinsics.npz`, los clics son sobre la imagen undistorsionada.

### 1c. Comprobar con un AVI de corrida (no recalibra)

```bash
python -m pure_pursuit.replay_calib --self-test
python -m pure_pursuit.replay_calib --avi /ruta/orillasNNNN.avi --save-dir /tmp/bev_replay
```

Izquierda = cámara del HUD, centro = re-warp con la H actual, derecha = BEV grabado.

> Si mueves o cambias el ángulo de la cámara, repite 1b (y 1a si cambias el lente).

---

## 2. Calibrar colores (antes de cada competencia / si cambia la luz)

El detector **no usa rangos RGB**. Usa HSV: conos en `vision.py` (`COLOR_RANGES`),
cinta naranja / rosa / piso en `config.py`. La cámara tiene el balance de blancos
apagado, así que otra luz corre los tonos. `color_corr.py` intenta devolverlos
al color del cuarto de pruebas midiendo el piso; si el piso está quemado no
puede, y hay que medir a mano.

Para el servicio (tiene la cámara tomada):

```bash
sudo systemctl stop wro-runtime && sleep 4
```

Ventana (recomendado): pinta el cono/cinta/pared o pulsa `a` para auto-detectar
por RGB (dominancia de canal, aguanta el cambio de luz). Te imprime HSV listo
para pegar:

```bash
python3 -m pure_pursuit.calibra_luz --gui
```

- `1` rojo  `2` verde  `3` naranja  `4` rosa
- click + arrastrar = muestrear  |  `a` = auto-detectar el color activo
- `p` imprime ese color  |  `s` imprime todos  |  `c` prende/apaga color_corr
- overlay: lo que AGARRAN los rangos actuales vs lo que estás midiendo

Sin pantalla (SSH), con un cono rojo y uno verde enfrente:

```bash
python3 -m pure_pursuit.calibra_luz --vivo --sugerir
```

Sobre una corrida grabada (panel izquierdo del HUD = lo que ve la visión):

```bash
python3 -m pure_pursuit.calibra_luz --avi videos_orillas/orillasNNNN.avi --sugerir
```

Dónde pegar lo que imprime:

| Color | Archivo |
|---|---|
| rojo / verde | `vision.py` → `COLOR_RANGES` (y `LINE_CONE_HSV` en `config.py`) |
| naranja (cinta) | `config.py` → `LINE_ORANGE_HSV` / `LINE_CORE_HSV` |
| rosa (estacionamiento) | `config.py` → `PARK_PINK_HSV` |
| piso BGR | `config.py` → `COLOR_CORR_FLOOR_REF_BGR` |

Después: `sudo systemctl start wro-runtime` (nunca `restart`: deja la CSI colgada).

---

## 3. Probar solo la visión (sin robot, sin ESP32)

Útil para verificar que la BEV y la centerline se ven bien antes de conectar todo.

```bash
python -m pure_pursuit.test_vision
```

Con un video grabado en lugar de cámara en vivo:
```bash
python -m pure_pursuit.test_vision --video ruta/al/video.mp4
```

Lo que deberías ver:
- Ventana izquierda: cámara con los obstáculos detectados (rojo/verde marcados)
- Ventana derecha: BEV con la centerline en cyan, punto look-ahead en amarillo, robot en naranja
- Texto arriba: `pp=ON`, ángulo de dirección, cantidad de puntos del path

Presiona `S` en cualquier momento para guardar un screenshot.
Presiona `ESC` para salir.

---

## 4. Correr en el robot real

### Requisitos previos
- `bev_calib.npz` generado (paso 1)
- ESP32 con `PurePursuit.ino` ya flasheado
- Pi conectada al ESP32 por Serial2 (RX=17, TX=16)

### Comando

```bash
python -m pure_pursuit.runtime
```

### Secuencia de arranque automática
1. LED en GPIO27 enciende → Pi lista
2. Espera que presiones el botón en GPIO17
3. Calienta la cámara ~2 segundos
4. Manda `READY` al ESP32 → ESP32 responde y arranca motores
5. Loop: cámara → BEV → centerline → Pure Pursuit → serial al ESP32

### Flags útiles

```bash
# Sin ventana (en competencia sin monitor)
python -m pure_pursuit.runtime --no-window

# Grabar video de lo que ve la cámara
python -m pure_pursuit.runtime --no-window --record-orillas

# Puerto serial diferente (si usas adaptador USB)
python -m pure_pursuit.runtime --serial-port /dev/ttyUSB0
```

---

## Resumen de archivos

| Archivo | Para qué |
|---------|----------|
| `calibrate.py` | Calibrar la homografía BEV con la cámara real |
| `calibrate_intrinsics.py` | Chessboard → K + distorsión (`cam_intrinsics.npz`) |
| `replay_calib.py` | Self-test sintético / re-warp del HUD de una corrida |
| `test_vision.py` | Probar visión + centerline sin robot ni serial |
| `runtime.py` | Runtime completo para competencia |
| `config.py` | Todos los parámetros ajustables (HSV, lookahead, gains) |
| `bev_calib.npz` | Archivo de calibración generado por calibrate.py |
