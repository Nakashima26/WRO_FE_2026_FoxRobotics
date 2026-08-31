"""
Detección de líneas de esquina (naranja / azul) en la imagen BEV.

Recuperado del historial (commit 2ab26e2 ".ino updated + test turns",
2026-08-17), pero con un método de localización distinto: la versión
original tomaba el Y máximo de TODOS los pixeles del color en todo el
frame — un puñado de ruido disperso (que nunca forma una franja real)
contaba igual que la línea de verdad, y si el ruido caía más cerca del
robot que la línea real, ganaba el ruido.

Método actual: recorre el BEV fila por fila desde el robot hacia adelante
y se queda con la PRIMERA fila que tenga una corrida CONTIGUA de ese color
de al menos LINE_MIN_RUN_PX — exige que sea una franja real en esa fila
(no puntos sueltos), y como se evalúa fila por fila (no un bounding box de
todo el blob), una línea curva o en diagonal no se penaliza por su altura
total como sí le pasaba al enfoque de bounding box.

Convención BEV (ver bev.py): Y crece hacia ABAJO (hacia el robot). Un punto
con Y grande está CERCA del robot; Y chico está LEJOS (adelante).
"""

import cv2
import numpy as np

from . import config as C


def _line_mask(bev_hsv: np.ndarray, ranges) -> np.ndarray:
    masks = [cv2.inRange(bev_hsv, lo, hi) for lo, hi in ranges]
    mask = np.bitwise_or.reduce(masks) if len(masks) > 1 else masks[0]
    # Cierre morfológico: puentea huecos de 1-3 px (oclusión parcial por un
    # obstáculo, sombras) para que un segmento real siga formando una corrida
    # contigua >= LINE_MIN_RUN_PX -- sin esto 'seen'/near_y brincan entre ese
    # segmento y otro más lejano por un par de pixeles de diferencia frame a frame.
    k = cv2.getStructuringElement(cv2.MORPH_RECT, C.LINE_MASK_CLOSE_KERNEL)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)


def _row_max_runs(mask: np.ndarray) -> np.ndarray:
    """
    Longitud de la corrida contigua de pixeles>0 más larga, UNA POR FILA,
    para toda la máscara de una sola vez (sin loop de Python fila por fila).

    Truco estándar de "run-length vectorizado": running = cumsum de 1's que
    se resetea a 0 en cada 0; reset_at = el último valor de running antes de
    cada reset, propagado hacia adelante con maximum.accumulate; la longitud
    de la corrida en curso en cada posición es running - reset_at, y el
    máximo por fila es el resultado que antes se calculaba con
    np.where/np.diff/np.split fila por fila (caro en Python puro sobre
    hasta 400 filas, justo el caso más común: línea no visible).
    """
    a = (mask > 0).astype(np.int32)
    running = a.cumsum(axis=1)
    reset_at = np.where(a == 0, running, 0)
    reset_at = np.maximum.accumulate(reset_at, axis=1)
    return (running - reset_at).max(axis=1)


def _find_near_line_row(mask: np.ndarray, min_run_px: int) -> float | None:
    """
    Retorna la Y (más cercana al robot, Y grande) de la primera fila con una
    corrida contigua >= min_run_px, escaneando desde el robot hacia adelante.
    None si ninguna fila califica.
    """
    max_runs = _row_max_runs(mask)
    qualifying = np.where(max_runs >= min_run_px)[0]
    if qualifying.size == 0:
        return None
    return float(qualifying.max())


def _fit_line_near(mask: np.ndarray, near_y: float, band_px: float,
                    min_points: int) -> tuple[float, float, float, float] | None:
    """
    Ajusta una recta (vx, vy, x0, y0) a los pixeles de la máscara dentro de
    una banda de +-band_px alrededor de near_y, cubriendo todo el ancho de
    la imagen — a diferencia de _find_near_line_row(), que solo mira una
    fila, esto junta pixeles de varias filas para poder estimar la
    PENDIENTE real de la línea (puede venir inclinada, no necesariamente
    horizontal).

    Retorna None si no hay suficientes pixeles en la banda (línea muy
    ocluida/fragmentada ahí) — en ese caso, quien llame debe usar un
    fallback horizontal en near_y en vez de una recta con pendiente.
    """
    h = mask.shape[0]
    y0b = max(0, int(near_y - band_px))
    y1b = min(h, int(near_y + band_px) + 1)
    band = mask[y0b:y1b, :]
    ys, xs = np.where(band > 0)
    if len(xs) < min_points:
        return None
    pts = np.column_stack([xs.astype(np.float32), (ys + y0b).astype(np.float32)])
    # DIST_HUBER (no DIST_L2): mínimos cuadrados le da todo el peso a los
    # outliers -- unos pocos pixeles de ruido lejos del eje de la línea bastaban
    # para torcer la pendiente de un frame al siguiente.
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).flatten()
    # Descarta ajustes demasiado inclinados: la línea de esquina en BEV es
    # ~perpendicular al avance (casi horizontal). Una pendiente empinada casi
    # siempre es ruido / pixeles de otro segmento -> que el caller use el
    # fallback horizontal plano en near_y.
    ang = abs(float(np.degrees(np.arctan2(float(vy), float(vx)))))
    ang = min(ang, 180.0 - ang)          # 0 = horizontal, 90 = vertical
    if ang > C.LINE_FIT_MAX_SLOPE_DEG:
        return None
    return float(vx), float(vy), float(x0), float(y0)


def line_side_is_near(px: float, py: float,
                       line_params: tuple[float, float, float, float],
                       ref_x: float, ref_y: float) -> bool:
    """
    True si el punto (px, py) está del MISMO lado de la recta (vx,vy,x0,y0)
    que el punto de referencia (ref_x, ref_y) — normalmente el robot, cuyo
    lado por definición es "antes de la línea" (mi recta).

    Usa el signo del producto cruzado (dirección de la recta) x (vector al
    punto) — no importa la orientación de (vx,vy) que devuelva cv2.fitLine
    porque el signo se calibra contra la referencia, no contra un eje fijo.
    """
    vx, vy, x0, y0 = line_params
    cross_ref = vx * (ref_y - y0) - vy * (ref_x - x0)
    cross_pt  = vx * (py - y0) - vy * (px - x0)
    return (cross_ref >= 0) == (cross_pt >= 0)


def detect_lines(bev_bgr: np.ndarray, bev_hsv: np.ndarray | None = None) -> dict:
    """
    Retorna {'seen': bool, 'near_y': float|None} para la línea naranja.
    near_y = coordenada Y-BEV de la franja real más cercana al robot
    (no el pixel aislado más cercano — ver _find_near_line_row()).

    Azul se quitó de la ecuación (daba muchos falsos positivos/negativos y
    no era confiable) — por ahora solo se seguirá la línea naranja.

    Lectura CRUDA de un solo frame — brinca entre un segmento cercano
    parcialmente ocluido por un obstáculo (a veces cruza LINE_MIN_RUN_PX,
    a veces no, por un par de pixeles de diferencia frame a frame) y el
    siguiente segmento realmente visible más lejos. Para uso en el
    runtime real, usar OrangeLineTracker (abajo), que suaviza esto en
    el tiempo.

    bev_hsv: conversión HSV de bev_bgr ya calculada, si el caller ya la tiene
    (evita convertir la misma imagen BGR->HSV más de una vez por frame — ver
    runtime_nuevo.py, que también se la pasa a detect_centerline()). Si no
    se pasa, se calcula aquí como antes.
    """
    hsv = bev_hsv if bev_hsv is not None else cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
    mask = _line_mask(hsv, C.LINE_ORANGE_HSV)
    near_y = _find_near_line_row(mask, C.LINE_MIN_RUN_PX)
    return {"Orange": {"seen": near_y is not None, "near_y": near_y}}


class OrangeLineTracker:
    """
    Suaviza detect_lines() en el tiempo.

    La lectura cruda de un solo frame puede "brincar" entre dos segmentos
    reales distintos (uno cercano parcialmente ocluido por un obstáculo,
    otro más lejos y despejado) de un frame a otro — confirmado en pista:
    la Y reportada saltaba entre "muy pegada al obstáculo" y "bien
    separada" sin que el robot se moviera lo suficiente para justificarlo.

    En vez de confiar en la lectura de un solo frame, exige que una nueva
    lectura (mismo 'seen' y near_y dentro de una tolerancia) se repita
    PERSIST_FRAMES seguidos antes de aceptarla como el estado "real" — un
    brinco de un solo frame no alcanza a mover el valor reportado.

    Encima de eso: (1) el near_y aceptado se mezcla por EMA con el anterior
    (rápido si la línea se acerca, lento si se aleja); (2) los extremos de la
    recta con pendiente también se suavizan por EMA -> la diagonal deja de
    "bailar"; (3) si 'seen' se pierde, la última línea se mantiene hold_frames
    frames antes de soltarla, para absorber dropouts cortos de la máscara.
    """

    def __init__(self, persist_frames: int = C.LINE_TRACK_PERSIST_FRAMES,
                 tolerance_px: float = C.LINE_TRACK_TOLERANCE_PX,
                 hold_frames: int = C.LINE_TRACK_HOLD_FRAMES,
                 near_y_ema: float = C.LINE_TRACK_NEAR_Y_EMA,
                 line_ema: float = C.LINE_TRACK_LINE_EMA):
        self.persist_frames = persist_frames
        self.tolerance_px = tolerance_px
        self.hold_frames = hold_frames
        self.near_y_ema = near_y_ema
        self.line_ema = line_ema
        self.stable: dict = {"seen": False, "near_y": None, "line": None}
        self._candidate: dict | None = None
        self._candidate_count = 0
        self._lost_count = 0

    def reset(self):
        self.stable = {"seen": False, "near_y": None, "line": None}
        self._candidate = None
        self._candidate_count = 0
        self._lost_count = 0

    def _matches_candidate(self, raw: dict) -> bool:
        if self._candidate is None or raw["seen"] != self._candidate["seen"]:
            return False
        if not raw["seen"]:
            return True
        return abs(raw["near_y"] - self._candidate["near_y"]) <= self.tolerance_px

    def update(self, bev_bgr: np.ndarray, bev_hsv: np.ndarray | None = None) -> dict:
        """
        bev_hsv: ver detect_lines() -- si el caller ya convirtió bev_bgr a
        HSV este frame (runtime_nuevo.py lo hace para compartirla con
        detect_centerline()), pásala aquí para no repetir la conversión.
        """
        if bev_hsv is None:
            bev_hsv = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
        raw = detect_lines(bev_bgr, bev_hsv=bev_hsv)["Orange"]

        if self._matches_candidate(raw):
            self._candidate_count += 1
            # Refrescar el candidato a la lectura MÁS reciente (dentro de
            # tolerancia): el valor que se acepta al llegar a persist_frames
            # debe ser el actual, no el primero del tramo -- si no, tras varios
            # frames acercándose quedaba hasta tolerance_px desfasado.
            if raw["seen"]:
                self._candidate = raw
        else:
            self._candidate = raw
            self._candidate_count = 1

        if self._candidate_count >= self.persist_frames:
            self._apply_candidate(dict(self._candidate), bev_hsv,
                                  bev_bgr.shape[1])

        return self.stable

    def _apply_candidate(self, cand: dict, bev_hsv: np.ndarray, w: int) -> None:
        """Acepta la lectura persistida, suavizándola contra el estado previo."""
        if not cand["seen"]:
            # 'seen' se perdió de forma persistente. No soltar la línea de
            # golpe: un dropout corto de la máscara no debe tumbarla. Solo tras
            # hold_frames frames seguidos así se da por perdida.
            if self.stable["seen"]:
                self._lost_count += 1
                if self._lost_count >= self.hold_frames:
                    self.stable = {"seen": False, "near_y": None, "line": None}
                    self._lost_count = 0
            return

        self._lost_count = 0
        raw_ny = float(cand["near_y"])
        if self.stable["seen"] and self.stable["near_y"] is not None:
            prev_ny = float(self.stable["near_y"])
            # Y-BEV crece hacia el robot: near_y creciente => el corredor se
            # acerca (dato para frenar/clasificar) => seguir rápido. Si se
            # aleja, suele ser ruido / salto a un segmento más lejano => lento.
            a = 0.7 if raw_ny > prev_ny else self.near_y_ema
            new_ny = a * raw_ny + (1.0 - a) * prev_ny
        else:
            new_ny = raw_ny

        mask = _line_mask(bev_hsv, C.LINE_ORANGE_HSV)
        fitted = _fit_line_near(mask, new_ny, C.LINE_FIT_BAND_PX,
                                C.LINE_FIT_MIN_POINTS)
        self.stable = {
            "seen": True,
            "near_y": new_ny,
            "line": self._smooth_line(fitted, w),
        }

    def _smooth_line(self, fitted, w: int):
        """
        EMA de los extremos de la recta ('y en x=0' / 'y en x=w-1') contra la
        recta estable anterior -- amortigua el vaivén de pendiente frame a
        frame. Se usa esa representación (no vx,vy directos) porque cv2.fitLine
        no fija el signo de (vx,vy). Re-empaqueta como (vx,vy,x0,y0) con x0=0,
        contrato intacto para line_side_is_near()/HUD.
        """
        if fitted is None:
            return None
        vx, vy, x0, y0 = fitted
        if abs(vx) < 1e-6:
            return fitted
        slope = vy / vx
        yl = y0 + (0.0     - x0) * slope
        yr = y0 + (w - 1.0 - x0) * slope

        prev = self.stable.get("line")
        if prev is not None and abs(prev[0]) > 1e-6:
            pslope = prev[1] / prev[0]
            pyl = prev[3] + (0.0     - prev[2]) * pslope
            pyr = prev[3] + (w - 1.0 - prev[2]) * pslope
            yl = self.line_ema * yl + (1.0 - self.line_ema) * pyl
            yr = self.line_ema * yr + (1.0 - self.line_ema) * pyr

        return (float(w - 1.0), float(yr - yl), 0.0, float(yl))

    def classify(self, ox: float, oy: float, robot_x: float, robot_y: float) -> bool | None:
        """
        True si (ox, oy) está del mismo lado que el robot (antes de la
        línea, mi recta); False si está del otro lado (siguiente recta).
        None si no hay línea estable todavía (no se puede clasificar).

        Usa la recta con pendiente cuando se pudo ajustar (self.stable["line"]);
        si no hubo suficientes pixeles para ajustarla (línea muy ocluida/corta
        en este momento), cae de vuelta a comparar solo Y contra near_y —
        funciona igual de bien que antes cuando la línea SÍ es horizontal,
        y es mejor que no clasificar nada.
        """
        if not self.stable["seen"]:
            return None
        line = self.stable["line"]
        if line is not None:
            return line_side_is_near(ox, oy, line, robot_x, robot_y)
        return oy > self.stable["near_y"]


class TurnDirectionTracker:
    """
    Fija la dirección de giro (izquierda/derecha) de la pista. Dos fuentes:

      1. PRIMARIA — signo de la pendiente de la línea naranja (vy de
         `line`): la MISMA línea de esquina se ve con pendiente negativa
         yendo en un sentido y positiva en el otro. vy<0 -> giro derecha,
         vy>0 -> giro izquierda. Disponible apenas la naranja es estable.
      2. RESPALDO — posición lateral de un obstáculo "beyond": si cae a la
         derecha del robot, giro derecha; a la izquierda, giro izquierda.
         Solo se usa si no hay pendiente utilizable (su historial de fijar
         mal — rojo de arranque mal clasificado — rompió el intento viejo de
         interior-pass).

    Igual que OrangeLineTracker, exige que la misma dirección salga
    PERSIST_FRAMES seguidos antes de fijarla — es una decisión demasiado
    importante (si sale mal, el carro gira contra la dirección real) como
    para fijarla de un solo dato ruidoso.

    Una vez fijada (self.direction is not None), NO vuelve a cambiar en
    toda la carrera — mismo criterio que direccionIzquierda/primerGiro en
    el ESP32 (PurePursuit.ino), pero determinado por visión en vez de
    ultrasónicos, y potencialmente antes de llegar físicamente a la
    primera esquina.
    """

    def __init__(self, persist_frames: int = 5):
        self.persist_frames = persist_frames
        self.direction: str | None = None   # "L" o "R"; None = aún no confirmada
        self._candidate: str | None = None
        self._candidate_count = 0

    def reset(self):
        self.direction = None
        self._candidate = None
        self._candidate_count = 0

    def update(self, bev_obstacles_beyond: list[tuple[float, float, str]],
               robot_x: float, line: tuple | None = None) -> str | None:
        if self.direction is not None:
            return self.direction   # ya fija, no se vuelve a evaluar

        # ── Fuente PRIMARIA: signo de la pendiente de la línea naranja ──
        # La MISMA línea de esquina se ve con pendiente de signo OPUESTO según
        # el sentido de vuelta (confirmado en pista 2026-08-31):
        #   vy < 0  (la recta sube de izquierda->derecha en BEV)  -> giro DERECHA
        #   vy > 0                                                 -> giro IZQUIERDA
        # `line` = (vx, vy, x0, y0) de OrangeLineTracker.stable["line"] (ya
        # suavizada por EMA + persistencia). vx es siempre +399, así que el
        # signo de la pendiente == signo de vy. Está disponible apenas la
        # naranja se ve estable — antes que un obstáculo "beyond" con offset
        # lateral suficiente, y sin depender de que haya un obstáculo.
        guess_slope = None
        if line is not None and getattr(C, "LINE_DIR_FROM_SLOPE_ENABLED", True):
            vy = float(line[1])
            dead = float(getattr(C, "LINE_DIR_SLOPE_DEADBAND", 60.0))
            if vy <= -dead:
                guess_slope = "R"
            elif vy >= dead:
                guess_slope = "L"
            # |vy| < dead -> línea casi plana en BEV, no vota (evita latchear
            # de una lectura al borde de horizontal).

        # ── Fuente de RESPALDO: posición lateral de un obstáculo "beyond" ──
        # 2026-08-28: guard de offset mínimo. En pista se fijó "L" con un rojo
        # en x=194 (rx=200, offset 6px = ruido). Un obstáculo en la SIGUIENTE
        # recta cae CLARAMENTE a un lado; pocos px no son señal.
        guess_obs = None
        if bev_obstacles_beyond:
            ox0 = bev_obstacles_beyond[0][0]
            if abs(ox0 - robot_x) >= 40.0:
                guess_obs = "R" if ox0 > robot_x else "L"

        # La pendiente manda cuando está; el obstáculo es solo respaldo (su
        # historial de fijar mal es lo que rompió el intento viejo de
        # interior-pass).
        guess = guess_slope if guess_slope is not None else guess_obs
        if guess is None:
            self._candidate = None
            self._candidate_count = 0
            return None

        if guess == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = guess
            self._candidate_count = 1

        just_fixed = False
        if self._candidate_count >= self.persist_frames and self.direction is None:
            self.direction = self._candidate
            just_fixed = True

        # DEBUG: por qué se elige/fija la dirección (se fijó "L" mal en pista
        # por el rojo del arranque). Log al cambiar candidato, primeros
        # conteos, y al fijar. slope vs obs deja ver si discrepan.
        if just_fixed or self._candidate_count <= 2:
            _vy = None if line is None else round(float(line[1]))
            print(f"[TURNDIR] {'>>> FIJADA ' if just_fixed else ''}"
                  f"cand={self._candidate} x{self._candidate_count}/{self.persist_frames} "
                  f"slope={guess_slope}(vy={_vy}) obs={guess_obs} rx={robot_x:.0f}",
                  flush=True)

        return self.direction


def is_interior_pass(direction: str | None, color: str) -> bool:
    """
    True si pasar este obstáculo (por su color, regla WRO: Rojo->derecha,
    Verde->izquierda) coincide con el lado hacia el que va a girar la
    pista — en ese caso, el giro mismo ya resuelve el paso, no hace falta
    bloquear detectarEsquina() esperando a que Pure Pursuit lo esquive del
    todo. False si no coincide (exterior) o si la dirección aún no se ha
    confirmado (default seguro: tratar como bloqueante, comportamiento de
    siempre).
    """
    if direction is None:
        return False
    if direction == "R":
        return color == "Red"
    return color == "Green"
