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


def _run_lengths(mask: np.ndarray) -> np.ndarray:
    """
    Longitud de la corrida contigua de pixeles>0 que TERMINA en cada posición,
    para toda la máscara de una sola vez (sin loop de Python fila por fila).

    Truco estándar de "run-length vectorizado": running = cumsum de 1's que
    se resetea a 0 en cada 0; reset_at = el último valor de running antes de
    cada reset, propagado hacia adelante con maximum.accumulate; la longitud
    de la corrida en curso en cada posición es running - reset_at. Antes se
    calculaba con np.where/np.diff/np.split fila por fila (caro en Python puro
    sobre hasta 400 filas, justo el caso más común: línea no visible).

    El valor en (i, j) es el largo de la corrida que ACABA en j, así que
    `>= min_run` marca exactamente los FINALES de corrida que califican — eso
    es lo que necesita _find_near_line_col() para saber DÓNDE está la corrida,
    no solo que existe en esa columna.
    """
    a = (mask > 0).astype(np.int32)
    running = a.cumsum(axis=1)
    reset_at = np.where(a == 0, running, 0)
    reset_at = np.maximum.accumulate(reset_at, axis=1)
    return running - reset_at


def _row_max_runs(mask: np.ndarray) -> np.ndarray:
    """Longitud de la corrida contigua más larga, UNA POR FILA."""
    return _run_lengths(mask).max(axis=1)


def _band_slice(h: int, near_y: float, half_px: float) -> tuple[int, int]:
    return (max(0, int(near_y - half_px)), min(h, int(near_y + half_px) + 1))


def _band_px_count(mask: np.ndarray, near_y: float, half_px: float) -> int:
    """Pixeles de la máscara dentro de +-half_px de near_y (toda la fila)."""
    y0, y1 = _band_slice(mask.shape[0], near_y, half_px)
    if y1 <= y0:
        return 0
    return int(np.count_nonzero(mask[y0:y1, :]))


def _core_px_count(bev_hsv: np.ndarray, near_y: float, half_px: float) -> int:
    """
    Pixeles de naranja SATURADO (LINE_CORE_HSV) en la banda de near_y.

    La banda naranja "ancha" (LINE_ORANGE_HSV, S>=85) no distingue la cinta de
    competencia de una marca café/tostada sobre el tapete claro: medido en
    orillas820, los trazos del piso salen H 11-15 / S 86-112 / V 152-168 y la
    cinta real H 10-16 / S 140-198. Con S>=140 la separación es total:
      - frames con línea real cruzando (giros 1-4): core 14..221 px
      - frames de la esquiva abortada (116-120): core 0,0,0,0,3 px
    y la banda ANCHA no servía de filtro ahí (tenía 46-272 px de "masa").
    Solo se evalúa la banda (~40 filas), sin morfología: es una cuenta, no una
    detección, así que no hace falta cerrar huecos.
    """
    y0, y1 = _band_slice(bev_hsv.shape[0], near_y, half_px)
    if y1 <= y0:
        return 0
    n = 0
    for lo, hi in C.LINE_CORE_HSV:
        n += int(np.count_nonzero(cv2.inRange(bev_hsv[y0:y1], lo, hi)))
    return n


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


def _find_near_line_col(mask: np.ndarray, min_run_px: int,
                         min_group_cols: int = 1) -> float | None:
    """
    Y (más cercana al robot) de la línea cuando se ve CASI VERTICAL — el caso
    del giro CCW: la naranja cruza el BEV a ~55-70° y en las filas cercanas al
    robot sólo deja 1-4 px contiguos -> _find_near_line_row() falla ahí y engancha
    una fila MÁS LEJOS (near_y ~40-60px corto), o no engancha nada (frames ciegos
    en la boca de la esquina).

    En vertical el patrón se invierte: hay COLUMNAS con una corrida vertical
    larga. Se transpone la máscara y se reusa el mismo run-length vectorizado por
    "fila" (= columna real). Complementa, no reemplaza, al escaneo por fila:
    detect_lines() se queda con el más cercano de los dos.

    Se devuelve el Y del EXTREMO INFERIOR (más cercano al robot) de una corrida
    que califica — NO el pixel encendido más bajo de esas columnas.
    2026-09-09: eso último era el bug que ponía la línea naranja donde no hay
    línea. Con `ys = mask[:, cols].any(...).max()`, un speck aislado a y=293
    que compartiera COLUMNA con la corrida real (que estaba a y=120-171, 170px
    más lejos) reportaba near_y=293 — línea "pegada al carro" a partir de ruido
    a media pista, con los conos de la recta cayendo del otro lado -> `beyond`
    -> esquiva abandonada (medido en orillas818 ~22:00:27).

    min_group_cols: una línea real casi vertical es una franja de ~10px de ancho
    (20mm / MM_PER_PX) -> deja VARIAS columnas contiguas con corrida larga. Una
    columna suelta (borde de cono, reflejo, dos specks que el cierre morfológico
    unió) no. Solo cuentan los grupos de >= min_group_cols columnas contiguas.
    """
    ends = _run_lengths(mask.T) >= min_run_px      # (col, fila): fin de corrida válida
    cols = np.flatnonzero(ends.any(axis=1))
    if cols.size == 0:
        return None
    if min_group_cols > 1:
        groups = np.split(cols, np.flatnonzero(np.diff(cols) > 1) + 1)
        keep = [g for g in groups if g.size >= min_group_cols]
        if not keep:
            return None
        cols = np.concatenate(keep)
    rows = np.flatnonzero(ends[cols].any(axis=0))
    if rows.size == 0:
        return None
    return float(rows.max())


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
    near_row = _find_near_line_row(mask, C.LINE_MIN_RUN_PX)
    near_col = _find_near_line_col(
        mask, int(getattr(C, "LINE_MIN_COL_RUN_PX", 10)),
        int(getattr(C, "LINE_MIN_COL_GROUP", 1)))

    # GUARDA DE MASA: donde near_y dice que cruza la línea tiene que HABER
    # línea. Una franja real (20mm de cinta = ~10px de alto en BEV, y cruza
    # buena parte del ancho) deja decenas/cientos de px en una banda de
    # +-LINE_BAND_CHECK_PX; un speck de ruido deja <15. Sin esta guarda, la
    # lectura se aceptaba con el vecindario vacío -- justo la firma de la
    # línea falsa de orillas818 (near_y=248->291 y `line: None` 14 frames
    # seguidos, o sea _fit_line_near ni juntaba 35px ahí).
    # Se prueba el candidato MÁS CERCANO primero; si no tiene masa se cae al
    # otro (más lejos) antes de darse por no vista.
    half = float(getattr(C, "LINE_BAND_CHECK_PX", 20.0))
    need = int(getattr(C, "LINE_BAND_MIN_PX", 25))
    need_core = int(getattr(C, "LINE_CORE_MIN_PX", 10))
    near_y, src, band, core = None, None, 0, 0
    rej = None
    for cand, tag in sorted(
            [(v, t) for v, t in ((near_row, "fila"), (near_col, "col")) if v is not None],
            reverse=True):
        n = _band_px_count(mask, cand, half)
        # ...y de esa masa, algo tiene que ser naranja DE VERDAD (saturado).
        # Sin esto, una marca café del tapete pasa la guarda de masa (tenía
        # 46-272 px anchos) y se reporta como línea a 20 cm del carro.
        c = _core_px_count(hsv, cand, half) if n >= need else 0
        if n < need or c < need_core:
            # Se guarda el candidato rechazado MÁS CERCANO para el log: si algún
            # día se deja de ver una línea REAL, aquí se ve por cuánto falló
            # (band/core contra LINE_BAND_MIN_PX / LINE_CORE_MIN_PX).
            if rej is None:
                rej = (round(float(cand)), tag, n, c)
            continue
        near_y, src, band, core = cand, tag, n, c
        break
    out = {"seen": near_y is not None, "near_y": near_y,
           "src": src, "band": band, "core": core}
    if near_y is None and rej is not None:
        out["rej"] = rej
    return {"Orange": out}


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
        # Dead-reckon de near_y cuando la línea se pierde CERCA de la esquina
        # (ver ORANGE_DR_* en config). `_out` es lo que devolvió update() este
        # frame -- puede ser self.stable (real) o una lectura estimada; classify()
        # usa _out, no self.stable.
        self._dr_ny: float | None = None
        self._dr_frames = 0
        self._dr_latched = False   # el ancla estuvo "en la boca" -> no expira hasta reset()
        self._post_turn_cd = 0     # tras reset(): ignora TODA lectura de naranja N frames
        self._out: dict = self.stable

    def reset(self):
        self.stable = {"seen": False, "near_y": None, "line": None}
        self._candidate = None
        self._candidate_count = 0
        self._lost_count = 0
        self._dr_ny = None
        self._dr_frames = 0
        self._dr_latched = False
        # Cooldown post-giro: la línea que se ve recién girado suele ser la que
        # se acaba de pasar (o su residual) -> no clasificar contra ella.
        self._post_turn_cd = int(getattr(C, "ORANGE_POST_TURN_CD_FRAMES", 20))
        self._out = self.stable

    def hold_cooldown(self):
        """Re-arma el cooldown post-giro SIN borrar el estado del tracker.
        El caller lo llama cada frame mientras dura la maniobra de giro: así el
        conteo de ORANGE_POST_TURN_CD_FRAMES empieza a bajar recién cuando la
        maniobra TERMINA, no desde que arranca. La MANIOBRA reversa dura ~3s y al
        retroceder la cámara vuelve a ver la línea del giro recién hecho -> con el
        cooldown armado solo desde el inicio, expiraba a media maniobra y esa
        línea latcheaba (near_y >= ORANGE_DR_LATCH_Y) toda la recta nueva."""
        self._post_turn_cd = max(
            self._post_turn_cd,
            int(getattr(C, "ORANGE_POST_TURN_CD_FRAMES", 20)))

    def _matches_candidate(self, raw: dict) -> bool:
        if self._candidate is None or raw["seen"] != self._candidate["seen"]:
            return False
        if not raw["seen"]:
            return True
        return abs(raw["near_y"] - self._candidate["near_y"]) <= self.tolerance_px

    def update(self, bev_bgr: np.ndarray, bev_hsv: np.ndarray | None = None,
              ds_px: float = 0.0, in_turn_cooldown: bool = False) -> dict:
        """
        bev_hsv: ver detect_lines() -- si el caller ya convirtió bev_bgr a
        HSV este frame (runtime_nuevo.py lo hace para compartirla con
        detect_centerline()), pásala aquí para no repetir la conversión.

        ds_px / in_turn_cooldown: para el dead-reckon de near_y cuando la línea
        se pierde CERCA de la esquina (ver ORANGE_DR_* en config). ds_px = px
        que la línea se acerca al robot este frame (== el ds_px de la memoria).
        """
        # Cooldown post-giro: ignora TODA lectura de naranja estos frames (la que
        # se ve recién girado es la del giro que se acaba de hacer). NO se
        # re-acumula candidato ni se toca el DR.
        if self._post_turn_cd > 0:
            self._post_turn_cd -= 1
            self.stable = {"seen": False, "near_y": None, "line": None}
            self._out = self.stable
            return self._out

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

        # self.stable es SIEMPRE la lectura real del tracker (nunca la estimada).
        dr_min_y = float(getattr(C, "ORANGE_DR_MIN_ANCHOR_Y", 240.0))
        dr_max_f = int(getattr(C, "ORANGE_DR_MAX_FRAMES", 8))
        dr_latch_y = float(getattr(C, "ORANGE_DR_LATCH_Y", 290.0))
        if self.stable["seen"]:
            ny = self.stable["near_y"]
            if ny is not None and ny >= dr_min_y:
                # línea real y CERCA: re-ancla el DR. Si llegó a la "boca"
                # (>= LATCH_Y) queda latcheada -> al perderse NO expira: todo lo
                # que se vea después es siguiente segmento hasta el giro.
                self._dr_ny = float(ny)
                self._dr_latched = (ny >= dr_latch_y)
            else:
                self._dr_ny = None
                self._dr_latched = False
            self._dr_frames = 0
            self._out = self.stable
            return self._out

        # línea NO vista: ¿la mantenemos "marcada"?
        _dr_ok = (self._dr_ny is not None and not in_turn_cooldown
                  and (self._dr_latched or self._dr_frames < dr_max_f))
        if _dr_ok:
            if not self._dr_latched and ds_px > 0.0:
                self._dr_ny += ds_px          # sin latch: marcha unos frames
            self._dr_frames += 1              # latcheada: SIT, solo cuenta
            self._out = {"seen": True, "near_y": self._dr_ny,
                         "line": None, "dead_reckoned": True, "src": "dr"}
            return self._out

        self._dr_ny = None
        self._dr_frames = 0
        self._dr_latched = False
        # Sin línea: si la lectura cruda tenía un candidato rechazado, pasarlo al
        # log (ver `rej` en detect_lines) -- es la única forma de notar que se
        # está descartando una línea real por poco.
        self._out = (dict(self.stable, rej=raw["rej"]) if "rej" in raw
                     else self.stable)
        return self._out

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
            # diag (log [LINEA]): de qué escaneo salió la lectura y cuántos px
            # naranjas hay en su banda. `src=col` + `band` bajo = sospechar ruido.
            "src": cand.get("src"),
            "band": cand.get("band"),
            "core": cand.get("core"),
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
        # _out = lo que devolvió el último update(): la lectura real o la
        # ESTIMADA (dead_reckoned). Sin update() previo, cae a self.stable.
        s = getattr(self, "_out", None) or self.stable
        if not s["seen"]:
            return None
        line = s.get("line")
        if line is not None:
            res = line_side_is_near(ox, oy, line, robot_x, robot_y)
        else:
            res = oy > s["near_y"]
        # La línea ESTIMADA solo puede decir "beyond" (diferir la esquiva),
        # nunca "mía" (hacerla): si se equivoca, el peor caso es no esquivar
        # algo que debía, nunca esquivar en la boca de la esquina.
        if s.get("dead_reckoned") and res is True:
            return None
        return res


class TurnDirectionTracker:
    """
    Fija la dirección de giro (izquierda/derecha) de la pista.

      PRIMARIA — posición lateral de un obstáculo "beyond" (siguiente recta):
        cae a la derecha del eje -> giro derecha; a la izquierda -> izquierda.
        Guard de offset mínimo 40px (un rojo de arranque a 6px fijó "L" mal
        en pista, 2026-08-28).
      CONFIRMACIÓN opcional (apagada por defecto) — signo de la pendiente de
        la naranja. Se probó como primaria y FALLÓ: `vy` sigue a la distancia
        a la línea, no al sentido de giro (osciló +170->-27->+126 en una sola
        aproximación). Si se enciende, solo veta/confirma con la línea cerca.

    Solo se infiere DURANTE la corrida (runtime_nuevo llama con beyond=[] y
    line=None mientras está desarmado, y reset() al armar). El caso crítico es
    el PRIMER giro; en los siguientes, tras el primer GIRANDO del ESP32, ya
    hay obstáculos "beyond" limpios de la recta nueva.

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

    def set_esp_direction(self, d: str) -> None:
        """El ESP32 confirmó su dirección de giro (direccionIzquierda, fijada
        en su 1er GIRANDO con distL>distR y mandada como dir= en el ACK:V2).
        Es AUTORITATIVA: fija o corrige la dirección. Cubre las esquinas 2-12;
        la 1 sigue dependiendo de la estimación por visión (el ESP manda '?'
        hasta que hace su primer giro). El ESP se resetea entre corridas ->
        no llega un dir= viejo."""
        if d not in ("L", "R"):
            return
        if self.direction != d:
            print(f"[TURNDIR] >>> ESP dir={d} (antes {self.direction})", flush=True)
            self.direction = d
            self._candidate = None
            self._candidate_count = 0

    def update(self, bev_obstacles_beyond: list[tuple[float, float, str]],
               robot_x: float, line: tuple | None = None,
               near_y: float | None = None) -> str | None:
        if self.direction is not None:
            return self.direction   # ya fija, no se vuelve a evaluar

        # ── PRIMARIO: posición lateral de un obstáculo "beyond" ──
        # Un obstáculo de la SIGUIENTE recta cae CLARAMENTE a un lado del eje
        # según hacia dónde gira la pista: a la derecha -> giro derecha; a la
        # izquierda -> giro izquierda.
        # Guard de offset mínimo (2026-08-28): en pista se fijó "L" con un rojo
        # a x=194 (rx=200, offset 6px = ruido). Pocos px no son señal.
        guess_obs = None
        if bev_obstacles_beyond:
            ox0 = bev_obstacles_beyond[0][0]
            if abs(ox0 - robot_x) >= 40.0:
                guess_obs = "R" if ox0 > robot_x else "L"

        # ── CONFIRMACIÓN opcional: signo de la pendiente de la naranja ──
        # 2026-08-31: se PROBÓ como fuente primaria y FALLÓ — `vy` sigue a la
        # distancia a la línea (distorsión BEV de campo lejano), no al sentido
        # de giro: en UNA aproximación osciló +170 -> -27 -> +126 y latcheó
        # "L" con la pista girando a la derecha. Queda apagada por defecto
        # (LINE_DIR_FROM_SLOPE_ENABLED=False); si se enciende, solo se lee con
        # la línea CERCA (near_y alto = zona BEV bien calibrada) y solo sirve
        # para CONFIRMAR / vetar al obstáculo, nunca para decidir sola.
        guess_slope = None
        if (line is not None
                and getattr(C, "LINE_DIR_FROM_SLOPE_ENABLED", False)
                and near_y is not None
                and near_y >= getattr(C, "LINE_DIR_MIN_NEAR_Y", 285.0)):
            vy = float(line[1])
            dead = float(getattr(C, "LINE_DIR_SLOPE_DEADBAND", 20.0))
            if vy <= -dead:
                guess_slope = "R"
            elif vy >= dead:
                guess_slope = "L"

        # El obstáculo manda. Si la pendiente está y lo CONTRADICE, no se vota
        # este frame (que se resuelva o que una fuente se caiga) -- así una
        # lectura de pendiente mala no arrastra, pero tampoco puede fijar sola.
        if (guess_obs is not None and guess_slope is not None
                and guess_obs != guess_slope):
            if self._candidate is not None:   # solo al romper un candidato en curso
                print(f"[TURNDIR] conflicto obs={guess_obs} vs slope={guess_slope} "
                      f"-> no vota", flush=True)
            self._candidate = None
            self._candidate_count = 0
            return None
        guess = guess_obs if guess_obs is not None else guess_slope
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

        # DEBUG: por qué se elige/fija la dirección (se fijó mal en pista antes).
        if just_fixed or self._candidate_count <= 2:
            _vy = None if line is None else round(float(line[1]))
            _ny = None if near_y is None else round(float(near_y))
            _ox = None if not bev_obstacles_beyond else round(bev_obstacles_beyond[0][0])
            print(f"[TURNDIR] {'>>> FIJADA ' if just_fixed else ''}"
                  f"cand={self._candidate} x{self._candidate_count}/{self.persist_frames} "
                  f"obs={guess_obs}(ox={_ox}) slope={guess_slope}(vy={_vy},ny={_ny}) "
                  f"rx={robot_x:.0f}", flush=True)

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
