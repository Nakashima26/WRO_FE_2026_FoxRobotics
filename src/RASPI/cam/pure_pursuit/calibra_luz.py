"""
calibra_luz.py — Calibración de COLOR para Pure Pursuit (otra iluminación).

El detector NO usa rangos RGB. Usa HSV (visión de conos en vision.py,
cinta naranja / rosa / piso en config.py). El RGB se imprime solo para
que veas cómo se ve el color; lo que hay que pegar es el HSV.

POR QUÉ EXISTE
  Los rangos se midieron con la luz del cuarto de pruebas y la cámara corre
  con balance de blancos APAGADO y ganancias fijas (vision.py:
  awb-enable=false colour-gains=<1.2,1.5>). Con otra luz TODO se corre y
  los conos / la cinta / el rosa dejan de existir para el carro.

  color_corr.py escala B,G,R para que el PISO quede del color de siempre.
  Si el piso está quemado (V~255) no tiene tono y no corrige nada: entonces
  hay que medir a mano con ESTE script.

CÓMO ENCUENTRA LOS OBJETOS (modo --avi / --vivo, sin clic)
  No busca por HSV (si el tono está corrido no encuentra nada). Busca por
  DOMINANCIA DE CANAL RGB, que aguanta el cambio de luz:
    rojo    = R >> G y R >> B, sin amarillear (separa cono de cinta/piel)
    verde   = G >> R y G >> B
    naranja = rojo que amarillea (bastante G, poco B)
    rosa    = rojizo CON mucho azul (pared magenta del estacionamiento)

USO — desde src/RASPI/cam/

  # 1) Ventana en vivo: pinta el color o pulsa 'a' para auto-detectar.
  #    Misma corrección de piso que el runtime. 1=rojo 2=verde 3=naranja 4=rosa.
  python3 -m pure_pursuit.calibra_luz --gui

  # 2) Sin pantalla (SSH): pon un cono rojo y uno verde enfrente y camina.
  python3 -m pure_pursuit.calibra_luz --vivo --sugerir

  # 3) Sobre una corrida grabada (panel izquierdo del HUD = lo que ve la visión):
  python3 -m pure_pursuit.calibra_luz --avi videos_orillas/orillasNNNN.avi --sugerir

OJO: el servicio wro-runtime tiene tomada la cámara. Para --gui / --vivo:
  sudo systemctl stop wro-runtime && sleep 4
  ... calibrar ...
  sudo systemctl start wro-runtime
  NUNCA `systemctl restart` (deja la CSI colgada).
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_CAM_DIR = os.path.dirname(_HERE)
for _p in (_CAM_DIR, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from . import config as C
    from .bev import BEVTransformer
    from .color_corr import FloorColorCorrector
except ImportError:
    import config as C
    from bev import BEVTransformer
    from color_corr import FloorColorCorrector

from vision import COLOR_RANGES, Vision, open_camera


# ── Cómo se BUSCA cada objetivo (independiente de la luz) ────────────────────
def _mask_rojo(b, g, r):
    """Cono ROJO. Lo que lo separa de la cinta naranja y de una mano/brazo es
    G contra B: el rojo del cono no amarillea (G queda AL NIVEL de B o por
    debajo), la cinta y la piel sí (G bastante por encima de B).
    Medido: cono g-b = -65 .. 0 | cinta +8 | piel +39."""
    return ((r - g > 50) & (r - b > 40) & (r > 55)
            & ((g - b) * 100 < 8 * (r - b))
            & (b * 100 < 42 * np.maximum(r, 1)))       # fuera la pared rosa


def _mask_rosa(b, g, r):
    """Pared magenta del estacionamiento: rojiza pero con MUCHO azul."""
    return (r - g > 40) & (b > g) & (b * 100 >= 42 * np.maximum(r, 1)) & (r > 55)


def _mask_verde(b, g, r):
    return (g - r > 30) & (g - b > 20) & (g > 45)


def _mask_naranja(b, g, r):
    # naranja = rojo que trae bastante verde (amarillea) y poco azul
    return (r - b > 55) & (g - b > 25) & (r - g > 15) & (r > 90)


OBJETIVOS = {
    "rojo":    _mask_rojo,
    "verde":   _mask_verde,
    "naranja": _mask_naranja,
    "rosa":    _mask_rosa,
}

MIN_PX = 600          # blob mínimo para que la medición signifique algo
PATCH_HALF = 10       # radio del parche al pintar en --gui (grande a propósito: en Mac el clic es fácil de perder)

# Dónde se pega cada rango (Pure Pursuit).
DESTINO = {
    "rojo":    "vision.py  COLOR_RANGES['Red']   (+ LINE_CONE_HSV en config.py)",
    "verde":   "vision.py  COLOR_RANGES['Green'] (+ LINE_CONE_HSV en config.py)",
    "naranja": "config.py  LINE_ORANGE_HSV  (cinta de esquina, se aplica en BEV)",
    "rosa":    "config.py  PARK_PINK_HSV",
    "piso":    "config.py  COLOR_CORR_FLOOR_REF_BGR  (BGR del piso, no HSV)",
}

# Espacio en el que el runtime aplica el rango.
ESPACIO = {
    "rojo":    "cam",
    "verde":   "cam",
    "rosa":    "cam",
    "naranja": "bev",
    "piso":    "cam",
}

OVERLAY_BGR = {
    "rojo":    (40, 40, 255),
    "verde":   (40, 220, 40),
    "naranja": (0, 140, 255),
    "rosa":    (180, 50, 220),
    "piso":    (200, 200, 200),
}


def _rangos_prod() -> dict:
    """Rangos ACTUALES del código, no una copia que se queda vieja."""
    return {
        "rojo":    [(tuple(int(x) for x in lo), tuple(int(x) for x in hi))
                    for lo, hi in COLOR_RANGES["Red"]],
        "verde":   [(tuple(int(x) for x in lo), tuple(int(x) for x in hi))
                    for lo, hi in COLOR_RANGES["Green"]],
        "naranja": [(tuple(int(x) for x in lo), tuple(int(x) for x in hi))
                    for lo, hi in C.LINE_ORANGE_HSV],
        "rosa":    [(tuple(int(x) for x in lo), tuple(int(x) for x in hi))
                    for lo, hi in C.PARK_PINK_HSV],
        "piso":    [(tuple(int(x) for x in C.FLOOR_LOWER),
                     tuple(int(x) for x in C.FLOOR_UPPER))],
    }


def _fmt_ranges(rangos) -> str:
    parts = []
    for lo, hi in rangos:
        parts.append(
            f"(np.array([{lo[0]}, {lo[1]}, {lo[2]}]), "
            f"np.array([{hi[0]}, {hi[1]}, {hi[2]}]))"
        )
    return "[" + ", ".join(parts) + "]"


def _mask_hsv(hsv, rangos) -> np.ndarray:
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in rangos:
        m = cv2.bitwise_or(
            m, cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        )
    return m


class Acumulador:
    """Junta los px de un objetivo a lo largo de muchos frames."""

    def __init__(self, nombre):
        self.nombre = nombre
        self.hist_h = np.zeros(180, np.int64)
        self.hist_s = np.zeros(256, np.int64)
        self.hist_v = np.zeros(256, np.int64)
        self.hist_b = np.zeros(256, np.int64)
        self.hist_g = np.zeros(256, np.int64)
        self.hist_r = np.zeros(256, np.int64)
        self.br = []         # mediana de B/R por frame (separa cono de pared rosa)
        self.frames = 0
        self.px = 0

    def reset(self):
        self.__init__(self.nombre)

    def add(self, hsv, bgr, m):
        n = int(np.count_nonzero(m))
        if n < MIN_PX:
            return False
        sel = m.astype(bool) if m.dtype != bool else m
        self.hist_h += np.bincount(hsv[..., 0][sel], minlength=180)
        self.hist_s += np.bincount(hsv[..., 1][sel], minlength=256)
        self.hist_v += np.bincount(hsv[..., 2][sel], minlength=256)
        self.hist_b += np.bincount(bgr[..., 0][sel], minlength=256)
        self.hist_g += np.bincount(bgr[..., 1][sel], minlength=256)
        self.hist_r += np.bincount(bgr[..., 2][sel], minlength=256)
        b = bgr[..., 0][sel].astype(np.float32)
        r = bgr[..., 2][sel].astype(np.float32)
        self.br.append(float(np.median(b / np.maximum(r, 1.0))))
        self.frames += 1
        self.px += n
        return True

    def add_patch(self, hsv_patch, bgr_patch):
        """Parche chico al pintar: no exige MIN_PX."""
        if hsv_patch.size == 0:
            return
        h = hsv_patch.reshape(-1, 3)
        b = bgr_patch.reshape(-1, 3)
        self.hist_h += np.bincount(h[:, 0], minlength=180)
        self.hist_s += np.bincount(h[:, 1], minlength=256)
        self.hist_v += np.bincount(h[:, 2], minlength=256)
        self.hist_b += np.bincount(b[:, 0], minlength=256)
        self.hist_g += np.bincount(b[:, 1], minlength=256)
        self.hist_r += np.bincount(b[:, 2], minlength=256)
        bb = b[:, 0].astype(np.float32)
        rr = b[:, 2].astype(np.float32)
        self.br.append(float(np.median(bb / np.maximum(rr, 1.0))))
        self.frames += 1
        self.px += int(h.shape[0])

    @staticmethod
    def _pct(hist, qs):
        tot = hist.sum()
        if tot == 0:
            return [0] * len(qs)
        acc = np.cumsum(hist)
        return [int(np.searchsorted(acc, tot * q / 100.0)) for q in qs]

    def h_circular(self, qs=(2, 50, 98)):
        """Percentiles de H tolerando el wrap 179->0 (el rojo vive a caballo)."""
        hist = self.hist_h
        if hist.sum() == 0:
            return [0] * len(qs), 0
        suave = np.convolve(np.r_[hist, hist], np.ones(9), "same")[:180]
        corte = int(np.argmin(suave))
        rot = np.roll(hist, -corte)
        vals = self._pct(rot, qs)
        return [(v + corte) % 180 for v in vals], corte

    def resumen(self):
        (h2, h50, h98), corte = self.h_circular()
        s2, s50, s98 = self._pct(self.hist_s, (2, 50, 98))
        v2, v50, v98 = self._pct(self.hist_v, (2, 50, 98))
        b2, b50, b98 = self._pct(self.hist_b, (2, 50, 98))
        g2, g50, g98 = self._pct(self.hist_g, (2, 50, 98))
        r2, r50, r98 = self._pct(self.hist_r, (2, 50, 98))
        br = float(np.median(self.br)) if self.br else float("nan")
        return dict(
            h=(h2, h50, h98), s=(s2, s50, s98), v=(v2, v50, v98),
            b=(b2, b50, b98), g=(g2, g50, g98), r=(r2, r50, r98),
            br=br, corte=corte, frames=self.frames, px=self.px,
        )

    def banda_h(self, margen=4):
        """1 o 2 tramos [lo,hi] de H que cubren p2..p98 (wrap 179->0)."""
        (h2, _, h98), _corte = self.h_circular()
        lo = (h2 - margen) % 180
        hi = (h98 + margen) % 180
        if lo <= hi:
            return [(lo, hi)]
        return [(0, hi), (lo, 179)]

    def sugerido(self, margen_h=4, margen_s=20, margen_v=20):
        if self.px == 0:
            return []
        rs = self.resumen()
        s_lo = max(0, rs["s"][0] - margen_s)
        v_lo = max(0, rs["v"][0] - margen_v)
        v_hi = min(255, rs["v"][2] + 25)
        return [((lo, s_lo, v_lo), (hi, 255, v_hi)) for lo, hi in self.banda_h(margen_h)]


def _pasa(hsv, m, rangos):
    """Fracción del blob que cae en los rangos de producción, y por tope H/S/V.

    H/S/V se miden en la BANDA que más px cubre (no se mezclan topes de
    dos tramos distintos: el rojo tiene banda 0-5 y banda 168-179).
    """
    sel = m.astype(bool) if getattr(m, "dtype", None) != bool else m
    H = hsv[..., 0][sel].astype(np.int16)
    S = hsv[..., 1][sel].astype(np.int16)
    V = hsv[..., 2][sel].astype(np.int16)
    if H.size == 0 or not rangos:
        return 0.0, 0.0, 0.0, 0.0
    ok = np.zeros(H.shape, bool)
    best = None
    for lo, hi in rangos:
        okh = (H >= lo[0]) & (H <= hi[0])
        oks = (S >= lo[1]) & (S <= hi[1])
        okv = (V >= lo[2]) & (V <= hi[2])
        full = okh & oks & okv
        ok |= full
        score = (float(full.mean()), float(okh.mean()) + float(oks.mean()) + float(okv.mean()),
                 float(okh.mean()), float(oks.mean()), float(okv.mean()))
        if best is None or score > best:
            best = score
    return float(ok.mean()), best[2], best[3], best[4]


# ── Fuentes de frames ────────────────────────────────────────────────────────
def frames_avi(path):
    """Panel IZQUIERDO del HUD = processed_frame (lo que ve process_frame)."""
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f"[calibra] no se pudo abrir {path}")
    k = 0
    while True:
        ok, f = cap.read()
        if not ok:
            break
        k += 1
        yield k, f[:, :640]
    cap.release()


def frames_bin(path):
    """orillasNNN_bev.bin = [u32 len][u32 n][len bytes JPEG]."""
    with open(path, "rb") as f:
        while True:
            h = f.read(8)
            if len(h) < 8:
                break
            ln, n = struct.unpack("<II", h)
            d = f.read(ln)
            img = cv2.imdecode(np.frombuffer(d, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                yield n, img


def _linea_piso(img):
    """Color del piso en la franja de abajo + aviso de sobreexposición."""
    h = img.shape[0]
    roi = img[int(h * 0.70)::4, ::4]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    piso = (hsv[..., 1] <= 90) & (hsv[..., 2] >= 60) & (hsv[..., 2] <= 250)
    frac = float(piso.mean())
    med = (np.median(roi[piso].reshape(-1, 3), axis=0) if piso.any()
           else np.array([0, 0, 0]))
    quemado = float((cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[..., 2] >= 250).mean())
    return med, frac, quemado


def _channels(img):
    return (img[..., 0].astype(np.int16),
            img[..., 1].astype(np.int16),
            img[..., 2].astype(np.int16))


def _print_objetivo(nom, a: Acumulador, prod_cov, sugerir: bool, rangos_prod):
    if a.frames == 0 or a.px == 0:
        print(f"\n{nom.upper()}: no se vio en ningun frame")
        print(f"  pegar en {DESTINO[nom]}")
        return
    rs = a.resumen()
    print(f"\n{nom.upper()}: {a.frames} frames, {a.px} px")
    print(f"  pegar en {DESTINO[nom]}")
    print(f"  HSV  H p2/med/p98 = {rs['h'][0]:3d} / {rs['h'][1]:3d} / {rs['h'][2]:3d}"
          f"   S = {rs['s'][0]:3d} / {rs['s'][1]:3d} / {rs['s'][2]:3d}"
          f"   V = {rs['v'][0]:3d} / {rs['v'][1]:3d} / {rs['v'][2]:3d}")
    print(f"  RGB  R p2/med/p98 = {rs['r'][0]:3d} / {rs['r'][1]:3d} / {rs['r'][2]:3d}"
          f"   G = {rs['g'][0]:3d} / {rs['g'][1]:3d} / {rs['g'][2]:3d}"
          f"   B = {rs['b'][0]:3d} / {rs['b'][1]:3d} / {rs['b'][2]:3d}"
          f"   (informativo: el código usa HSV, no RGB)")
    extra = ""
    if nom == "rojo":
        extra = "   (cono real: <=0.35; pared rosa del estacionamiento: >=0.45)"
    print(f"  B/R mediano = {rs['br']:.2f}{extra}")
    if nom in rangos_prod and prod_cov is not None and prod_cov[4]:
        ok, ph, ps, pv = (prod_cov[0] / prod_cov[4] * 100, prod_cov[1] / prod_cov[4] * 100,
                          prod_cov[2] / prod_cov[4] * 100, prod_cov[3] / prod_cov[4] * 100)
        print(f"  con los rangos ACTUALES pasa el {ok:.0f}% de esos px", end="")
        if ok < 40:
            culpa = min((("H", ph), ("S", ps), ("V", pv)), key=lambda t: t[1])
            print(f"  <-- el tope que lo tira es {culpa[0]} "
                  f"(solo {culpa[1]:.0f}% lo pasa; H={ph:.0f}% S={ps:.0f}% V={pv:.0f}%)")
        else:
            print("  (ok)")
    if sugerir and a.px:
        sug = a.sugerido()
        print(f"  SUGERIDO HSV -> {_fmt_ranges(sug)}")
        if nom == "naranja":
            core_s = max(140, rs["s"][1] - 10)
            cores = [((lo[0], core_s, lo[2]), (hi[0], 255, hi[2])) for lo, hi in sug]
            print(f"  LINE_CORE_HSV  -> {_fmt_ranges(cores)}   "
                  f"(mismo H, S mas alto: nucleo saturado de la cinta)")


def _print_piso(piso_acc, quemado_acc):
    if quemado_acc:
        q = float(np.mean(quemado_acc)) * 100
        print(f"piso/exposicion: {q:.1f}% de los px estan QUEMADOS (V>=250)", end="")
        if q > 10:
            print("  <-- DEMASIADO: un piso quemado no tiene tono y color_corr.py")
            print("                 no puede calcular ganancias. Baja la exposicion")
            print("                 de la camara antes de calibrar nada mas.")
        else:
            print("  (ok)")
    if piso_acc:
        pm = np.median(np.array(piso_acc), axis=0)
        print(f"piso BGR medido = ({pm[0]:.0f}, {pm[1]:.0f}, {pm[2]:.0f})")
        print(f"  -> si esta sede es la definitiva: "
              f"COLOR_CORR_FLOOR_REF_BGR = ({pm[0]:.0f}.0, {pm[1]:.0f}.0, {pm[2]:.0f}.0)")
    else:
        print("piso: NO se pudo medir (franja de abajo demasiado oscura o quemada)")
        print("  -> color_corr.py va a saltarse todas las actualizaciones aqui "
              "(frac_piso=0.00 en el log [COLOR])")


def _print_cone_sync_note():
    print("\nSi cambias Red/Green/rosa, LINE_CONE_HSV en config.py debe cubrir")
    print("los mismos tonos (borra px de cono/pared de la cinta naranja).")


# ── Análisis batch (SSH / AVI) ───────────────────────────────────────────────
def analizar(it, cada: int, vivo: bool, sugerir: bool):
    rangos_prod = _rangos_prod()
    acc = {k: Acumulador(k) for k in OBJETIVOS}
    prod = {k: [0.0, 0.0, 0.0, 0.0, 0] for k in rangos_prod}
    piso_acc, quemado_acc, nfr = [], [], 0
    t_log = time.monotonic()

    try:
        for n, img in it:
            if n % cada:
                continue
            nfr += 1
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            b, g, r = _channels(img)
            for nom, fn in OBJETIVOS.items():
                m = fn(b, g, r)
                if not acc[nom].add(hsv, img, m):
                    continue
                if nom in rangos_prod:
                    p = _pasa(hsv, m, rangos_prod[nom])
                    d = prod[nom]
                    for i in range(4):
                        d[i] += p[i]
                    d[4] += 1
            med, frac, quem = _linea_piso(img)
            if frac > 0.2:
                piso_acc.append(med)
            quemado_acc.append(quem)

            if vivo and time.monotonic() - t_log > 1.0:
                t_log = time.monotonic()
                rs = acc["rojo"].resumen()
                d = prod["rojo"]
                pct = (d[0] / d[4] * 100) if d[4] else 0.0
                print(f"[cal] f={nfr} rojo: H{rs['h']} S{rs['s']} V{rs['v']} "
                      f"B/R={rs['br']:.2f} vistos={rs['frames']} "
                      f"-> pasan rango actual {pct:.0f}%", flush=True)
    except KeyboardInterrupt:
        print("\n[cal] interrumpido")

    print(f"\n=== {nfr} frames analizados ===")
    _print_piso(piso_acc, quemado_acc)
    for nom in ("rojo", "verde", "naranja", "rosa"):
        _print_objetivo(nom, acc[nom], prod.get(nom), sugerir, rangos_prod)
    if sugerir:
        _print_cone_sync_note()
        print("\nNota: los rangos sugeridos cubren p2..p98 de lo MEDIDO aqui. Antes de")
        print("pegarlos, vuelve a correr este script sobre una run VIEJA (luz del")
        print("cuarto de pruebas) para confirmar que no pierdes nada alla.")


# ── GUI ──────────────────────────────────────────────────────────────────────
def _tint(disp, mask, color, alpha=0.45):
    if mask is None or not np.any(mask):
        return
    sel = mask > 0 if mask.dtype != bool else mask
    overlay = disp[sel].astype(np.float32)
    col = np.array(color, np.float32)
    disp[sel] = (overlay * (1.0 - alpha) + col * alpha).astype(np.uint8)


def _prepare_frame(raw, color_corr, use_corr, want_bev, bev):
    """Misma corrección que runtime_nuevo: color_corr sobre el frame de cámara,
    luego warp BEV si se pide (la naranja se detecta en BEV)."""
    img = color_corr.process(raw) if (use_corr and color_corr is not None) else raw
    if want_bev:
        if bev is None or not bev.is_calibrated:
            return img, False
        return bev.warp(img), True
    return img, False


def run_gui(get_raw, cam_index, use_corr_0, force_bev, image_mode: bool,
            start_frozen: bool = False):
    rangos_prod = _rangos_prod()
    acc = {k: Acumulador(k) for k in OBJETIVOS}
    color_corr = FloorColorCorrector() if use_corr_0 else None
    bev = BEVTransformer()
    vision = Vision(cam_index, open_cam=False)

    targets = ["rojo", "verde", "naranja", "rosa"]
    tgt = "rojo"
    use_corr = use_corr_0
    overlay_mode = "off"      # off | prod | sug | both  (off = solo se ve TU pintura)
    show_bboxes = False
    dragging = False
    mouse = (0, 0)
    frozen = None             # frame congelado (útil con --image / 'f')
    nfr = 0
    _first = True
    single_step = False
    paint = {k: None for k in OBJETIVOS}   # máscara de lo que YA pintaste
    _cb_ok = False

    win = "calibra_luz"
    win_m = "calibra_luz mascara"
    # AUTOSIZE: 1 px imagen = 1 px ventana. WINDOW_NORMAL en Mac/Retina
    # descuadra el mouse y el clic "no hace nada".
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(win_m, cv2.WINDOW_AUTOSIZE)

    state = {"hsv": None, "img": None}

    def _xy_img(x, y):
        img = state["img"]
        if img is None:
            return x, y
        h, w = img.shape[:2]
        if 0 <= x < w and 0 <= y < h:
            return x, y
        try:
            _wx, _wy, ww, wh = cv2.getWindowImageRect(win)
            if ww > 0 and wh > 0:
                x = int(round(x * w / ww))
                y = int(round(y * h / wh))
        except Exception:
            pass
        if x >= w or y >= h:
            x, y = x // 2, y // 2
        return int(np.clip(x, 0, w - 1)), int(np.clip(y, 0, h - 1))

    def _sample_at(x, y, verbose=False):
        hsv, img = state["hsv"], state["img"]
        if hsv is None or img is None:
            return 0
        x, y = _xy_img(x, y)
        h, w = hsv.shape[:2]
        x0, x1 = max(0, x - PATCH_HALF), min(w, x + PATCH_HALF + 1)
        y0, y1 = max(0, y - PATCH_HALF), min(h, y + PATCH_HALF + 1)
        if x1 <= x0 or y1 <= y0:
            return 0
        acc[tgt].add_patch(hsv[y0:y1, x0:x1], img[y0:y1, x0:x1])
        if paint[tgt] is None or paint[tgt].shape[:2] != (h, w):
            paint[tgt] = np.zeros((h, w), np.uint8)
        paint[tgt][y0:y1, x0:x1] = 255
        n = (x1 - x0) * (y1 - y0)
        if verbose:
            print(f"[calibra] pintado {tgt} en ({x},{y}) +{n} px  total={acc[tgt].px}",
                  flush=True)
        return n

    def on_mouse(event, x, y, flags, param):
        nonlocal dragging, mouse, _cb_ok
        _cb_ok = True
        mouse = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN or event == cv2.EVENT_LBUTTONDBLCLK:
            dragging = True
            _sample_at(x, y, verbose=True)
        elif event == cv2.EVENT_LBUTTONUP:
            dragging = False
        elif event == cv2.EVENT_MOUSEMOVE and (dragging or (flags & cv2.EVENT_FLAG_LBUTTON)):
            dragging = True
            _sample_at(x, y, verbose=False)

    cv2.setMouseCallback(win, on_mouse)
    cv2.setMouseCallback(win_m, on_mouse)

    print("[calibra] 1=rojo  2=verde  3=naranja  4=rosa")
    print("[calibra] CLIC en la ventana 'calibra_luz' (no en Terminal) y ARRASTRA")
    print("[calibra] Si el clic no pinta: cursor sobre el color y ESPACIO")
    print("[calibra] a=auto-detectar   n=siguiente frame   f=play/pausa")
    print("[calibra] r=reset  p=imprimir  s=sugerir todos  q=salir\n")
    if force_bev and not bev.is_calibrated:
        print("[calibra] --bev pedido pero no hay bev_calib.npz — naranja se mide en cámara.")
    if not bev.is_calibrated:
        print("[calibra] sin BEV: la cinta naranja se calibra en cámara (en carrera se ve en BEV).")

    def current_space(nombre):
        if force_bev:
            return True
        return ESPACIO.get(nombre, "cam") == "bev"

    try:
        while True:
            if frozen is None:
                raw = get_raw()
                if raw is None:
                    if image_mode:
                        time.sleep(0.03)
                        continue
                    continue
                if start_frozen and _first:
                    frozen = raw.copy()
                    _first = False
                    print("[calibra] pausado — clic en la IMAGEN (ventana 'calibra_luz'), no en Terminal")
                    print("[calibra] o pon el cursor sobre el color y pulsa ESPACIO")
                elif single_step:
                    frozen = raw.copy()
                    single_step = False
            else:
                raw = frozen

            nfr += 1
            want_bev = current_space(tgt)
            img, used_bev = _prepare_frame(raw, color_corr, use_corr, want_bev, bev)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            state["hsv"] = hsv
            state["img"] = img

            b, g, rch = _channels(img)
            auto_m = OBJETIVOS[tgt](b, g, rch).astype(np.uint8) * 255
            prod_m = _mask_hsv(hsv, rangos_prod[tgt])
            sug = acc[tgt].sugerido()
            sug_m = _mask_hsv(hsv, sug) if sug else np.zeros(hsv.shape[:2], np.uint8)

            disp = img.copy()
            if paint[tgt] is not None and paint[tgt].shape[:2] == disp.shape[:2]:
                _tint(disp, paint[tgt], (0, 255, 255), 0.55)
            if overlay_mode in ("prod", "both"):
                _tint(disp, prod_m, OVERLAY_BGR[tgt], 0.40)
            if overlay_mode in ("sug", "both") and sug:
                _tint(disp, sug_m, (0, 255, 255), 0.35)

            if show_bboxes and not used_bev:
                vis = img.copy()
                vis, pos = vision.detect_on(vis)
                for (x, y, w, h) in pos.get("Red", []):
                    cv2.rectangle(disp, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(disp, "Red", (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                for (x, y, w, h) in pos.get("Green", []):
                    cv2.rectangle(disp, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(disp, "Green", (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                pink_m = _mask_hsv(hsv, rangos_prod["rosa"])
                ratio = float(np.count_nonzero(pink_m)) / max(1, pink_m.size)
                cv2.putText(disp, f"rosa {ratio * 100:.0f}%", (8, disp.shape[0] - 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 50, 220), 1)

            mx, my = _xy_img(*mouse)
            h, w = disp.shape[:2]
            if 0 <= mx < w and 0 <= my < h:
                hv = hsv[my, mx]
                bv, gv, rv = img[my, mx]
                live = f"HSV=({hv[0]},{hv[1]},{hv[2]})  RGB=({rv},{gv},{bv})"
                cv2.circle(disp, (mx, my), PATCH_HALF, (0, 255, 255), 2)
                cv2.drawMarker(disp, (mx, my), (0, 255, 255), cv2.MARKER_CROSS, 18, 1)
            else:
                live = "HSV=(-,-,-)  pon el mouse en la ventana calibra_luz"
            if not _cb_ok:
                live = "el mouse no llega a OpenCV — usa ESPACIO con la ventana al frente"

            a = acc[tgt]
            if a.px:
                rs = a.resumen()
                acum = (f"pintados {a.px} px de {tgt}   HSV H {rs['h'][0]}-{rs['h'][2]}  "
                        f"S {rs['s'][0]}-{rs['s'][2]}  V {rs['v'][0]}-{rs['v'][2]}")
            else:
                acum = f"aun no pintas {tgt}: clic/espacio SOLO sobre ese color"

            gan = "-"
            if color_corr is not None and use_corr:
                g = color_corr.gains
                gan = f"({g[0]:.2f},{g[1]:.2f},{g[2]:.2f})"
            med, frac, quem = _linea_piso(img)
            espacio = "BEV" if used_bev else "camara"

            lines = [
                f"midiendo {tgt.upper()}   (1 rojo  2 verde  3 naranja  4 rosa)",
                live,
                acum,
                "r = BORRAR lo pintado de este color     p = ver rango HSV",
                "espacio = muestrear   n = otro frame   q = salir (no guarda solo)",
            ]
            for i, txt in enumerate(lines):
                cv2.putText(disp, txt, (8, 18 + i * 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 0, 0), 3)
                cv2.putText(disp, txt, (8, 18 + i * 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 255, 255) if i == 0 else (240, 240, 240), 1)

            mask_show = (img // 2).copy()
            if paint[tgt] is not None and paint[tgt].shape[:2] == mask_show.shape[:2]:
                mask_show[paint[tgt] > 0] = (0, 255, 255)
            cv2.putText(mask_show, f"AMARILLO = lo que pintaste de {tgt}   r = borrar",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(mask_show, "esto NO se pega solo al robot; p imprime el HSV",
                        (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

            cv2.imshow(win, disp)
            cv2.imshow(win_m, mask_show)
            cv2.setMouseCallback(win, on_mouse)
            cv2.setMouseCallback(win_m, on_mouse)
            key = cv2.waitKey(20) & 0xFF

            if key in (27, ord("q")):
                break
            if key == ord("1"):
                tgt = "rojo"
            elif key == ord("2"):
                tgt = "verde"
            elif key == ord("3"):
                tgt = "naranja"
            elif key == ord("4"):
                tgt = "rosa"
            elif key == ord("r"):
                acc[tgt].reset()
                paint[tgt] = None
                print(f"[calibra] reset {tgt}")
            elif key == ord("a"):
                ok = acc[tgt].add(hsv, img, auto_m > 0)
                if ok:
                    if paint[tgt] is None or paint[tgt].shape[:2] != auto_m.shape[:2]:
                        paint[tgt] = np.zeros(auto_m.shape[:2], np.uint8)
                    paint[tgt][auto_m > 0] = 255
                print(f"[calibra] auto {tgt}: "
                      f"{'ok +' + str(int(np.count_nonzero(auto_m))) + ' px' if ok else 'blob demasiado chico'}")
            elif key == ord("p"):
                _print_objetivo(tgt, acc[tgt], None, True, rangos_prod)
            elif key == ord("s"):
                print(f"\n=== muestra GUI ({nfr} frames de visor) ===")
                _print_piso(*_piso_from_frame(img))
                for nom in targets:
                    _print_objetivo(nom, acc[nom], None, True, rangos_prod)
                _print_cone_sync_note()
            elif key == ord("c"):
                use_corr = not use_corr
                print(f"[calibra] color_corr={'ON' if use_corr else 'OFF'}")
            elif key == ord("b"):
                force_bev = not force_bev
                print(f"[calibra] BEV forzado={'ON' if force_bev else 'OFF (auto por color)'}")
            elif key == ord("m"):
                order = ["prod", "sug", "both", "off"]
                overlay_mode = order[(order.index(overlay_mode) + 1) % len(order)]
            elif key == ord("d"):
                show_bboxes = not show_bboxes
            elif key == ord("f"):
                frozen = None if frozen is not None else raw.copy()
                print(f"[calibra] freeze={'ON' if frozen is not None else 'OFF'}")
            elif key == ord("n"):
                single_step = True
                frozen = None
                print("[calibra] siguiente frame")
            elif key == ord(" "):
                n = _sample_at(*mouse, verbose=True)
                if n == 0:
                    print("[calibra] pon el cursor SOBRE la imagen y pulsa espacio otra vez")
    except KeyboardInterrupt:
        print("\n[calibra] interrumpido")
    finally:
        cv2.destroyAllWindows()

    print(f"\n=== cierre GUI ===")
    for nom in targets:
        _print_objetivo(nom, acc[nom], None, True, rangos_prod)
    _print_cone_sync_note()


def _piso_from_frame(img):
    med, frac, quem = _linea_piso(img)
    piso_acc = [med] if frac > 0.2 else []
    return piso_acc, [quem]


def frames_camara(cam_index, segundos, use_corr: bool):
    cap = open_camera(cam_index)
    corr = FloorColorCorrector() if use_corr else None
    t0 = time.monotonic()
    k = 0
    try:
        while segundos <= 0 or time.monotonic() - t0 < segundos:
            ok, f = cap.read()
            if not ok:
                continue
            k += 1
            if corr is not None:
                f = corr.process(f)
            yield k, f
    finally:
        cap.release()


def _panel_camara(frame):
    """HUD de la Pi = cámara 640 + BEV. Video de WhatsApp/pantalla = usar todo."""
    if frame is None:
        return None
    w = frame.shape[1]
    return frame[:, :640] if w > 640 else frame


def main():
    ap = argparse.ArgumentParser(
        description="Calibra HSV de conos / naranja / rosa para Pure Pursuit (otra luz)")
    ap.add_argument("--gui", action="store_true",
                    help="ventana: pintar / auto-detectar / overlay (se puede juntar con --avi/--image)")
    src = ap.add_mutually_exclusive_group(required=False)
    src.add_argument("--avi", help="video .avi/.mp4 (HUD: panel izquierdo; si es más angosto, el frame entero)")
    src.add_argument("--bin", dest="binf", help="orillasNNN_bev.bin (BEV limpio)")
    src.add_argument("--vivo", action="store_true", help="cámara en vivo, solo texto (SSH)")
    ap.add_argument("--image", type=str, default=None, help="foto fija (implica --gui)")
    ap.add_argument("--segundos", type=float, default=0, help="--vivo: cuánto medir (0 = hasta Ctrl-C)")
    ap.add_argument("--cada", type=int, default=1, help="procesar 1 de cada N frames")
    ap.add_argument("--cam-index", type=int, default=None)
    ap.add_argument("--sugerir", action="store_true", help="imprime los rangos listos para pegar")
    ap.add_argument("--sin-corr", action="store_true",
                    help="no aplicar color_corr.py (por defecto SÍ, como el runtime)")
    ap.add_argument("--bev", action="store_true", help="--gui: forzar espacio BEV para todos los colores")
    args = ap.parse_args()
    if args.image:
        args.gui = True
    if not any([args.gui, args.avi, args.binf, args.vivo]):
        ap.error("indica --gui, --avi, --vivo o --image")

    cam_index = args.cam_index if args.cam_index is not None else C.CAM_INDEX
    use_corr = not args.sin_corr

    if args.gui:
        if args.image:
            img = cv2.imread(args.image)
            if img is None:
                raise SystemExit(f"[calibra] no se pudo leer {args.image}")
            run_gui(lambda: img, cam_index, use_corr, args.bev, image_mode=True,
                    start_frozen=True)
            return
        if args.avi:
            cap = cv2.VideoCapture(args.avi)
            if not cap.isOpened():
                raise SystemExit(f"[calibra] no se pudo abrir {args.avi}")

            def get_raw():
                ok, f = cap.read()
                if not ok:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, f = cap.read()
                    if not ok:
                        return None
                return _panel_camara(f)
            try:
                run_gui(get_raw, cam_index, False, args.bev, image_mode=False,
                        start_frozen=True)
            finally:
                cap.release()
            return

        print("[calibra] cámara en vivo. Si wro-runtime está arriba: "
              "sudo systemctl stop wro-runtime && sleep 4", flush=True)
        cap = open_camera(cam_index)

        def get_raw():
            ok, f = cap.read()
            return f if ok else None
        try:
            run_gui(get_raw, cam_index, use_corr, args.bev, image_mode=False)
        finally:
            cap.release()
        return

    if args.avi:
        it = frames_avi(args.avi)
        vivo = False
    elif args.binf:
        it = frames_bin(args.binf)
        vivo = False
    else:
        print("[calibra] cámara en vivo. Si wro-runtime está arriba: "
              "sudo systemctl stop wro-runtime && sleep 4", flush=True)
        it = frames_camara(cam_index, args.segundos, use_corr)
        vivo = True

    analizar(it, args.cada, vivo, args.sugerir)


if __name__ == "__main__":
    main()
