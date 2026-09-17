"""
calibra_luz.py — Calibración de color para OTRA ILUMINACIÓN (sede nueva).

POR QUÉ EXISTE
  Los rangos HSV del proyecto (vision.py Red/Green, LINE_ORANGE_HSV,
  PARK_PINK_HSV, FLOOR_*) se midieron con la luz del cuarto de pruebas y la
  cámara corre con balance de blancos APAGADO y ganancias fijas
  (vision.py: awb-enable=false colour-gains=<1.2,1.5>). Con otra luz TODO se
  corre y los conos dejan de existir para el carro.

  Medido en orillas1013 (sede 2026-09-16): el cono ROJO salió en H 169-178
  (cola azulada del rojo) en vez de H 0-1. El rango de producción cubría
  H<=5 y H>=177, así que el cono grande que el carro tenía ENFRENTE
  (frames 276-285, 18k px rojos) daba CERO detecciones.

  color_corr.py debería tapar esto solo, pero se apoya en medir el PISO y
  en esa sede el piso salía quemado (V~237, 33% de px a 255): un piso
  quemado ya no tiene tono, las ganancias salen ~1 y no corrige nada.
  Por eso hace falta medir a mano.

LA IDEA
  Para ENCONTRAR el cono no se usa HSV (que es justo lo que está corrido):
  se usa DOMINANCIA DE CANAL (R claramente por encima de G y B = objeto
  rojo, pase lo que pase con la luz). Una vez ubicado el blob, se MIDE su
  HSV real y se imprime el rango que sí lo cubre.

USO (en la Pi, sin pantalla — todo por SSH)
  # 1) Ver si los rangos ACTUALES agarran los conos de esta sede, y si no,
  #    cuál de los topes (H / S / V) es el que los está tirando:
  python3 -m pure_pursuit.calibra_luz --avi videos_orillas/orillas1013.avi

  # 2) En vivo, con la cámara: pon un cono rojo y otro verde enfrente y
  #    camina el carro por la pista. Imprime un resumen cada segundo.
  python3 -m pure_pursuit.calibra_luz --vivo

  # 3) Rangos listos para pegar en vision.py / config.py:
  python3 -m pure_pursuit.calibra_luz --avi ... --sugerir

  Con --avi se lee el panel IZQUIERDO del HUD (= processed_frame, ya
  corregido por color_corr y volteado), que es exactamente lo que ve
  Vision.process_frame(). Con --bin se lee el BEV limpio.

OJO: el servicio wro-runtime tiene tomada la cámara. Para --vivo hay que
pararlo ANTES (y NUNCA con `systemctl restart`, que deja la CSI colgada):
  sudo systemctl stop wro-runtime && sleep 4
  ... calibrar ...
  sudo systemctl start wro-runtime
"""

import argparse
import os
import struct
import sys
import time

import cv2
import numpy as np


# ── Cómo se BUSCA cada objetivo (independiente de la luz) ────────────────────
# No se usa HSV aquí a propósito: si el tono está corrido, buscar por tono no
# encuentra nada. Se busca por qué canal domina y por cuánto.
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


class Acumulador:
    """Junta los px de un objetivo a lo largo de muchos frames."""

    def __init__(self, nombre):
        self.nombre = nombre
        # se guardan histogramas, no los px: no crece con la duracion
        self.hist_h = np.zeros(180, np.int64)
        self.hist_s = np.zeros(256, np.int64)
        self.hist_v = np.zeros(256, np.int64)
        self.br = []         # mediana de B/R por frame (separa cono de pared rosa)
        self.frames = 0
        self.px = 0

    def add(self, hsv, bgr, m):
        n = int(m.sum())
        if n < MIN_PX:
            return False
        self.hist_h += np.bincount(hsv[..., 0][m], minlength=180)
        self.hist_s += np.bincount(hsv[..., 1][m], minlength=256)
        self.hist_v += np.bincount(hsv[..., 2][m], minlength=256)
        b = bgr[..., 0][m].astype(np.float32)
        r = bgr[..., 2][m].astype(np.float32)
        self.br.append(float(np.median(b / np.maximum(r, 1.0))))
        self.frames += 1
        self.px += n
        return True

    @staticmethod
    def _pct(hist, qs):
        tot = hist.sum()
        if tot == 0:
            return [0] * len(qs)
        acc = np.cumsum(hist)
        return [int(np.searchsorted(acc, tot * q / 100.0)) for q in qs]

    def h_circular(self, qs=(2, 50, 98)):
        """Percentiles de H tolerando el wrap 179->0 (el rojo vive a caballo).

        Se rota el histograma al punto de corte más vacío, se saca el
        percentil y se des-rota.
        """
        hist = self.hist_h
        if hist.sum() == 0:
            return [0] * len(qs), 0
        # el mejor corte es el bin (o hueco) con menos px alrededor
        suave = np.convolve(np.r_[hist, hist], np.ones(9), "same")[:180]
        corte = int(np.argmin(suave))
        rot = np.roll(hist, -corte)
        vals = self._pct(rot, qs)
        return [(v + corte) % 180 for v in vals], corte

    def resumen(self):
        (h2, h50, h98), corte = self.h_circular()
        s2, s50, s98 = self._pct(self.hist_s, (2, 50, 98))
        v2, v50, v98 = self._pct(self.hist_v, (2, 50, 98))
        br = float(np.median(self.br)) if self.br else float("nan")
        return dict(h=(h2, h50, h98), s=(s2, s50, s98), v=(v2, v50, v98),
                    br=br, corte=corte, frames=self.frames, px=self.px)

    def banda_h(self, margen=4):
        """Devuelve 1 o 2 tramos [lo,hi] de H que cubren p2..p98, partiendo
        el tramo en dos si cruza el wrap 179->0 (como hace vision.py)."""
        (h2, _, h98), corte = self.h_circular()
        lo = (h2 - margen) % 180
        hi = (h98 + margen) % 180
        if lo <= hi:
            return [(lo, hi)]
        return [(0, hi), (lo, 179)]


# Copia literal de los rangos de vision.py — se comparan contra lo medido.
# Si se cambian allá, cambiarlos aquí (este script solo REPORTA).
RANGOS_PROD = {
    "rojo":  [((0, 150, 40), (5, 255, 200)), ((168, 170, 40), (179, 255, 235))],
    "verde": [((30, 35, 25), (85, 255, 255))],
}


def _pasa(hsv, m, rangos):
    """px del blob que caen dentro de los rangos de producción, y cuántos
    fallarían por CADA tope por separado (para saber a quién culpar)."""
    H = hsv[..., 0][m].astype(np.int16)
    S = hsv[..., 1][m].astype(np.int16)
    V = hsv[..., 2][m].astype(np.int16)
    ok = np.zeros(H.shape, bool)
    okh = np.zeros(H.shape, bool)
    oks = np.zeros(H.shape, bool)
    okv = np.zeros(H.shape, bool)
    for lo, hi in rangos:
        okh |= (H >= lo[0]) & (H <= hi[0])
        oks |= (S >= lo[1]) & (S <= hi[1])
        okv |= (V >= lo[2]) & (V <= hi[2])
        ok |= ((H >= lo[0]) & (H <= hi[0]) & (S >= lo[1]) & (S <= hi[1])
               & (V >= lo[2]) & (V <= hi[2]))
    n = max(1, H.size)
    return ok.sum() / n, okh.sum() / n, oks.sum() / n, okv.sum() / n


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


def frames_camara(cam_index, segundos):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from vision import open_camera
    cap = open_camera(cam_index)
    t0 = time.monotonic()
    k = 0
    try:
        while segundos <= 0 or time.monotonic() - t0 < segundos:
            ok, f = cap.read()
            if not ok:
                continue
            k += 1
            yield k, cv2.flip(f, 1)
    finally:
        cap.release()


# ── Reporte ──────────────────────────────────────────────────────────────────
def _linea_piso(img):
    """Color del piso en la franja de abajo + aviso de sobreexposición.
    Es el número que va en COLOR_CORR_FLOOR_REF_BGR (si se quiere re-anclar)
    y la señal de que el piso está quemado y color_corr no puede trabajar."""
    h = img.shape[0]
    roi = img[int(h * 0.70)::4, ::4]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    piso = (hsv[..., 1] <= 90) & (hsv[..., 2] >= 60) & (hsv[..., 2] <= 250)
    frac = float(piso.mean())
    med = (np.median(roi[piso].reshape(-1, 3), axis=0) if piso.any()
           else np.array([0, 0, 0]))
    quemado = float((cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[..., 2] >= 250).mean())
    return med, frac, quemado


def main():
    ap = argparse.ArgumentParser(
        description="Mide los colores reales de la sede y dice por qué no se ve el rojo")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--avi", help="orillasNNN.avi (usa el panel izquierdo del HUD)")
    src.add_argument("--bin", dest="binf", help="orillasNNN_bev.bin (BEV limpio)")
    src.add_argument("--vivo", action="store_true", help="cámara en vivo (para el servicio primero)")
    ap.add_argument("--segundos", type=float, default=0, help="--vivo: cuánto medir (0 = hasta Ctrl-C)")
    ap.add_argument("--cada", type=int, default=1, help="procesar 1 de cada N frames")
    ap.add_argument("--cam-index", type=int, default=0)
    ap.add_argument("--sugerir", action="store_true", help="imprime los rangos listos para pegar")
    args = ap.parse_args()

    if args.avi:
        it = frames_avi(args.avi)
    elif args.binf:
        it = frames_bin(args.binf)
    else:
        it = frames_camara(args.cam_index, args.segundos)

    acc = {k: Acumulador(k) for k in OBJETIVOS}
    prod = {k: [0.0, 0.0, 0.0, 0.0, 0] for k in RANGOS_PROD}   # ok,h,s,v,frames
    piso_acc, quemado_acc, nfr = [], [], 0
    t_log = time.monotonic()

    try:
        for n, img in it:
            if n % args.cada:
                continue
            nfr += 1
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            b, g, r = (img[..., 0].astype(np.int16), img[..., 1].astype(np.int16),
                       img[..., 2].astype(np.int16))
            for nom, fn in OBJETIVOS.items():
                m = fn(b, g, r)
                if not acc[nom].add(hsv, img, m):
                    continue
                if nom in RANGOS_PROD:
                    p = _pasa(hsv, m, RANGOS_PROD[nom])
                    d = prod[nom]
                    for i in range(4):
                        d[i] += p[i]
                    d[4] += 1
            med, frac, quem = _linea_piso(img)
            if frac > 0.2:
                piso_acc.append(med)
            quemado_acc.append(quem)

            if args.vivo and time.monotonic() - t_log > 1.0:
                t_log = time.monotonic()
                rs = acc["rojo"].resumen()
                d = prod["rojo"]
                pct = (d[0] / d[4] * 100) if d[4] else 0.0
                print(f"[cal] f={nfr} rojo: H{rs['h']} S{rs['s']} V{rs['v']} "
                      f"B/R={rs['br']:.2f} vistos={rs['frames']} "
                      f"-> pasan rango actual {pct:.0f}%", flush=True)
    except KeyboardInterrupt:
        print("\n[cal] interrumpido")

    # ── informe ──────────────────────────────────────────────────────────────
    print(f"\n=== {nfr} frames analizados ===")
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

    for nom in ("rojo", "verde", "naranja", "rosa"):
        a = acc[nom]
        if a.frames == 0:
            print(f"\n{nom.upper()}: no se vio en ningun frame")
            continue
        rs = a.resumen()
        print(f"\n{nom.upper()}: {a.frames} frames, {a.px} px")
        print(f"  H p2/med/p98 = {rs['h'][0]:3d} / {rs['h'][1]:3d} / {rs['h'][2]:3d}"
              f"   S = {rs['s'][0]:3d} / {rs['s'][1]:3d} / {rs['s'][2]:3d}"
              f"   V = {rs['v'][0]:3d} / {rs['v'][1]:3d} / {rs['v'][2]:3d}")
        print(f"  B/R mediano = {rs['br']:.2f}"
              + ("   (cono real: <=0.35; pared rosa del estacionamiento: >=0.45)"
                 if nom == "rojo" else ""))
        if nom in RANGOS_PROD:
            d = prod[nom]
            if d[4]:
                ok, ph, ps, pv = (d[0] / d[4] * 100, d[1] / d[4] * 100,
                                  d[2] / d[4] * 100, d[3] / d[4] * 100)
                print(f"  con los rangos ACTUALES pasa el {ok:.0f}% de esos px", end="")
                if ok < 40:
                    culpa = min((("H", ph), ("S", ps), ("V", pv)), key=lambda t: t[1])
                    print(f"  <-- el tope que lo tira es {culpa[0]} "
                          f"(solo {culpa[1]:.0f}% lo pasa; H={ph:.0f}% S={ps:.0f}% V={pv:.0f}%)")
                else:
                    print("  (ok)")
        if args.sugerir:
            tramos = a.banda_h()
            s_lo = max(0, rs['s'][0] - 20)
            v_lo = max(0, rs['v'][0] - 20)
            v_hi = min(255, rs['v'][2] + 25)
            txt = ", ".join(f"(np.array([{lo}, {s_lo}, {v_lo}]), "
                            f"np.array([{hi}, 255, {v_hi}]))" for lo, hi in tramos)
            print(f"  SUGERIDO -> [{txt}]")

    print("\nNota: los rangos sugeridos cubren p2..p98 de lo MEDIDO aqui. Antes de")
    print("pegarlos, vuelve a correr este script sobre una run VIEJA (luz del")
    print("cuarto de pruebas) para confirmar que no pierdes nada alla.")


if __name__ == "__main__":
    main()
