"""Simetriza la tabla de BRILLO (luminance_lut) de la ALSC de un tuning.

Por qué: el oscurecimiento de un lente (viñeteo) es simétrico respecto al centro,
pero si la hoja blanca de la calibración no estaba iluminada parejo, la tabla que
calcula rpi-ctt también "corrige" ese degradado de luz (en la nuestra: esquinas de
arriba x4.3, de abajo x2.0). Promediar la tabla con sus espejos (izq/der, arriba/abajo)
conserva la parte simétrica (el lente) y cancela la inclinada (la luz).
Las tablas de COLOR (calibrations_Cr / Cb) no se tocan.

Uso:  python simetriza_luminancia.py imx219_noir_wro_pi5.json   (sobrescribe el archivo)
"""
import json
import sys

import numpy as np

path = sys.argv[1]
t = json.load(open(path))
alsc = [a["rpi.alsc"] for a in t["algorithms"] if "rpi.alsc" in a][0]
lut = np.array(alsc["luminance_lut"], dtype=float).reshape(32, 32)
sym = (lut + lut[:, ::-1] + lut[::-1, :] + lut[::-1, ::-1]) / 4.0
sym /= sym.min()   # la ALSC espera mínimo 1.0 (centro sin ganancia)
print("esquinas antes  :", [round(lut[i, j], 2) for i, j in ((0, 0), (0, -1), (-1, 0), (-1, -1))])
print("esquinas después:", [round(sym[i, j], 2) for i, j in ((0, 0), (0, -1), (-1, 0), (-1, -1))])
alsc["luminance_lut"] = [round(float(v), 3) for v in sym.flatten()]
json.dump(t, open(path, "w"), indent=4)
print("guardado:", path)
