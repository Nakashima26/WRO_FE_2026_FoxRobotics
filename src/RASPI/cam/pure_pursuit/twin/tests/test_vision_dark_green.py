"""r276: ruido oscuro (lente tapado, V~28, H~30) no debe salir como Green con GREEN_V_MIN_MED."""
import cv2
import numpy as np

from vision import Vision


def _frame(v):
    hsv = np.zeros((480, 640, 3), np.uint8)
    hsv[100:300, 100:300] = (60, 200, v)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def test_dark_green_blob_rejected_only_when_enabled():
    vis = Vision(open_cam=False)
    dark, bright = _frame(28), _frame(200)
    assert vis.detect_on(dark)[1]["Green"]          # apagado: comportamiento anterior
    vis.GREEN_V_MIN_MED = 60
    assert not vis.detect_on(dark)[1]["Green"]
    assert vis.detect_on(bright)[1]["Green"]
