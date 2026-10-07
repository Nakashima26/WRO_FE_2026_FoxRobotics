"""Fotos RAW (DNG) de campo plano para calibrar ALSC con rpi-ctt.
1) mide la temperatura de color de la luz con AWB automatico
2) fija exposicion un poco abajo de la automatica (que nada sature)
3) guarda N DNG a resolucion completa: alsc_<CT>k_<n>.dng"""
import sys, time, numpy as np
from picamera2 import Picamera2
N = int(sys.argv[1]) if len(sys.argv) > 1 else 3
cam = Picamera2()
mode = max(cam.sensor_modes, key=lambda md: md["size"][0])
print("modo RAW:", mode["size"], mode["unpacked"])
cfg = cam.create_still_configuration(raw={"size": mode["size"], "format": mode["unpacked"]}, buffer_count=2)
cam.configure(cfg); cam.start(); time.sleep(3)
m = cam.capture_metadata()
ct = int(round(m.get("ColourTemperature", 4000), -2))
exp, ag = m["ExposureTime"], m["AnalogueGain"]
print(f"luz ~{ct} K  exp={exp} us  ganancia={ag:.2f}  lux~{m.get("Lux",0):.0f}")
cam.set_controls({"AeEnable": False, "ExposureTime": int(exp * 0.7), "AnalogueGain": ag}); time.sleep(1.5)
for i in range(N):
    req = cam.capture_request()
    raw = req.make_array("raw").view(np.uint16)
    req.save_dng(f"/home/user/alsc_cal/alsc_{ct}k_{i}.dng"); req.release()
    print(f"alsc_{ct}k_{i}.dng  raw max={raw.max()}  p99={np.percentile(raw,99):.0f}")
cam.stop()
