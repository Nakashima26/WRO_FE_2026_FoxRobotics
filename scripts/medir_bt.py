"""Pruebas de dinámica del carro con CalibFox por Bluetooth (o USB).

Manda un comando de CalibFox, guarda el CSV que regresa en logs_tof/ con el nombre
de la prueba y el voltaje, y saca un resumen (velocidad final, tiempos de
subida, distancia con motor y distancia hasta pararse).

  python scripts/medir_bt.py escalon255 "COAST 255 1000 2000 BRAKE" --volt 12.1
  python scripts/medir_bt.py coast150 "COAST 150 1500 3000" --volt 12.0 --cinta 132
  python scripts/medir_bt.py ver logs_tof/escalon255_12.1V_....csv

--cinta: lo que midió la cinta (cm, de la marca de salida a donde quedó), para
compararlo con el encoder. Siempre manda CPMM 5.45 y MAXPWM 255 antes.
"""
import argparse
import os
import sys
import time

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS = os.path.join(RAIZ, "logs_tof")
CPMM = 5.45


def correr(puerto, comando, timeout):
    import serial
    s = serial.Serial(puerto, 115200, timeout=0.2)
    time.sleep(0.5)
    s.read(10000)
    lineas = []
    for previo in (f"CPMM {CPMM}", "MAXPWM 255"):
        s.write((previo + "\n").encode())
        time.sleep(0.3)
    s.read(10000)
    s.write((comando + "\n").encode())
    clave = comando.split()[0].upper()
    t0, buf = time.time(), b""
    while time.time() - t0 < timeout:
        buf += s.read(4096)
        while b"\n" in buf:
            lin, buf = buf.split(b"\n", 1)
            lin = lin.decode(errors="replace").strip()
            if not lin:
                continue
            lineas.append(lin)
            if lin.startswith("OK " + clave) or lin.startswith("ERR"):
                s.close()
                return lineas
    s.close()
    lineas.append("# TIMEOUT")
    return lineas


def tof_frente(puerto):
    """Mediana del ToF de enfrente (Long, 300 ms, 4 s) parado: mm a la pared. A ~2 m
    sale con status 2 (señal débil) pero estable (±22 mm, la mediana de 14 ±7 mm)."""
    lineas = correr(puerto, "TOF F LONG 300 4000", 20)
    mm = []
    for lin in lineas:
        c = lin.split(",")
        if len(c) == 6 and c[0].isdigit() and 50 < float(c[2]) < 4000:
            mm.append(float(c[2]))
    return (float(np.median(mm)) if len(mm) >= 3 else None), len(mm)


def tabla(lineas):
    filas = []
    for lin in lineas:
        if lin[:1].isdigit():
            try:
                filas.append([float(z) for z in lin.split(",")])
            except ValueError:
                pass
    return np.array(filas) if filas else np.zeros((0, 7))


def resumen(D, cinta=None):
    if len(D) < 5:
        return ["sin datos"]
    t, pwm, cnt, yaw = D[:, 0] / 1000, D[:, 1], D[:, 2], D[:, 6]
    x = cnt / CPMM                                   # mm
    v = np.gradient(x, t)                            # mm/s, más limpio que la columna v
    vs = np.convolve(v, np.ones(5) / 5, mode="same")
    on = np.abs(pwm) > 0
    out = []
    if on.any():
        i_off = int(np.nonzero(on)[0][-1]) + 1
        t_off = t[min(i_off, len(t) - 1)]
        tramo = (t > t_off - 0.3) & (t <= t_off)
        v_fin = float(np.median(v[tramo])) if tramo.any() else float("nan")
        out.append(f"pwm {int(np.max(np.abs(pwm)))}  motor {t_off:.2f} s  v_final {v_fin:.0f} mm/s "
                   f"(últimos 0.3 s)  vmax {vs.max():.0f}")
        for frac in (0.1, 0.5, 0.63, 0.9):
            k = np.nonzero((vs >= frac * v_fin) & on)[0]
            if len(k):
                out.append(f"  llega a {frac * 100:3.0f}%  en {t[k[0]]:.3f} s")
        k0 = np.nonzero(x > 2)[0]
        if len(k0):
            out.append(f"  primer movimiento (2 mm) {t[k0[0]]:.3f} s")
        x_off = float(np.interp(t_off, t, x))
        v_off = v_fin
        quieto = np.nonzero((t > t_off) & (np.abs(vs) < 20))[0]
        t_stop = t[quieto[0]] if len(quieto) else t[-1]
        d_freno = x[-1] - x_off
        out.append(f"  con motor {x_off:.0f} mm, al soltar {v_off:.0f} mm/s; después {d_freno:.0f} mm "
                   f"en {t_stop - t_off:.2f} s")
        if d_freno > 5 and v_off > 50:
            out.append(f"  desaceleración media (encoder) {v_off ** 2 / (2 * d_freno):.0f} mm/s²")
    out.append(f"  encoder total {x[-1]:.0f} mm  yaw final {yaw[-1]:+.1f}°")
    if cinta is not None and x[-1] > 0:
        out.append(f"  real {cinta * 10:.0f} mm  ->  real - encoder = {cinta * 10 - x[-1]:+.0f} mm "
                   f"({(cinta * 10 / x[-1] - 1) * 100:+.1f} %)")
    return out


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "ver":
        lineas = open(sys.argv[2], encoding="utf-8").read().splitlines()
        cinta = next((float(l.split("=", 1)[1]) for l in lineas if l.startswith("# cinta_cm=")), None)
        print("\n".join(resumen(tabla(lineas), cinta)))
        return
    ap = argparse.ArgumentParser()
    ap.add_argument("prueba")
    ap.add_argument("comando")
    ap.add_argument("--volt", type=float, required=True)
    ap.add_argument("--cinta", type=float, help="cm medidos con cinta")
    ap.add_argument("--puerto", default="COM6")
    ap.add_argument("--sin-tof", action="store_true",
                    help="no medir con el ToF de enfrente antes y después (la regla sin cinta)")
    ap.add_argument("--timeout", type=float, default=40)
    a = ap.parse_args()
    antes = despues = None
    if not a.sin_tof:
        antes, n = tof_frente(a.puerto)
        print(f"ToF enfrente antes: {antes} mm ({n} lecturas válidas)")
    lineas = correr(a.puerto, a.comando, a.timeout)
    if not a.sin_tof:
        time.sleep(0.5)
        despues, n = tof_frente(a.puerto)
        print(f"ToF enfrente después: {despues} mm ({n} lecturas válidas)")
        D = tabla(lineas)
        if antes and despues and a.cinta is None and len(D):
            # Sin cinta, la regla es el ToF. Si quedó girado, el rayo pega chueco en la
            # pared: la distancia de frente es despues·cos(yaw).
            a.cinta = (antes - despues * np.cos(np.radians(D[-1, 6]))) / 10.0
    os.makedirs(LOGS, exist_ok=True)
    nombre = os.path.join(LOGS, f"{a.prueba}_{a.volt:.2f}V_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    with open(nombre, "w", encoding="utf-8") as f:
        f.write(f"# prueba={a.prueba}\n# comando={a.comando}\n# volt={a.volt}\n# cpmm={CPMM}\n")
        if antes and despues:
            f.write(f"# tof_antes_mm={antes:.0f}\n# tof_despues_mm={despues:.0f}\n")
        if a.cinta is not None:
            f.write(f"# cinta_cm={a.cinta}\n")
        f.write("\n".join(lineas) + "\n")
    print(nombre)
    for lin in lineas:
        if not lin[:1].isdigit():
            print(lin)
    print("\n".join(resumen(tabla(lineas), a.cinta)))


if __name__ == "__main__":
    main()
