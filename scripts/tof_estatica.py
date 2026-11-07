"""
Calibración estática de los ToF con VueltaCalibracion en ESPERA (sin mandar 'g').

Pones una pared (negra, la de la pista) perpendicular a la distancia indicada,
medida con cinta desde la CARA del sensor, y el script prueba varias
configuraciones de los ToF por Bluetooth, junta ~3 s de cada una y lo agrega a
un CSV. Luego `--analiza` ajusta la corrección por sensor y configuración.

Uso:
  python scripts/tof_estatica.py 30 L          pared a 30 cm del sensor izquierdo
  python scripts/tof_estatica.py 30 LR         la misma distancia para L y R (dos paredes)
  python scripts/tof_estatica.py 100 F --cfg "L 8 50,S 8 50"
  python scripts/tof_estatica.py --analiza

Configuración = "<modo S|M|L> <ROI 4-16> <presupuesto ms>". La de carrera es "L 8 50".
"""
import argparse
import csv
import datetime
import os
import re
import sys
import time

import serial
import serial.tools.list_ports

ESP32_MAC = "1CC3ABC14462"
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CSV_PATH = os.path.join(RAIZ, "logs_tof", "tof_estatica.csv")
CFG_DEFECTO = "L 8 50,S 8 50,S 16 50,L 16 50"
SEG_POR_CFG = 3
COLS = ["fecha", "sensor", "dist_mm", "cfg", "med_mm", "min_mm", "max_mm", "n", "st0", "st2", "st7", "otro", "senal"]
PAT = re.compile(r"# ToF (\w): med\s+(\d+) mm \((\d+)-(\d+)\) n=(\d+) st0:(\d+) st2:(\d+) st7:(\d+) otro:(\d+) se\S*al ([\d.]+)")


def puerto():
    for p in serial.tools.list_ports.comports():
        if ESP32_MAC in p.hwid.upper():
            return p.device
    sys.exit("No encontré el ESP32 por Bluetooth (¿emparejado y prendido?).")


def medir(dist_cm, sensores, cfgs):
    p = serial.Serial()
    p.port, p.baudrate, p.timeout = puerto(), 115200, 0.3
    p.dtr = p.rts = False
    print("conectando...", flush=True)
    p.open()
    print("conectado", flush=True)
    time.sleep(2.0)
    p.reset_input_buffer()
    nuevo = not os.path.exists(CSV_PATH)
    filas = []
    try:
        for cfg in cfgs:
            p.write(f"c {cfg}\n".encode())
            t0 = time.time()
            while time.time() - t0 < 1.5:            # que se asiente con la config nueva
                p.readline()
            p.write(b"r\n")
            datos = {s: [] for s in sensores}
            t0 = time.time()
            while time.time() - t0 < SEG_POR_CFG + 0.3:
                ln = p.readline().decode("utf-8", "replace").strip()
                m = PAT.search(ln)
                if m and m.group(1) in datos:
                    datos[m.group(1)].append([int(x) for x in m.groups()[1:9]] + [float(m.group(10))])
            for s, d in datos.items():
                if not d:
                    print(f"  {s} {cfg}: sin datos")
                    continue
                meds = sorted(x[0] for x in d)
                n = sum(x[3] for x in d)
                fila = [datetime.datetime.now().isoformat(timespec="seconds"), s, int(round(dist_cm * 10)), cfg,
                        meds[len(meds) // 2], min(x[1] for x in d), max(x[2] for x in d), n,
                        sum(x[4] for x in d), sum(x[5] for x in d), sum(x[6] for x in d), sum(x[7] for x in d),
                        round(sum(x[8] * x[3] for x in d) / max(n, 1), 3)]
                filas.append(fila)
                err = fila[4] - fila[2]
                print(f"  {s} {cfg:9s}: med {fila[4]:5d} mm (real {fila[2]}, error {err:+5d})  "
                      f"rango {fila[5]}-{fila[6]}  n={n}  st0/2/7: {fila[8]}/{fila[9]}/{fila[10]}  señal {fila[12]}")
        p.write(f"c {cfgs_carrera()}\n".encode())   # deja la de carrera
    finally:
        p.close()
    with open(CSV_PATH, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if nuevo:
            w.writerow(COLS)
        w.writerows(filas)
    print(f"guardado en {CSV_PATH}")


def cfgs_carrera():
    return "L 8 50"


def analiza():
    import numpy as np
    rows = list(csv.DictReader(open(CSV_PATH, encoding="utf-8")))
    grupos = {}
    for r in rows:
        grupos.setdefault((r["sensor"], r["cfg"]), []).append(r)
    print("sensor cfg        puntos  error medio  ruido medio  real = a*medido + b        error tras corregir")
    for (s, cfg), g in sorted(grupos.items()):
        real = np.array([float(r["dist_mm"]) for r in g])
        med = np.array([float(r["med_mm"]) for r in g])
        ruido = np.array([float(r["max_mm"]) - float(r["min_mm"]) for r in g])
        linea = ""
        if len(g) >= 2:
            a, b = np.polyfit(med, real, 1)
            res = real - (a * med + b)
            linea = f"{a:6.3f} * med {b:+7.1f}      max {np.abs(res).max():5.1f} mm"
        print(f"  {s}   {cfg:9s}  {len(g):4d}  {np.mean(med - real):+8.1f} mm  {ruido.mean():8.1f} mm   {linea}")
        for r in sorted(g, key=lambda r: float(r["dist_mm"])):
            print(f"        {float(r['dist_mm']) / 10:5.0f} cm -> med {r['med_mm']:>5} (st0/2/7 {r['st0']}/{r['st2']}/{r['st7']}, señal {r['senal']})")


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dist_cm", nargs="?", type=float, help="distancia real de la pared a la cara del sensor (cm)")
    ap.add_argument("sensores", nargs="?", default="LR", help="F, L, R o combinaciones (LR)")
    ap.add_argument("--cfg", default=CFG_DEFECTO, help=f"configuraciones separadas por coma (def. {CFG_DEFECTO!r})")
    ap.add_argument("--analiza", action="store_true", help="ajusta la corrección con lo medido")
    a = ap.parse_args()
    if a.analiza:
        analiza()
        return
    if a.dist_cm is None:
        ap.error("falta la distancia")
    medir(a.dist_cm, a.sensores.upper(), [c.strip() for c in a.cfg.split(",") if c.strip()])


if __name__ == "__main__":
    main()
