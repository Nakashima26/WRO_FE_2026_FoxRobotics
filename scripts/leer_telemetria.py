#!/usr/bin/env python3
"""
leer_telemetria.py — Recibe en la laptop la telemetría UDP que manda la Pi
(~/telemetria_udp.py) y la muestra en consola; opcional: la guarda en CSV.

Uso:
    python scripts/leer_telemetria.py                 # escucha en 5005
    python scripts/leer_telemetria.py --csv corrida.csv
    python scripts/leer_telemetria.py --puerto 5006

Ojo: solo un programa puede escuchar el puerto a la vez. Si LabVIEW tiene
abierto el 5005, cierra el VI o usa otro puerto (y córrelo así en la Pi).
"""
import argparse
import csv
import socket
import time

COLUMNAS = ["seq", "t_pi", "armado", "est", "tc", "px", "py", "yaw", "bno", "od",
            "tL", "tR", "tF", "tB", "srv", "pw", "fr", "bnoM", "ue", "lpm", "temp"]
EST_NOMBRE = {0: "desarmado", 1: "I", 2: "S", 3: "C", 4: "G", 5: "R", 6: "E", 7: "T", -1: "?"}


def a_num(s):
    try:
        f = float(s)
        return int(f) if f.is_integer() else f
    except ValueError:
        return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto", type=int, default=5005)
    ap.add_argument("--csv", help="archivo donde guardar todos los paquetes")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.puerto))
    sock.settimeout(2.0)
    print(f"Escuchando UDP {args.puerto} (Ctrl+C para salir)")

    archivo = open(args.csv, "w", newline="") if args.csv else None
    escritor = csv.writer(archivo) if archivo else None
    if escritor:
        escritor.writerow(COLUMNAS + ["t_laptop"])

    ultimo_seq = None
    perdidos = 0
    try:
        while True:
            try:
                datos, origen = sock.recvfrom(4096)
            except socket.timeout:
                print("... sin datos (¿corre telemetria_udp.py en la Pi y el runtime está activo?)")
                continue
            valores = datos.decode(errors="replace").strip().split(",")
            if len(valores) != len(COLUMNAS):
                print(f"paquete raro ({len(valores)} columnas): {datos[:80]!r}")
                continue
            d = dict(zip(COLUMNAS, map(a_num, valores)))

            if ultimo_seq is not None and d["seq"] > ultimo_seq + 1:
                perdidos += d["seq"] - ultimo_seq - 1
            ultimo_seq = d["seq"]

            if escritor:
                escritor.writerow(valores + [f"{time.time():.3f}"])

            est = EST_NOMBRE.get(d["est"], d["est"])
            if d["armado"]:
                print(f"#{d['seq']:6} est={est} tc={d['tc']} pos=({d['px']},{d['py']}) yaw={d['yaw']} "
                      f"ToF L/R/F/B={d['tL']}/{d['tR']}/{d['tF']}/{d['tB']} srv={d['srv']} pw={d['pw']} "
                      f"temp={d['temp']}°C perdidos={perdidos}")
            else:
                print(f"#{d['seq']:6} desarmado, esperando botón  temp={d['temp']}°C")
    except KeyboardInterrupt:
        pass
    finally:
        if archivo:
            archivo.close()
            print(f"Guardado en {args.csv}")


if __name__ == "__main__":
    main()
