#!/usr/bin/env python3
"""
telemetria_udp.py — Reenvía la telemetría del carro (Fox Robotics) a LabVIEW por UDP.

Corre APARTE del runtime: solo lee el journal de wro-runtime.service y manda
una línea CSV por paquete. No toca el código de carrera.
SOLO PARA PRUEBAS: en competencia el WiFi va apagado y esto no se corre.

Uso (en la Pi):
    python3 telemetria_udp.py 192.168.1.50          # IP de la laptop
    python3 telemetria_udp.py 192.168.1.255 --bcast # broadcast a toda la red
"""
import re
import socket
import subprocess
import sys
import time

PORT = 5005
SERVICIO = "wro-runtime.service"
CAMPOS_ACK = ["tc", "px", "py", "yaw", "bno", "od", "tL", "tR", "tF", "tB",
              "srv", "pw", "fr", "bnoM", "ue", "lpm"]
EST_CODIGO = {"I": 1, "S": 2, "C": 3, "G": 4, "R": 5, "E": 6, "T": 7}
LATIDO_CADA_S = 0.5
KV = re.compile(r"(\w+)=([^,\s*]+)")


def temp_cpu():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read()) / 1000.0
    except Exception:
        return -1.0


def main():
    if len(sys.argv) < 2:
        print("Uso: python3 telemetria_udp.py <IP_laptop> [--bcast]")
        sys.exit(1)
    destino = (sys.argv[1], PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if "--bcast" in sys.argv:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    p = subprocess.Popen(["journalctl", "-u", SERVICIO, "-f", "-n", "0", "-o", "cat"],
                         stdout=subprocess.PIPE, text=True, bufsize=1)
    print(f"Mandando telemetría a {destino[0]}:{PORT} (Ctrl+C para salir)", flush=True)

    seq = 0
    ultimo_latido = 0.0
    for linea in p.stdout:
        ahora = time.time()

        if "RX: ACK" in linea:
            kv = dict(KV.findall(linea.split("RX:", 1)[1]))
            est = EST_CODIGO.get(kv.get("est", ""), -1)
            valores = [kv.get(c, "-1") for c in CAMPOS_ACK]
            armado = 1
        elif linea.startswith("TX(desarmado)"):
            if ahora - ultimo_latido < LATIDO_CADA_S:
                continue
            ultimo_latido = ahora
            est = 0
            valores = ["-1"] * len(CAMPOS_ACK)
            armado = 0
        else:
            continue

        msg = f"{seq},{ahora:.3f},{armado},{est}," + ",".join(valores) + f",{temp_cpu():.1f}"
        try:
            sock.sendto(msg.encode(), destino)
        except OSError:
            pass
        seq += 1


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
