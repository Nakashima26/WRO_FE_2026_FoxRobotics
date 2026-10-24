"""SeguirRuta.ino (rama parking) corriendo en el gemelo: el mismo firmware que el carro,
con el Vehicle calibrado (params.py) como física. Se le manda lo mismo que mandó
ruta_bt.py en una vuelta real (ruta, mapa, salida y todos los parámetros, del log)
y se compara lo que hizo el gemelo contra lo que hizo el carro.

  python twin/tools/calib/seguir_ruta_sil/sr_sil.py ../WRO2026/logs_tof/ruta_vuelta_20261024_130029.txt
      [--ino ../WRO2026/src/ESP32/SeguirRuta/SeguirRuta.ino] [--png salida.png] [--set clave=valor]

Desde src/RASPI/cam. Sensores ideales: el encoder cuenta lo que gira la llanta en el
modelo (v_wheel: patina al arrancar en reversa y no cuenta lo que resbala frenando) y el
BNO da el rumbo del modelo (escala medida 0.9998). Sin ToF: SeguirRuta corre como con
--pared 0. Si en la vuelta real salidaPared movió la salida, el firmware del gemelo arranca
creyendo esa misma salida ("# start" del log) y el carro del gemelo en la de RTS.
Compila con zig (pip install ziglang) o FOX_CXX.
"""
import argparse
import ctypes
import hashlib
import math
import os
import re
import shutil
import subprocess
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
CAM = os.path.abspath(os.path.join(AQUI, "..", "..", "..", "..", ".."))
sys.path.insert(0, CAM)
from pure_pursuit.twin.params import TwinParams  # noqa: E402
from pure_pursuit.twin.vehicle import Vehicle  # noqa: E402

REPO = os.path.abspath(os.path.join(CAM, "..", "..", ".."))
INO_DEFAULT = os.path.join(os.path.dirname(REPO), "WRO2026", "src", "ESP32", "SeguirRuta", "SeguirRuta.ino")
PWMA, A1, A2, SERVO_PIN = 23, 18, 19, 13
SIGNO_ENC = -1
_CARGADAS = []


def prototipos(ino):
    """Lo que hace el preprocesador de Arduino: declarar las funciones antes de usarlas
    (solo las de tipos simples; las que usan structs del sketch van después de ellos)."""
    pat = re.compile(r"^([A-Za-z_][\w \*&]*?[ \*&]+)(\w+)\s*\(([^)]*)\)\s*\{", re.M)
    out = []
    for tipo, nombre, args in pat.findall(ino):
        if tipo.strip() in ("else", "return") or nombre in ("setup", "loop"):
            continue
        if re.search(r"\b(Tof|Parametro|Fase)\b", tipo + args):
            continue
        out.append(f"{tipo.strip()} {nombre}({args});")
    return "\n".join(out)


def compilar(ino_path):
    ino = open(ino_path, encoding="utf-8").read()
    gen = '#include "Arduino.h"\n#include "Wire.h"\n#include "VL53L1X.h"\n#include "BluetoothSerial.h"\n' \
          '#include "driver/pulse_cnt.h"\nclass Stream;\n' + prototipos(ino) + "\n#line 1 \"SeguirRuta.ino\"\n" + ino
    core = open(os.path.join(AQUI, "shim", "sr_core.cpp"), encoding="utf-8").read()
    h = hashlib.sha1((gen + core).encode()).hexdigest()[:12]
    build = os.path.join(AQUI, "_build")
    os.makedirs(build, exist_ok=True)
    dll = os.path.join(build, f"sr_{h}.dll")
    if os.path.exists(dll):
        return dll
    src = os.path.join(build, f"sr_{h}.cpp")
    open(src, "w", encoding="utf-8").write(gen)
    cxx = os.environ.get("FOX_CXX", f"{sys.executable} -m ziglang c++").split()
    cmd = cxx + ["-std=c++17", "-O2", "-shared", "-Wno-everything", "-target", "x86_64-windows-gnu",
                 "-I" + os.path.join(AQUI, "shim"), src, os.path.join(AQUI, "shim", "sr_core.cpp"), "-o", dll]
    subprocess.run(cmd, check=True, stdin=subprocess.DEVNULL)
    return dll


def leer_log(f):
    L = open(f, encoding="utf-8").read().splitlines()
    d = {"R": [], "P": [], "par": {}, "fin": None, "salida": None, "start": None, "mapa": None, "pc": {}}
    for lin in L:
        if lin.startswith("R,"):
            d["R"].append([float(v) for v in lin.split(",")[1:3]])
        elif lin.startswith("P,"):
            d["P"].append([float(v) for v in lin.split(",")[1:10]])
        elif lin.startswith("# PC tipo="):
            d["pc"] = dict(kv.split("=", 1) for kv in lin[5:].split() if "=" in kv)
        elif lin.startswith("# par ") and "=" in lin:
            k, v = lin[6:].split("=", 1)
            d["par"][k] = v
        elif lin.startswith("# salida x="):
            d["salida"] = lin
        elif lin.startswith("# start x="):
            d["start"] = lin
        elif lin.startswith("# fin "):
            d["fin"] = lin
        elif lin.startswith("# mapa exterior"):
            d["mapa"] = [float(v) for v in re.findall(r"-?\d+(?:\.\d+)?", lin)]
    d["P"] = np.array(d["P"])
    return d


def campos(lin):
    return {k: float(v) for k, v in re.findall(r"(\w+)=(-?[\d.]+)", lin)}


def comandos(d, sets):
    pc = d["pc"]
    ruta, paso, cerrada = d["R"], float(pc["paso"]), int(pc["cerrada"])
    out = [f"RTB,{len(ruta)},{paso:.3f},{cerrada}\n"]
    for i in range(0, len(ruta), 10):
        out.append(f"RTP,{i}," + ",".join(f"{int(round(x))},{int(round(y))}" for x, y in ruta[i:i + 10]) + "\n")
    out.append(f"RTE,{len(ruta)}\n")
    if d["mapa"]:
        out.append("RTM," + ",".join(f"{v:.0f}" for v in d["mapa"]) + "\n")
    s, st = campos(d["salida"]), campos(d["start"])
    # El firmware arranca creyendo lo que creyó el carro (salidaPared ya aplicada).
    out.append(f"RTS,{st['x']:.0f},{st['y']:.0f},{st['th']:.1f},{s['pwm']:.0f}," + d["salida"].split("pwm=")[1].split("/")[1].split()[0]
               + f",{s['vueltas']:.0f},{s['lookahead']:.0f},{s['centro']:.0f},{s['kd']:.3f},{s['pared']:.0f},"
               f"{s['encCurva']:.3f},{s['bnoEscala']:.4f},{s['anticipo']:.3f}\n")
    par = dict(d["par"])
    par["salidaPared"] = "0"
    par.update(sets)
    for k, v in par.items():
        out.append(f"SET,{k},{v}\n")
    return out, (s["x"], s["y"], s["th"])


def correr(dll, d, params, sets, us_por_millis=25):
    # Una copia por corrida: el sketch tiene estado global y una DLL ya cargada no se reinicia.
    copia = dll.replace(".dll", f"_{os.getpid()}_{len(_CARGADAS)}.dll")
    shutil.copyfile(dll, copia)
    lib = ctypes.CDLL(copia)
    _CARGADAS.append(lib)
    lib.sr_duty.restype = ctypes.c_uint32
    lib.sr_now_us.restype = ctypes.c_uint64
    lib.sr_set_yaw.argtypes = [ctypes.c_float]
    lib.sr_set_us_por_millis.argtypes = [ctypes.c_uint64]
    lib.sr_set_us_por_millis(us_por_millis)
    cmds, (sx, sy, sth) = comandos(d, sets)
    centro = float(campos(d["salida"])["centro"])
    veh = Vehicle(params, sx, sy, 90.0 - sth, servo_deg=centro)
    cpmm = float(d["par"].get("cuentasPorMm", 5.45))
    st = {"rueda_mm": 0.0, "verdad": []}

    def tick(now_us):
        a1, a2 = lib.sr_pin(A1), lib.sr_pin(A2)
        duty = lib.sr_duty(PWMA)
        pulso = lib.sr_duty(SERVO_PIN) * 20000.0 / 65535.0
        servo = (pulso - 500.0) * 180.0 / 2000.0 if pulso > 0 else centro
        if a1 and a2:
            direccion, pwm = 1, 0.0               # freno corto del TB6612
        elif a1:
            direccion, pwm = 1, float(duty)
        elif a2:
            direccion, pwm = -1, float(duty)
        else:
            direccion, pwm = 0, 0.0
        veh.step(0.001, servo, pwm, direccion)
        st["rueda_mm"] += veh.v_wheel * 0.001
        lib.sr_set_count(int(round(SIGNO_ENC * st["rueda_mm"] * cpmm)))
        lib.sr_set_yaw(((veh.heading_deg + 180.0) % 360.0) - 180.0)
        if now_us % 20000 == 0:
            st["verdad"].append((now_us / 1e6, veh.x, veh.y, 90.0 - veh.heading_deg, veh.v, servo, pwm))

    cb = ctypes.CFUNCTYPE(None, ctypes.c_uint64)(tick)
    lib.sr_set_tick_cb(cb)
    buf = ctypes.create_string_buffer(1 << 20)
    salida = []

    def tomar():
        n = lib.sr_take_output(buf, len(buf))
        if n:
            salida.extend(buf.raw[:n].decode("utf-8", "replace").splitlines())

    lib.sr_setup()
    for c in cmds:
        lib.sr_bt_send(c.encode())
        for _ in range(5):
            lib.sr_loop()
    lib.sr_bt_send(b"g")
    t_g = lib.sr_now_us() / 1e6
    fin = False
    while lib.sr_now_us() / 1e6 - t_g < 200:
        lib.sr_loop()
        tomar()
        if not fin and any(l.startswith("# fin ") for l in salida[-20:]):
            fin = True
            break
    tomar()
    V = np.array(st["verdad"])
    V[:, 0] -= t_g
    return salida, V


def comparar(d, salida, V, nombre):
    P_sim = np.array([[float(v) for v in l.split(",")[1:10]] for l in salida if l.startswith("P,")])
    P = d["P"]
    fin_sim = next((l for l in salida if l.startswith("# fin ")), None)
    sets = [l for l in salida if l.startswith("# SET:") or "NO ARRANCA" in l or "RECHAZADA" in l]
    for l in sets:
        print("  !!", l)
    fr, fs = campos(d["fin"] or ""), campos(fin_sim or "")
    print(f"\n{nombre}")
    print(f"  {'':28s}{'carro':>12s}{'gemelo':>12s}")

    def fila(txt, a, b, fmt="{:.1f}"):
        print(f"  {txt:28s}{fmt.format(a):>12s}{fmt.format(b):>12s}")

    fila("tiempo (s)", P[-1, 0] / 1000, P_sim[-1, 0] / 1000, "{:.2f}")
    m_r = re.search(r"en ([\d.]+) s", d["fin"] or "")
    m_s = re.search(r"en ([\d.]+) s", fin_sim or "")
    if m_r and m_s:
        fila("tiempo al frenar (s)", float(m_r.group(1)), float(m_s.group(1)), "{:.1f}")
    for k in ("recorrido",):
        if k in fr and k in fs:
            fila(f"{k} encoder (mm)", fr[k], fs[k], "{:.0f}")
    fila("fin odometría x (mm)", fr.get("x", np.nan), fs.get("x", np.nan), "{:.0f}")
    fila("fin odometría y (mm)", fr.get("y", np.nan), fs.get("y", np.nan), "{:.0f}")
    fila("fin odometría th (°)", fr.get("th", np.nan), fs.get("th", np.nan), "{:.1f}")
    rr = re.search(r"lateral max (\d+) mm rms (\d+)", d["fin"] or "")
    rs = re.search(r"lateral max (\d+) mm rms (\d+)", fin_sim or "")
    if rr and rs:
        print(f"  {'lateral max / rms (mm)':28s}{rr.group(1) + ' / ' + rr.group(2):>12s}{rs.group(1) + ' / ' + rs.group(2):>12s}")
    fb_r = re.search(r"frenando (\d+) mm", d["fin"] or "")
    fb_s = re.search(r"frenando (\d+) mm", fin_sim or "")
    if fb_r and fb_s:
        fila("frenando (mm, encoder)", float(fb_r.group(1)), float(fb_s.group(1)), "{:.0f}")
    corriendo = P[:, 7] > 0
    corr_s = P_sim[:, 7] > 0
    fila("servo medio", P[corriendo, 6].mean(), P_sim[corr_s, 6].mean())
    fila("servo std", P[corriendo, 6].std(), P_sim[corr_s, 6].std())
    fila("PWM medio", P[corriendo, 7].mean(), P_sim[corr_s, 7].mean(), "{:.0f}")
    # Mismo instante: diferencia de la odometría (lo que cree cada uno).
    t = P[:, 0]
    xs = np.interp(t, P_sim[:, 0], P_sim[:, 1])
    ys = np.interp(t, P_sim[:, 0], P_sim[:, 2])
    dpos = np.hypot(xs - P[:, 1], ys - P[:, 2])
    print(f"  odometría carro vs gemelo al mismo tiempo: media {dpos.mean():.0f} mm, máx {dpos.max():.0f} mm")
    # Por avance (para que el tiempo no cuente): servo en el mismo punto de la ruta.
    vx, vy, vth = V[-1, 1], V[-1, 2], V[-1, 3]
    print(f"  gemelo de verdad al final: x={vx:.0f} y={vy:.0f} th={vth:.1f}  (odometría {fs.get('x', 0):.0f}, "
          f"{fs.get('y', 0):.0f}: el encoder/BNO del gemelo se equivocó {math.hypot(vx - fs.get('x', 0), vy - fs.get('y', 0)):.0f} mm)")
    return P_sim


def graficar(d, P_sim, V, png, titulo):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    R, P = np.array(d["R"]), d["P"]
    fig, ax = plt.subplots(1, 2, figsize=(14, 6.5))
    ax[0].plot(R[:, 0], R[:, 1], "k:", lw=1, label="ruta")
    ax[0].plot(P[:, 1], P[:, 2], "C0", lw=1.2, label="carro (odometría)")
    ax[0].plot(P_sim[:, 1], P_sim[:, 2], "C1", lw=1.2, label="gemelo (odometría)")
    ax[0].plot(V[:, 1], V[:, 2], "C1--", lw=0.8, label="gemelo (de verdad)")
    ax[0].set_aspect("equal")
    ax[0].legend(fontsize=8)
    ax[0].set_title(titulo, fontsize=9)
    ax[1].plot(P[:, 0] / 1000, P[:, 6], "C0", lw=0.8, label="servo carro")
    ax[1].plot(P_sim[:, 0] / 1000, P_sim[:, 6], "C1", lw=0.8, label="servo gemelo")
    ax[1].plot(P[:, 0] / 1000, P[:, 7] / 3, "C0", lw=0.6, alpha=0.5, label="PWM/3 carro")
    ax[1].plot(P_sim[:, 0] / 1000, P_sim[:, 7] / 3, "C1", lw=0.6, alpha=0.5, label="PWM/3 gemelo")
    ax[1].set_xlabel("s")
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(png, dpi=110)
    print("  ->", png)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--ino", default=INO_DEFAULT)
    ap.add_argument("--png", help="carpeta o archivo para la gráfica")
    ap.add_argument("--set", action="append", default=[], help="clave=valor de SeguirRuta encima del log")
    ap.add_argument("--us-por-millis", type=int, default=25)
    a = ap.parse_args()
    dll = compilar(a.ino)
    sets = dict(kv.split("=", 1) for kv in a.set)
    for f in a.logs:
        d = leer_log(f)
        nombre = os.path.basename(f)
        salida, V = correr(dll, d, TwinParams(), sets, a.us_por_millis)
        P_sim = comparar(d, salida, V, nombre)
        if a.png:
            png = a.png if a.png.endswith(".png") and len(a.logs) == 1 else \
                os.path.join(a.png, nombre.replace(".txt", "_gemelo.png"))
            graficar(d, P_sim, V, png, nombre)


if __name__ == "__main__":
    main()
