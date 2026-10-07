#!/usr/bin/env python3
"""Reporte de una run del carro a partir del journal de la Pi (wro-runtime.service).

Uso (todo en tu PC, sin instalar nada, solo Python 3):

  python scripts/reporte_run.py --pi                     # baja el journal por ssh y reporta la ULTIMA run
  python scripts/reporte_run.py --pi --html reporte.html # ademas guarda un HTML para abrir en el navegador
  python scripts/reporte_run.py --pi --host 192.168.50.1 # si la Pi esta en otra IP
  python scripts/reporte_run.py orillas976_journal.log   # desde un journal ya guardado
  python scripts/reporte_run.py run.log --run -2         # la penultima run del archivo (1 = primera, -1 = ultima)
  ssh -p 443 user@100.107.87.42 "journalctl -u wro-runtime.service -b 0 --no-pager" | python scripts/reporte_run.py -

Fuente: los ACK:V2 que el ESP32 manda a la Pi (`[SERIAL] RX: ACK:V2,...`), mas unas
pocas lineas de la Pi. Nada de esto toca el carro: solo lee logs.
"""
import argparse
import re
import subprocess
import sys
from datetime import datetime

HOST_DEFAULT = "100.107.87.42"   # Tailscale; en LAN local suele ser 192.168.50.1


def ssh_cmd(host):
    # BatchMode: nunca pide password (falla en vez de colgarse); ConnectTimeout: falla rapido si no hay Pi
    return ["ssh", "-p", "443", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "user@" + host,
            "journalctl -u wro-runtime.service -b 0 --no-pager"]

EST_NOMBRE = {"I": "INICIO", "S": "SIGUIENDO", "C": "CRUCERO", "G": "GIRO/MANIOBRA",
              "R": "RECUPERANDO", "E": "ESTACIONANDO", "T": "TERMINANDO"}
# pns = subfase del scan del estacionamiento PARALELO (ver PurePursuit.ino, case ESTACIONANDO fase 0)
PNS_NOMBRE = {0: "buscando poste 1", 1: "sobre poste 1", 2: "hueco, buscando poste 2",
              3: "sobre poste 2", 4: "avance extra"}
PNF_NOMBRE = {0: "scan", 1: "avance", 2: "coast", 3: "rev-swing", 4: "rev", 7: "STOP",
              8: "contravuelta", 9: "coast", 11: "acomodo", 20: "retorno", 21: "media vuelta"}

RE_TS = re.compile(r"^([A-Z][a-z]{2}\s+\d+\s+\d\d:\d\d:\d\d\.\d+)")
RE_ACK = re.compile(r"ACK:V2,(.*)$")


def parse_ts(line):
    m = RE_TS.match(line)
    if not m:
        return None
    try:
        return datetime.strptime("2026 " + re.sub(r"\s+", " ", m.group(1)), "%Y %b %d %H:%M:%S.%f")
    except ValueError:
        return None


def num(v, default=None):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def cargar_lineas(args):
    if args.pi:
        r = subprocess.run(ssh_cmd(args.host), capture_output=True, text=True, encoding="utf-8", errors="replace")
        if r.returncode != 0 or not r.stdout.strip():
            sys.exit("ssh fallo (rc=%s): %s\nSi hace timeout, la Pi probablemente perdio el hotspot."
                     % (r.returncode, r.stderr.strip()[:300]))
        return r.stdout.splitlines()
    if args.archivo in (None, "-"):
        return sys.stdin.read().splitlines()
    with open(args.archivo, encoding="utf-8", errors="replace") as f:
        return f.read().splitlines()


def cortar_run(lineas, idx):
    """(lineas de la run, numero orillasNNN o None, cuantas runs hay)."""
    gos = [i for i, l in enumerate(lineas) if "[GPIO] GO" in l]
    if not gos:
        sys.exit("No hay ningun '[GPIO] GO' en el log: no se armo ninguna run.")
    try:
        ini = gos[idx if idx < 0 else idx - 1]
    except IndexError:
        sys.exit("--run %d fuera de rango (hay %d runs)." % (idx, len(gos)))
    fin = len(lineas)
    for g in gos:
        if g > ini:
            fin = g
            break
    nro = None
    for l in reversed(lineas[:ini]):
        m = re.search(r"orillas(\d+)\.avi", l)
        if m:
            nro = m.group(1)
            break
    return lineas[ini:fin], nro, len(gos)


def parse_acks(lineas):
    def recolectar(filtro):
        out = []
        for l in lineas:
            if not filtro(l):
                continue
            m = RE_ACK.search(l)
            ts = parse_ts(l)
            if not m or ts is None:
                continue
            d = dict(kv.split("=", 1) for kv in m.group(1).split(",") if "=" in kv)
            d["_ts"] = ts
            out.append(d)
        return out
    acks = recolectar(lambda l: "[SERIAL] RX:" in l)
    return acks or recolectar(lambda l: "| RX:" in l)  # respaldo: ACK pegado al TX


def fmt_t(t):
    return "%.1f" % t


def par(a, b, nd=0):
    def f(x):
        return "-" if x is None else ("%.*f" % (nd, x))
    return f(a) if a == b else "%s->%s" % (f(a), f(b))


def segmentos_estado(acks, t0):
    segs = []
    for a in acks:
        est = a.get("est", "?")
        if segs and segs[-1]["est"] == est:
            segs[-1]["acks"].append(a)
        else:
            segs.append({"est": est, "acks": [a]})
    # un est= suelto de 1 solo ACK es ruido/parpadeo: se funde con el anterior
    limpio = []
    for s in segs:
        if limpio and len(s["acks"]) < 2 and s is not segs[-1]:
            limpio[-1]["acks"].extend(s["acks"])
        elif limpio and limpio[-1]["est"] == s["est"]:
            limpio[-1]["acks"].extend(s["acks"])
        else:
            limpio.append(s)
    filas = []
    for i, s in enumerate(limpio, 1):
        a0, a1 = s["acks"][0], s["acks"][-1]
        ti = (a0["_ts"] - t0).total_seconds()
        tf = (a1["_ts"] - t0).total_seconds()
        extra = ""
        if s["est"] == "G":
            fases = sorted({a.get("fase") for a in s["acks"] if a.get("fase", "-1") != "-1"})
            extra = "fase %s%s" % ("/".join(fases) or "-", " REV" if a0.get("rev") == "1" else "")
        elif s["est"] == "E":
            pnfs = [a.get("pnf") for a in s["acks"] if a.get("pnf")]
            extra = "pnf %s->%s" % (pnfs[0], pnfs[-1]) if pnfs else ""
        filas.append([i, EST_NOMBRE.get(s["est"], s["est"]), fmt_t(ti), fmt_t(tf - ti),
                      "%s->%s" % (a0.get("tc", "-"), a1.get("tc", "-")),
                      par(num(a0.get("ang")), num(a1.get("ang")), 1),
                      par(num(a0.get("dL")), num(a1.get("dL"))),
                      par(num(a0.get("dR")), num(a1.get("dR"))),
                      par(num(a0.get("dF")), num(a1.get("dF"))), extra])
    return filas


def esquinas(acks, t0):
    filas, tc_prev = [], None
    for a in acks:
        tc = a.get("tc")
        if tc != tc_prev and tc is not None:
            filas.append([tc, fmt_t((a["_ts"] - t0).total_seconds()), EST_NOMBRE.get(a.get("est"), a.get("est")),
                          a.get("dir", "?"), a.get("ang", "-"), a.get("dL", "-"), a.get("dR", "-"), a.get("dF", "-")])
            tc_prev = tc
    return filas


def pared_ext(a):
    d = a.get("dir")
    if d == "L":
        return a.get("dL")
    if d == "R":
        return a.get("dR")
    return None


def estacionamiento(acks, t0):
    """(tablas, resumen) del tramo est=E; ([], []) si la run no estaciono."""
    ea = [a for a in acks if a.get("est") == "E" and a.get("pnf") is not None]
    if not ea:
        return [], []
    te0 = ea[0]["_ts"]
    paralelo = any("pns" in a for a in ea)   # el modo de punta no manda pns

    def rel(a):
        return (a["_ts"] - t0).total_seconds()

    cols = ["t(s)", "t en E", "pnf", "pns", "pared ext", "pnx", "pnb", "dL", "dR", "dF", "ang",
            "tp/tl", "at", "vel", "pnq", "pnd", "pnt"]
    filas, prev = [], None
    for a in ea:
        k = (a.get("pnf"), a.get("pns"))
        if k == prev:
            continue
        prev = k
        pnf, pns = a.get("pnf"), a.get("pns")
        pnf_txt = pnf if not paralelo or num(pnf) is None or int(num(pnf)) not in PNF_NOMBRE else "%s %s" % (pnf, PNF_NOMBRE[int(num(pnf))])
        pns_txt = "-" if pns is None else ("%s %s" % (pns, PNS_NOMBRE.get(int(num(pns, -1)), "")) if int(num(pns, -1)) in PNS_NOMBRE else pns)
        filas.append([fmt_t(rel(a)), fmt_t((a["_ts"] - te0).total_seconds()), pnf_txt, pns_txt,
                      pared_ext(a) or "-", a.get("pnx", "-"), a.get("pnb", "-"), a.get("dL", "-"), a.get("dR", "-"),
                      a.get("dF", "-"), a.get("ang", "-"),
                      "%s/%s" % (a.get("tp", "-"), a.get("tl", "-")), a.get("at", "-"), a.get("vel", "-"),
                      a.get("pnq", "-"), a.get("pnd", "-"), a.get("pnt", "-")])

    def primero(pred):
        return next((a for a in ea if pred(a)), None)

    def linea(a, etiqueta):
        if a is None:
            return [etiqueta, "no ocurrio", "", "", ""]
        return [etiqueta, "t=%s s (%s s en E)" % (fmt_t(rel(a)), fmt_t((a["_ts"] - te0).total_seconds())),
                "pared ext=%s (base pnb=%s, sonar pnx=%s)" % (pared_ext(a) or "-", a.get("pnb", "-"), a.get("pnx", "-")),
                "dF=%s" % a.get("dF", "-"), "ang=%s" % a.get("ang", "-")]

    p1 = primero(lambda a: a.get("pns") == "1")
    hu = primero(lambda a: a.get("pns") == "2")
    p2 = primero(lambda a: a.get("pns") == "3")
    # lanzamiento = primera fase de maniobra DESPUES de que el scan (pnf=0) empezo;
    # las fases 20-23 son el retorno/U-turn previo, no la maniobra.
    i_scan = next((i for i, a in enumerate(ea) if a.get("pnf") == "0"), None)
    lz = None
    if i_scan is not None:
        lz = next((a for a in ea[i_scan:] if a.get("pnf") not in ("0", "20", "21", "22", "23", None)), None)
    fin = ea[-1]
    resumen = [["Entrada a ESTACIONANDO", "t=%s s" % fmt_t(rel(ea[0])),
                "pared ext=%s (pnx=%s, pnb=%s)" % (pared_ext(ea[0]) or "-", ea[0].get("pnx", "-"), ea[0].get("pnb", "-")),
                "dF=%s" % ea[0].get("dF", "-"), "ang=%s" % ea[0].get("ang", "-")],
               ] + ([linea(p1, "Poste 1 (pns=1)"), linea(hu, "Hueco (pns=2)"),
                     linea(p2, "Poste 2 (pns=3)  <- pared a la que va"), linea(lz, "Lanzamiento (pnf!=0)")]
                    if paralelo else []) + [
               ["Ultimo ACK en E", "t=%s s (%s s en E)" % (fmt_t(rel(fin)), fmt_t((fin["_ts"] - te0).total_seconds())),
                "pnf=%s pns=%s" % (fin.get("pnf"), fin.get("pns", "-")),
                "dL=%s dR=%s dF=%s" % (fin.get("dL"), fin.get("dR"), fin.get("dF")), "ang=%s" % fin.get("ang")]]
    # veredicto inferido: pnf=7 (STOP) sin haber llegado a lanzamiento = corte por TIMEOUT/FRENTE
    veredicto = ""
    if not paralelo:
        veredicto = "Modo PUNTA (sin subfases pns): termina en pnf=%s." % fin.get("pnf")
    elif fin.get("pnf") == "7" and lz is None:
        dur = (fin["_ts"] - te0).total_seconds()
        veredicto = ("CORTE SIN CAJON: llego a pnf=7 sin lanzamiento, %.1f s en E "
                     "(PARK_TIMEOUT_MS=10 s%s). Ultima subfase del scan: pns=%s (%s)."
                     % (dur, " -> fue TIMEOUT" if dur >= 9.3 else "; tan corto que huele a FRENTE cerca",
                        fin.get("pns", "-"), PNS_NOMBRE.get(int(num(fin.get("pns"), -1)), "?")))
    elif p2 is None and lz is None:
        veredicto = "El scan nunca vio el poste 2 (ultima subfase pns=%s)." % fin.get("pns", "-")
    else:
        veredicto = "Termina en pnf=%s (%s); el log/ACKs acaban ahi." % (fin.get("pnf"), PNF_NOMBRE.get(int(num(fin.get("pnf"), -1)), "?"))
    tablas = [("Estacionamiento: eventos clave", ["evento", "cuando", "pared", "frente", "rumbo"], resumen),
              ("Estacionamiento: cada cambio de fase (pnf) / subfase (pns)", cols, filas)]
    return tablas, veredicto


def eventos_pi(lineas, t0):
    pat = re.compile(r"\[(INICIO|GPIO|PARK|CAM[^\]]*)\]\s*(.*)")
    filas, vistos = [], set()
    for l in lineas:
        m = pat.search(l)
        if not m:
            continue
        txt = m.group(0)
        if m.group(1).startswith("CAM") and not re.search(r"negr|black|ciega|ruido", txt, re.I):
            continue
        if txt in vistos:
            continue
        vistos.add(txt)
        ts = parse_ts(l)
        filas.append([fmt_t((ts - t0).total_seconds()) if ts else "-", txt[:150]])
    return filas[:25]


def construir(lineas, nro, n_runs):
    acks = parse_acks(lineas)
    t0 = parse_ts(lineas[0])
    if not acks:
        sys.exit("Esa run no tiene ACK:V2 del ESP (el ESP no contesto o el log esta truncado).")
    ult, primero = acks[-1], acks[0]
    dur = (ult["_ts"] - t0).total_seconds()
    huecos = [(b["_ts"] - a["_ts"]).total_seconds() for a, b in zip(acks, acks[1:])]
    n_giros = sum("Giro detectado" in l for l in lineas)
    est_cnt = {}
    for a in acks:
        est_cnt[a.get("est")] = est_cnt.get(a.get("est"), 0) + 1
    resumen = [
        ["Run", "orillas%s" % nro if nro else "?"],
        ["Inicio (GO)", t0.strftime("%H:%M:%S") + " (hora de la Pi; ojo, sin RTC puede saltar)"],
        ["Duracion GO -> ultimo ACK", "%.1f s" % dur],
        ["ACKs recibidos", "%d (%.1f/s)" % (len(acks), len(acks) / dur if dur else 0)],
        ["Hueco mas largo sin ACK", "%.1f s%s" % (max(huecos or [0]), "  <- revisa: ESP mudo" if max(huecos or [0]) > 3 else "")],
        ["Sentido (dir)", {"L": "L (CCW, pared exterior = izquierda)", "R": "R (CW, pared exterior = derecha)"}
            .get(ult.get("dir"), ult.get("dir", "?"))],
        ["Esquinas (tc)", "tc=%s   ('Giro detectado' en la Pi: %d veces; 12 = vuelta completa, 13 = el U-turn del park)" % (ult.get("tc", "?"), n_giros)],
        ["Estado final", "%s  (dL=%s dR=%s dF=%s ang=%s)" % (EST_NOMBRE.get(ult.get("est"), ult.get("est")),
                                                             ult.get("dL"), ult.get("dR"), ult.get("dF"), ult.get("ang"))],
        ["ACKs por estado", "  ".join("%s=%d" % (EST_NOMBRE.get(k, k), v) for k, v in est_cnt.items())],
    ]
    tablas = [("Resumen", ["dato", "valor"], resumen)]
    tablas.append(("Estados del ESP (en orden)", ["#", "estado", "t ini(s)", "dur(s)", "tc", "ang", "dL", "dR", "dF", "extra"],
                   segmentos_estado(acks, t0)))
    tablas.append(("Esquinas (cada vez que sube tc)", ["tc", "t(s)", "estado", "dir", "ang", "dL", "dR", "dF"], esquinas(acks, t0)))
    park_tablas, veredicto = estacionamiento(acks, t0)
    tablas.extend(park_tablas)
    ev = eventos_pi(lineas, t0)
    if ev:
        tablas.append(("Eventos de la Pi", ["t(s)", "linea"], ev))
    return tablas, veredicto


def a_texto(tablas, veredicto):
    out = []
    for titulo, cols, filas in tablas:
        out.append("\n## " + titulo)
        filas = [[str(c) for c in f] for f in filas]
        anchos = [max(len(str(c)), *(len(f[i]) for f in filas)) if filas else len(str(c)) for i, c in enumerate(cols)]
        out.append("  ".join(str(c).ljust(w) for c, w in zip(cols, anchos)))
        out.append("  ".join("-" * w for w in anchos))
        for f in filas:
            out.append("  ".join(c.ljust(w) for c, w in zip(f, anchos)))
        if titulo.startswith("Estacionamiento: eventos") and veredicto:
            out.append("\n>> " + veredicto)
    return "\n".join(out)


def a_html(tablas, veredicto, titulo):
    css = ("body{font:14px system-ui,sans-serif;margin:24px;background:#fff;color:#111}"
           "table{border-collapse:collapse;margin:8px 0 24px}th,td{border:1px solid #ccc;padding:3px 8px;text-align:left}"
           "th{background:#f0f0f0}tr:nth-child(even) td{background:#fafafa}.v{padding:8px 12px;background:#fff3cd;border:1px solid #e0c060}"
           "@media(prefers-color-scheme:dark){body{background:#151515;color:#eee}th{background:#2a2a2a}td,th{border-color:#444}"
           "tr:nth-child(even) td{background:#1c1c1c}.v{background:#3a3210;border-color:#7a6a20}}")
    esc = lambda s: str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    p = ["<!doctype html><meta charset=utf-8><title>%s</title><style>%s</style><h1>%s</h1>" % (esc(titulo), css, esc(titulo))]
    for t, cols, filas in tablas:
        p.append("<h2>%s</h2>" % esc(t))
        if t.startswith("Estacionamiento: eventos") and veredicto:
            p.append("<p class=v>%s</p>" % esc(veredicto))
        p.append("<table><tr>%s</tr>" % "".join("<th>%s</th>" % esc(c) for c in cols))
        for f in filas:
            p.append("<tr>%s</tr>" % "".join("<td>%s</td>" % esc(c) for c in f))
        p.append("</table>")
    return "".join(p)


def main():
    ap = argparse.ArgumentParser(description="Reporte de una run a partir del journal de la Pi.")
    ap.add_argument("archivo", nargs="?", help="journal guardado, o '-' para stdin")
    ap.add_argument("--pi", action="store_true", help="bajar el journal por ssh (Tailscale, puerto 443)")
    ap.add_argument("--host", default=HOST_DEFAULT, help="IP de la Pi (default %s; LAN: 192.168.50.1)" % HOST_DEFAULT)
    ap.add_argument("--run", type=int, default=-1, help="que run del log: -1 ultima (default), 1 primera, -2 penultima...")
    ap.add_argument("--html", metavar="ARCHIVO", help="ademas guardar el reporte como HTML")
    args = ap.parse_args()
    if not args.pi and args.archivo is None and sys.stdin.isatty():
        ap.error("pasa un archivo, '-' para stdin, o --pi")
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    lineas = cargar_lineas(args)
    run_lineas, nro, n_runs = cortar_run(lineas, args.run)
    tablas, veredicto = construir(run_lineas, nro, n_runs)
    titulo = "Reporte run %s  (%d runs en este log)" % ("orillas" + nro if nro else "?", n_runs)
    print("# " + titulo)
    print(a_texto(tablas, veredicto))
    if args.html:
        with open(args.html, "w", encoding="utf-8") as f:
            f.write(a_html(tablas, veredicto, titulo))
        print("\nHTML guardado en", args.html)


if __name__ == "__main__":
    main()
