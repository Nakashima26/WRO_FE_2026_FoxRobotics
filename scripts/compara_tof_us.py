#!/usr/bin/env python3
"""Compara ToF contra HC-SR04 en una run, a partir de los ACK:V2 del journal de la Pi.

PurePursuit.ino (desde 2026-11-16) manda en cada ACK los ToF (tL, tR, tF, en mm)
y los HC-SR04 (uL, uR, uF, en mm desde el sensor, -1 = sin eco). Este script dice,
por sensor y por rango de distancia, cuánto difieren, para decidir con datos si el
ultrasónico sirve como comprobación del ToF (pared negra, lejos) y con qué umbral.

Uso (solo lee logs, no toca el carro):

  python scripts/compara_tof_us.py run_journal.log
  ssh -p 443 user@<pi> "journalctl -u wro-runtime.service -b 0 --no-pager" | python scripts/compara_tof_us.py -
  python scripts/compara_tof_us.py run.log --umbral 60

Diferencia = ToF − (ultrasónico llevado al punto del ToF). El sonar lateral va 28.3 mm
más afuera que el ToF (más cerca de la pared) y el frontal 6.2 mm más adelante
(config.SENSOR_MOUNTS), así que se le suma ese offset antes de comparar.
"""
import argparse
import re
import statistics
import sys

RE_ACK = re.compile(r"ACK:V2,(.*)$")
PARES = (("L", "tL", "uL", 28.3), ("R", "tR", "uR", 28.3), ("F", "tF", "uF", 6.2))
RANGOS = ((0, 300), (300, 600), (600, 900), (900, 1500), (1500, 99999))


def acks(lineas):
    for l in lineas:
        m = RE_ACK.search(l)
        if not m:
            continue
        d = {}
        for p in m.group(1).split(","):
            if "=" in p:
                k, v = p.split("=", 1)
                d[k.strip()] = v.strip().split("*")[0]
        yield d


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def pct(n, tot):
    return "%5.1f %%" % (100.0 * n / tot) if tot else "    -  "


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("journal", help="archivo del journal, o - para stdin")
    ap.add_argument("--umbral", type=float, default=80.0,
                    help="mm de diferencia a partir de los que se cuentan como desacuerdo (def. 80)")
    a = ap.parse_args()
    f = sys.stdin if a.journal == "-" else open(a.journal, encoding="utf-8", errors="replace")
    datos = list(acks(f))
    con_us = [d for d in datos if "uL" in d]
    print("ACKs: %d, con campos uL/uR/uF: %d" % (len(datos), len(con_us)))
    if not con_us:
        sys.exit("Sin uL/uR/uF: el ESP corre el PurePursuit.ino con FOX_US (2026-11-16 o después)?")

    for nombre, kt, ku, off in PARES:
        tof_ok = us_ok = ambos = solo_us = solo_tof = 0
        difs = []
        por_rango = {r: [] for r in RANGOS}
        for d in con_us:
            t, u = num(d.get(kt)), num(d.get(ku))
            tv = t is not None and t > 0
            uv = u is not None and u > 0
            tof_ok += tv
            us_ok += uv
            if tv and uv:
                ambos += 1
                df = t - (u + off)
                difs.append(df)
                for r in RANGOS:
                    if r[0] <= u + off < r[1]:
                        por_rango[r].append(df)
            elif uv:
                solo_us += 1
            elif tv:
                solo_tof += 1
        n = len(con_us)
        print("\n== %s == validas: ToF %s | US %s | ambos %s | solo US %s | solo ToF %s"
              % (nombre, pct(tof_ok, n), pct(us_ok, n), pct(ambos, n), pct(solo_us, n), pct(solo_tof, n)))
        if not difs:
            print("   sin lecturas simultáneas")
            continue
        malos = sum(abs(x) > a.umbral for x in difs)
        print("   ToF - US: mediana %+.0f mm | |dif| p90 %.0f mm | desacuerdo > %.0f mm: %s"
              % (statistics.median(difs), sorted(abs(x) for x in difs)[int(0.9 * (len(difs) - 1))],
                 a.umbral, pct(malos, len(difs))))
        for (lo, hi), xs in por_rango.items():
            if xs:
                print("   %4d-%-5s mm: n=%4d  mediana %+5.0f  desacuerdo %s"
                      % (lo, hi if hi < 99999 else "", len(xs), statistics.median(xs),
                         pct(sum(abs(x) > a.umbral for x in xs), len(xs))))


if __name__ == "__main__":
    main()
