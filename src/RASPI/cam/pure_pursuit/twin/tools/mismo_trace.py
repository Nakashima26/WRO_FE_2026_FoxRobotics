"""Compara dos corridas de drive.py (antes/después de un cambio de velocidad).

uso: mismo_trace.py <dir_a> <dir_b>   (cada uno con s<seed>/trace.csv y s<seed>.out)
Exige trace.csv y fw_debug.log idénticos salvo pi_frame_ms (tiempo de pared).
Imprime mean_pi_frame_ms de ambos por seed."""
import csv, re, sys
from pathlib import Path

a, b = Path(sys.argv[1]), Path(sys.argv[2])
ok = True
for ta in sorted(a.glob("s*/trace.csv")):
    seed = ta.parent.name
    tb = b / seed / "trace.csv"
    ra, rb = list(csv.DictReader(open(ta))), list(csv.DictReader(open(tb)))
    for r in ra + rb:
        r.pop("pi_frame_ms", None)
    same = ra == rb and (ta.parent / "fw_debug.log").read_bytes() == (tb.parent / "fw_debug.log").read_bytes()
    ms = [re.search(r'"mean_pi_frame_ms": ([0-9.]+)', (d / f"{seed}.out").read_text()).group(1) for d in (a, b)]
    print(f"{seed}: {'IGUAL' if same else 'DISTINTO'}  ms/frame {ms[0]} -> {ms[1]}")
    ok &= same
sys.exit(0 if ok else 1)
