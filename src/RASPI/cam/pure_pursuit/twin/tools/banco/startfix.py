"""Error del arranque ciego (w0 estimado vs parkw de field.txt) y si la corrida falló.

Uso: python startfix.py RUNS/<nombre>
"""
import re, sys, glob, os
d = sys.argv[1]
rows = []
for n in range(1, 6):
    for f in glob.glob(f"{d}/cand_n{n}/s*/field.txt"):
        sd = os.path.dirname(f); seed = os.path.basename(sd)[1:]
        pw = float(re.search(r"parkw=(\d+)", open(f).read()).group(1))
        m = re.search(r"arranque: cajón en w0=(-?\d+) \(δ=[^,]*, (masa [0-9.]+|FORZADO)\) tras (\d+)", open(sd + "/pi.log", errors="ignore").read())
        st = [l for l in open(sd + ".out", errors="ignore") if l.startswith("STOP ")]
        ok = bool(st) and st[-1].startswith("STOP race_finished") and '"collisions": []' in st[-1] and '"sign_contacts": []' in st[-1]
        if m:
            rows.append((abs(float(m.group(1)) - pw), seed, n, m.group(2), int(m.group(3)), ok))
rows.sort(reverse=True)
print("n con arranque:", len(rows))
for r in rows[:15]:
    print(f"err={r[0]:5.0f} s{r[1]} n{r[2]} {r[3]:9s} lect={r[4]:3d} {'ok' if r[5] else 'FALLA'}")
import collections
for th in (50, 100, 150, 250):
    big = [r for r in rows if r[0] > th]
    print(f">{th} mm: {len(big)} corridas, fallan {sum(not r[5] for r in big)}")
print("fallas con err<=100:", sum(1 for r in rows if r[0] <= 100 and not r[5]), "de", sum(1 for r in rows if not r[5]))
