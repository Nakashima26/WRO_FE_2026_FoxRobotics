"""Resumen de un tag del banco con latencia: ok = parada normal, sin choques, fuera del cajón <= 5 mm (como optimizar_park)."""
import sys, glob, statistics as S, os
sys.path.insert(0, "pure_pursuit/twin/tools/banco"); sys.path.insert(0, ".")
from optimizar_park import parse_out, ok_t
tag = sys.argv[1]; ok, fl = [], []
for f in sorted(glob.glob(os.path.join(os.environ.get("LAT_OUT", "runs/latencia.noindex"), tag, "n*", "s*.out"))):
    k = f.split("/")[-2] + "/" + os.path.basename(f)[:-4]
    rec = parse_out(f); good, t = ok_t(rec, f[:-4]) if rec else (False, None)
    (ok.append((k, t)) if good else fl.append(k))
ts = [t for _, t in ok]
print(f"{tag}: {len(ok)}/{len(ok)+len(fl)} ok" + (f"  media {S.mean(ts):.2f}  mediana {S.median(ts):.2f}  mejor {min(ts):.2f}" if ts else "") + "  fallas: " + " ".join(fl))
