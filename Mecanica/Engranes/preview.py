import math, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from gen_pinon import gear
fig, axs = plt.subplots(1, 3, figsize=(13, 4.6))
for ax, Z in zip(axs, (14, 19, 20)):
    lines, arcs, r = gear(Z=Z)
    for (x1, y1), (x2, y2) in lines: ax.plot([x1, x2], [y1, y2], "k-", lw=.8)
    for rr, a1, a2 in arcs:
        t = [a1 + (a2-a1)*k/20 for k in range(21)]; ax.plot([rr*math.cos(a) for a in t], [rr*math.sin(a) for a in t], "k-", lw=.8)
    ax.add_patch(plt.Circle((0, 0), r["rp"], fill=False, ls="--", lw=.6, color="b"))
    ax.set_title(f"{Z} dientes | Ø prim {2*r['rp']:.0f}  Ø ext {2*r['ra']:.0f} mm", fontsize=10)
    ax.set_aspect("equal"); ax.set_xlim(-12, 12); ax.set_ylim(-12, 12); ax.grid(alpha=.2)
plt.tight_layout(); plt.savefig("preview_pinones.png", dpi=100)
