"""Figures for the README, made from data that is in this repository.

    python docs/img/make_figures.py            # all figures
    python docs/img/make_figures.py route      # only one

Each function says where its data comes from. Output: docs/img/*.png
"""
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "docs/img"
sys.path.insert(0, str(REPO / "scripts"))

RED, GREEN, MAGENTA = "#d03b3b", "#2e9e4f", "#c2188b"
ORANGE, BLUE = "#f08a24", "#2f6fd6"
CAR, ROUTE, GREY = "#e8622c", "#9aa0a6", "#5f6368"
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.color": "#ecebe6", "savefig.dpi": 130})


def field(ax, inner=(1000, 2000)):
    """3 x 3 m field, inner walls 1 x 1 m centered (WRO FE mat)."""
    ax.add_patch(plt.Rectangle((0, 0), 3000, 3000, fill=False, lw=2.5, color="#222"))
    a, b = inner
    ax.add_patch(plt.Rectangle((a, a), b - a, b - a, fill=True, lw=2.5, ec="#222", fc="#f3f3f1"))
    ax.set_aspect("equal")
    ax.set_xlim(-60, 3060)
    ax.set_ylim(-60, 3060)
    ax.set_xticks(range(0, 3001, 500))
    ax.set_yticks(range(0, 3001, 500))
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")


def pillar(ax, x, y, color, size=50, **kw):
    ax.add_patch(plt.Rectangle((x - size / 2, y - size / 2), size, size,
                               color=RED if color.lower().startswith("r") else GREEN, zorder=5, **kw))


# ─────────────────────────────────────────────────────────────────────────────
def fig_route():
    """§3.10.2 — ESP32-only route following on the obstacle layout (3 laps, PWM 255).
    Data: logs_tof/ruta_obstaculos_20261023_152651.txt (written by scripts/ruta_bt.py)."""
    from ruta_bt import leer_log

    route, meta, P = leer_log(REPO / "logs_tof/ruta_obstaculos_20261023_152651.txt")
    t, x, y, lat, servo = P[:, 0] / 1000, P[:, 1], P[:, 2], P[:, 5], P[:, 6]

    fig = plt.figure(figsize=(12, 5.6))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.05, 1], hspace=0.45, wspace=0.2)
    ax = fig.add_subplot(gs[:, 0])
    field(ax)
    r = np.vstack([route, route[:1]])
    ax.plot(r[:, 0], r[:, 1], color=ROUTE, lw=9, alpha=0.35, solid_capstyle="round", label="route sent by the planner")
    ax.plot(x, y, color=CAR, lw=1.4, label="car (odometry), 3 laps")
    for sx, sy, c in meta.get("senales", []):
        pillar(ax, sx, sy, c)
    sx, sy, sth = (float(v) for v in meta["salida"].split(","))
    ax.annotate("", xy=(sx + 220 * math.cos(math.radians(sth)), sy + 220 * math.sin(math.radians(sth))),
                xytext=(sx, sy), arrowprops=dict(arrowstyle="-|>", color="#222", lw=1.8))
    ax.plot(sx, sy, "s", color="#222", ms=7)
    ax.text(sx + 60, sy + 60, "start", fontsize=9)
    ax.legend(loc="upper left", frameon=True, fontsize=9)
    ax.set_title("Path on the field", loc="left", fontsize=11)

    a1 = fig.add_subplot(gs[0, 1])
    a1.plot(t, lat, color=CAR, lw=1.3)
    a1.axhline(0, color="#222", lw=0.8)
    rms = float(np.sqrt(np.mean(lat ** 2)))
    a1.set_title(f"Lateral error to the route (rms {rms:.0f} mm)", loc="left", fontsize=11)
    a1.set_ylabel("mm (+ = left)")
    a1.set_ylim(-60, 60)

    a2 = fig.add_subplot(gs[1, 1], sharex=a1)
    a2.plot(t, servo, color=BLUE, lw=1.2)
    a2.axhline(75, color="#222", lw=0.8)
    a2.set_title("Steering servo command (75 = straight)", loc="left", fontsize=11)
    a2.set_ylabel("servo (°)")
    a2.set_xlabel("time (s)")

    fig.suptitle("ESP32 alone follows a planned route around 4 pillars — 3 laps at full speed (PWM 255), "
                 "no camera", x=0.06, ha="left", fontsize=12, fontweight="bold")
    fig.savefig(OUT / "route_following_obstacles.png", bbox_inches="tight")
    print("route_following_obstacles.png", f"rms={rms:.1f} mm", f"t={t[-1]:.1f} s")


# ─────────────────────────────────────────────────────────────────────────────
def _mat(ax, title):
    """Mat with origin at the center (as wro_field.py): outer walls ±1500, island ±500."""
    ax.add_patch(plt.Rectangle((-1500, -1500), 3000, 3000, fill=False, lw=2.5, color="#222"))
    ax.add_patch(plt.Rectangle((-500, -500), 1000, 1000, lw=2.5, ec="#222", fc="#f3f3f1"))
    ax.set_aspect("equal")
    ax.set_xlim(-1560, 1560)
    ax.set_ylim(-1560, 1560)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_title(title, loc="left", fontsize=11, fontweight="bold")


def _parking(ax, section):
    from pure_pursuit.wro_field import _barrier_corners
    for poly in _barrier_corners(section):
        ax.add_patch(plt.Polygon(poly, color=MAGENTA, zorder=4))


def _seats(ax, wf, label=False, skip=()):
    for sec in wf.SECTIONS_CW:
        for sid, (h, w) in wf.SEATS.items():
            if (sec, sid) in skip:
                continue
            x, y = wf.section_to_world(sec, h, w)
            ax.add_patch(plt.Circle((x, y), 38, fill=False, ls=(0, (2, 2)), lw=1, color=GREY, zorder=2))
            if label:
                ax.text(x, y - 95, sid, ha="center", va="top", fontsize=7.5, color=GREY)


def fig_strategy():
    """§3.7 — the field as a table of seats, and the route the real planner (ring_route.plan) draws
    for one field: lap 1 with unknown seats, and from lap 2 with the full map."""
    sys.path.insert(0, str(REPO / "src/RASPI/cam"))
    from pure_pursuit import ring_route as R
    from pure_pursuit import wro_field as wf
    R._NATIVE_DP = False   # numpy DP: same result, no compiled library needed

    # One legal field (cards 26 / 4 / 21 / 9 style), parking lot in the south straight.
    seats = {("N", "T1"): "Green", ("N", "T2"): "Red", ("E", "X1"): "Red",
             ("S", "T1"): "Red", ("S", "T2"): "Green", ("W", "X2"): "Green"}
    parking, direction = "S", "CW"
    seen = {("S", "T1"), ("S", "T2"), ("W", "X2")}          # seen so far in lap 1 (start straight + next)
    unknown = [(sec, sid) for sec in wf.SECTIONS_CW for sid in wf.SEATS
               if (sec, sid) not in seats and not (sec == parking and sid in wf.PARKING_FORBIDDEN_SEATS)
               and sec not in ("S", "W")]
    known_lap1 = {k: v for k, v in seats.items() if k in seen}

    r1 = R.plan(direction, known_lap1, parking=parking, unknown=unknown)
    r2 = R.plan(direction, seats, parking=parking)

    fig, axs = plt.subplots(1, 3, figsize=(16, 5.9))
    # A — the seat table
    ax = axs[0]
    _mat(ax, "1 · Every straight has 6 possible pillar seats")
    _seats(ax, wf)
    for sid, (h, w) in wf.SEATS.items():                 # label the north straight only
        x, y = wf.section_to_world("N", h, w)
        dy = 70 if h == 400 else -70
        ax.text(x, y + dy, sid, ha="center", va="bottom" if dy > 0 else "top", fontsize=8.5, color="#222")
    ax.text(640, 1100, "outer row\n400 mm", ha="left", va="center", fontsize=8, color=GREY)
    ax.text(640, 900, "inner row\n600 mm", ha="left", va="center", fontsize=8, color=GREY)
    _parking(ax, parking)
    ax.text(0, 70, "island", ha="center", va="center", fontsize=9, color=GREY)
    ax.text(0, -110, "T = pair seats\nX = lone pillar", ha="center", va="center", fontsize=8, color=GREY)
    ax.text(-140, -1290, "parking lot", ha="right", va="center", fontsize=8.5, color=MAGENTA)

    # B — lap 1
    ax = axs[1]
    _mat(ax, "2 · Lap 1: unknown seats are obstacles with no side")
    _seats(ax, wf)
    for sec, sid in unknown:
        x, y = wf.section_to_world(sec, *wf.SEATS[sid])
        ax.add_patch(plt.Circle((x, y), 60, color="#bdbdbd", alpha=0.55, zorder=3))
    for (sec, sid), c in known_lap1.items():
        pillar(ax, *wf.section_to_world(sec, *wf.SEATS[sid]), c)
    _parking(ax, parking)
    ax.plot(r1.x, r1.y, color=BLUE, lw=2.2, zorder=6, label="planned route (lap 1)")
    ax.add_patch(plt.Circle((0, 0), 1, color="#bdbdbd", alpha=0.55, label="unknown seat"))
    ax.legend(loc="center", frameon=False, fontsize=7.5, handlelength=1.4)

    # C — full map
    ax = axs[2]
    _mat(ax, "3 · Lap 2+: full map, one fixed fast line")
    _seats(ax, wf)
    for (sec, sid), c in seats.items():
        pillar(ax, *wf.section_to_world(sec, *wf.SEATS[sid]), c)
    _parking(ax, parking)
    ax.plot(r2.x, r2.y, color=CAR, lw=2.2, zorder=6, label="planned route (lap 2–3)")
    i = len(r2.x) // 8
    ax.annotate("", xy=(r2.x[i + 4], r2.y[i + 4]), xytext=(r2.x[i], r2.y[i]),
                arrowprops=dict(arrowstyle="-|>", color=CAR, lw=2, mutation_scale=18), zorder=7)
    ax.text(0, 120, "red: pass on the right\ngreen: pass on the left", ha="center", va="center", fontsize=8.5)
    ax.legend(loc="center", bbox_to_anchor=(0.5, 0.42), frameon=False, fontsize=7.5, handlelength=1.4)

    fig.suptitle("Route mode: fill the seat table with the camera, plan the whole lap with dynamic programming",
                 x=0.03, ha="left", fontsize=12.5, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT / "strategy_seats_route.png", bbox_inches="tight")
    print("strategy_seats_route.png", f"lap1 {r1.length_mm:.0f} mm, full {r2.length_mm:.0f} mm",
          r1.report.get("feasible"), r2.report.get("feasible"))


# ─────────────────────────────────────────────────────────────────────────────
def _read_stl(path):
    """Binary or ASCII STL -> (N, 3, 3) triangles."""
    data = Path(path).read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = [list(map(float, ln.split()[1:4])) for ln in data.decode(errors="ignore").splitlines()
             if ln.strip().startswith("vertex")]
        return np.array(v).reshape(-1, 3, 3)
    n = int.from_bytes(data[80:84], "little")
    rec = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].astype(float)


def _draw_part(ax, tri, color, elev=28, azim=-55):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    tri = tri - tri.reshape(-1, 3).mean(axis=0)
    tri = tri[..., [0, 2, 1]] * np.array([1, -1, 1])   # SolidWorks exports Y-up; matplotlib is Z-up
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    e, a = math.radians(elev), math.radians(azim)
    light = np.array([math.cos(e) * math.cos(a + 0.6), math.cos(e) * math.sin(a + 0.6), math.sin(e) + 0.3])
    light /= np.linalg.norm(light)
    shade = 0.35 + 0.65 * np.abs(n @ light)
    rgb = np.array(matplotlib.colors.to_rgb(color))
    fc = np.clip(shade[:, None] * rgb[None, :], 0, 1)
    ax.add_collection3d(Poly3DCollection(tri, facecolors=fc, edgecolors="none", linewidths=0))
    r = np.abs(tri.reshape(-1, 3)).max()
    ax.set_xlim(-r, r)
    ax.set_ylim(-r, r)
    ax.set_zlim(-r, r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ext = tri.reshape(-1, 3).max(axis=0) - tri.reshape(-1, 3).min(axis=0)
    return ext


def fig_parts():
    """§1.5 — the printed parts of the v2 car, rendered from models/v2/stl/."""
    groups = [
        ("Chassis", "#d83a3a", [("BaseChasisMain_V2", "base chassis"), ("TapaChasis_V2", "top shell"),
                                ("BatterySupport", "battery tunnel support")]),
        ("Drivetrain", "#e8862c", [("DCsupport_V2", "motor holder (motor ∥ axle)"),
                                   ("Differential_V2", "open differential, 25 T")]),
        ("Steering", "#2f6fd6", [("NewCremallera_V2", "rack"), ("SteeringKnuckle", "knuckle"),
                                 ("NewLinkajeL_V2", "tie rod")]),
        ("Sensors", "#3d3d3a", [("CameraBase", "camera base"), ("CameraFrnt", "camera front"),
                                ("ToFSupport", "side ToF mount")]),
    ]
    fig = plt.figure(figsize=(13, 11))
    for gi, (gname, color, parts) in enumerate(groups):
        for pi, (stem, label) in enumerate(parts):
            ax = fig.add_subplot(4, 3, gi * 3 + pi + 1, projection="3d")
            tri = _read_stl(REPO / f"models/v2/stl/{stem}.stl")
            ext = _draw_part(ax, tri, color)
            ax.set_title(f"{label}\n{stem}.stl · {ext[0]:.0f}×{ext[1]:.0f}×{ext[2]:.0f} mm", fontsize=9.5, pad=-14)
            if pi == 0:
                ax.text2D(-0.08, 0.5, gname, transform=ax.transAxes, rotation=90, va="center",
                          fontsize=13, fontweight="bold", color=color)
    fig.suptitle("v2 printed parts (PLA, 0.2 mm layers; gears and rack 100 % infill) — all from models/v2/stl/",
                 x=0.03, ha="left", fontsize=13, fontweight="bold")
    fig.subplots_adjust(left=0.04, right=0.99, top=0.91, bottom=0.0, wspace=-0.15, hspace=-0.02)
    fig.savefig(OUT / "v2_printed_parts.png", dpi=110)
    print("v2_printed_parts.png")


# ─────────────────────────────────────────────────────────────────────────────
def fig_steering():
    """§1.3 — Ackermann geometry at full lock (CAD: 53.1° / 37.0°, kingpin track 60 mm, wheelbase 112.85 mm)
    and the turning radius measured on the mat per servo command (2026-10-22 table in §1.3)."""
    L, T = 112.85, 60.0
    di, do = math.radians(53.1), math.radians(37.0)
    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 5.4), gridspec_kw={"width_ratios": [1, 1.15]})

    # A — top view, turning left; rear axle on y = 0, kingpins at y = L
    a.set_aspect("equal")
    a.grid(False)
    kl, kr = (-T / 2, L), (T / 2, L)
    a.plot([-55, 55], [0, 0], color="#222", lw=3)                       # rear axle
    a.plot([0, 0], [0, L], color="#bbb", lw=1, ls="--")
    a.plot([kl[0], kr[0]], [L, L], color="#222", lw=3)                  # front kingpin line
    for (x, y), d, name in ((kl, di, "inner"), (kr, do, "outer")):
        ux, uy = -math.sin(d), math.cos(d)                               # wheel heading (left turn)
        a.plot([x - 14 * ux, x + 14 * ux], [y - 14 * uy, y + 14 * uy], color=BLUE, lw=7, solid_capstyle="butt")
        a.plot([x, x - 400 * uy], [y, y + 400 * ux], color=BLUE, lw=0.8, ls=":")   # perpendicular
    for x in (-55, 55):
        a.plot([x, x], [-14, 14], color="#222", lw=7, solid_capstyle="butt")
    # ideal center: on the rear axle line
    cx_i = -T / 2 - L / math.tan(di)
    cx_o = T / 2 - L / math.tan(do)
    a.plot([cx_o], [0], "o", color=CAR, ms=8)
    a.plot([cx_i], [0], "x", color=CAR, ms=9, mew=2)
    a.annotate(f"inner {math.degrees(di):.1f}°", kl, xytext=(-120, 150), fontsize=10, color=BLUE,
               arrowprops=dict(arrowstyle="-", color=BLUE, lw=0.8))
    a.annotate(f"outer {math.degrees(do):.1f}°", kr, xytext=(20, 150), fontsize=10, color=BLUE,
               arrowprops=dict(arrowstyle="-", color=BLUE, lw=0.8))
    a.text(cx_o, -22, "turn center from\nthe outer wheel", ha="center", va="top", fontsize=8.5, color=CAR)
    a.text(cx_i, 18, "from the inner", ha="center", va="bottom", fontsize=8.5, color=CAR)
    a.annotate("", xy=(75, 0), xytext=(75, L), arrowprops=dict(arrowstyle="<->", color=GREY))
    a.text(80, L / 2, f"wheelbase\n{L:.0f} mm", fontsize=8.5, color=GREY, va="center")
    a.set_xlim(-220, 140)
    a.set_ylim(-70, 200)
    a.set_xticks([])
    a.set_yticks([])
    for s in a.spines.values():
        s.set_visible(False)
    a.set_title("Full left lock (CAD): both wheel normals meet near\nthe rear-axle line → ~106 % Ackermann",
                loc="left", fontsize=11)

    # B — measured radius
    srv = np.array([1, 4, 8, 16, 140, 144, 148, 152, 156])
    rad = np.array([123, 130, 142, 166, 153, 140, 136, 134, 136])
    right, left = srv < 75, srv > 75
    b.plot(srv[right], rad[right], "o-", color=CAR, lw=2, label="right turns")
    b.plot(srv[left], rad[left], "s-", color=BLUE, lw=2, label="left turns")
    b.axvline(75, color="#222", lw=0.8)
    b.text(76.5, 175, "75 = straight", fontsize=9)
    for x, y in ((1, 123), (152, 134)):
        b.annotate(f"{y} mm", (x, y), xytext=(0, -16), textcoords="offset points", ha="center", fontsize=9,
                   fontweight="bold")
    b.set_xlabel("servo command (1 = right stop, 156 = left stop)")
    b.set_ylabel("turning radius at rear-axle center (mm)")
    b.set_ylim(110, 185)
    b.legend(frameon=False)
    b.set_title("Measured on the mat (PWM 150, 3 reps each, 2026-10-22):\nleft is ~15 % weaker → per-side servo table",
                loc="left", fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT / "steering_ackermann_radius.png", bbox_inches="tight")
    print("steering_ackermann_radius.png", f"centers {cx_i:.0f} / {cx_o:.0f} mm")


# ─────────────────────────────────────────────────────────────────────────────
def fig_sensors():
    """§2.4 — what each sensor covers, to scale. Mounts from config.SENSOR_MOUNTS (CAD v2), camera pose and
    HFOV from config.py, ToF cone ~13.5° with ROI 8×8, ranges = what the localizer accepts (LOC_TOF_MAX_MM)."""
    sys.path.insert(0, str(REPO / "src/RASPI/cam"))
    from pure_pursuit import config as C

    fig, ax = plt.subplots(figsize=(8.6, 9.2))
    ax.set_aspect("equal")
    # camera floor footprint: 200..1200 mm ahead of the rear axle, inside the horizontal FOV
    hf = math.radians(C.CAMERA_HFOV_DEG / 2)
    near, far, cy = 200.0, 1200.0, C.CAMERA_FWD_MM
    pts = [(-(near - cy) * math.tan(hf), near), (-(far - cy) * math.tan(hf), far),
           ((far - cy) * math.tan(hf), far), ((near - cy) * math.tan(hf), near)]
    ax.add_patch(plt.Polygon(pts, color="#f4c542", alpha=0.35, lw=0, label="camera: floor seen 0.2–1.2 m"))
    ax.plot([0, pts[1][0]], [cy, far], color="#c99a00", lw=0.8, ls="--")
    ax.plot([0, pts[2][0]], [cy, far], color="#c99a00", lw=0.8, ls="--")
    ax.text(0, 1080, f"camera IMX219, {C.CAMERA_HFOV_DEG:.0f}° wide,\n{C.CAMERA_TILT_DEG:.1f}° down, "
                     f"{C.CAMERA_HEIGHT_MM:.0f} mm high", ha="center", fontsize=9)
    # ToF cones
    half = math.radians(13.5 / 2)
    names = {"tof_left": ("ToF left", C.LOC_TOF_MAX_MM["tL"]), "tof_right": ("ToF right", C.LOC_TOF_MAX_MM["tR"]),
             "tof_front": ("ToF front", 800.0), "tof_rear": ("ToF rear", C.LOC_TOF_MAX_MM["tB"])}
    for key, (label, rng) in names.items():
        right, fwd, deg = C.SENSOR_MOUNTS[key]
        a = math.radians(deg)
        p1 = (right + rng * math.sin(a - half), fwd + rng * math.cos(a - half))
        p2 = (right + rng * math.sin(a + half), fwd + rng * math.cos(a + half))
        ax.add_patch(plt.Polygon([(right, fwd), p1, p2], color=BLUE, alpha=0.22, lw=0))
        ax.plot([right, right + rng * math.sin(a)], [fwd, fwd + rng * math.cos(a)], color=BLUE, lw=1)
        ax.plot(right, fwd, "o", color=BLUE, ms=5)
        tx, ty = right + (rng + 30) * math.sin(a), fwd + (rng + 30) * math.cos(a)
        extra = " (reliable ≤ 80 cm)" if key == "tof_front" else f" (used ≤ {rng / 10:.0f} cm)"
        ax.text(tx, ty, label + extra, fontsize=8.5, color=BLUE,
                ha="center" if deg in (0, 180) else ("left" if deg == 90 else "right"),
                va="bottom" if deg == 0 else ("top" if deg == 180 else "center"))
    # car outline (rear axle at origin, +y forward)
    L0, L1, w = -C.REAR_OVERHANG_MM, C.ROBOT_LENGTH_MM - C.REAR_OVERHANG_MM, C.ROBOT_WIDTH_MM / 2
    ax.add_patch(plt.Rectangle((-w, L0), 2 * w, L1 - L0, fc="#e8e8e6", ec="#222", lw=1.5, zorder=3))
    ax.plot(0, cy, "s", color="#c99a00", ms=6, zorder=4)
    ax.plot(0, 60, "*", color=RED, ms=12, zorder=4)
    ax.text(14, 60, "BNO085", fontsize=8, va="center", zorder=5)
    ax.annotate("", xy=(0, L1 + 40), xytext=(0, L1 - 5), arrowprops=dict(arrowstyle="-|>", color="#222"), zorder=5)
    ax.text(-w - 10, L0 + 5, f"car {C.ROBOT_LENGTH_MM:.0f} × {C.ROBOT_WIDTH_MM:.0f} mm", ha="right", fontsize=8.5)
    ax.set_xlim(-620, 620)
    ax.set_ylim(-340, 1260)
    ax.set_xlabel("mm (right +)")
    ax.set_ylabel("mm ahead of the rear axle")
    ax.legend(loc="lower right", frameon=False, fontsize=9)
    ax.set_title("Sensor coverage, to scale (top view)", loc="left", fontsize=12, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT / "sensor_coverage.png", bbox_inches="tight")
    print("sensor_coverage.png")


def fig_tof():
    """§2.4 — static error of the side ToF on the black wall, ROI 16×16 vs 8×8 (2026-10-21, table in §2.4)."""
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    d = np.array([60, 80])
    w = 6
    ax.bar(d - w / 2, [-50, -190], w, color="#bbb", label="ROI 16 × 16 (default): sees the white floor")
    ax.bar(d + w / 2, [-8, -40], w, color=BLUE, label="ROI 8 × 8 (v2)")
    for x, v in ((60 - w / 2, -50), (80 - w / 2, -190), (60 + w / 2, -8), (80 + w / 2, -40)):
        ax.text(x, v - 6, f"{v}", ha="center", va="top", fontsize=9)
    ax.axhline(0, color="#222", lw=0.8)
    ax.set_xticks(d, ["60 cm", "80 cm"])
    ax.set_ylabel("error (mm)")
    ax.set_ylim(-225, 20)
    ax.legend(frameon=False, fontsize=8.5, loc="lower left")
    ax.set_title("VL53L1X on the black wall: smaller ROI, ~5× less error", loc="left", fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT / "tof_roi_error.png", bbox_inches="tight")
    print("tof_roi_error.png")


def fig_wiring():
    """§2.3 — the whole wiring on one page: power rails and every signal with its GPIO
    (from the RevB schematics, the ESP32 pin map and the cables table)."""
    from matplotlib.patches import FancyBboxPatch
    C = {"bat": "#d62728", "5v": "#ff7f0e", "pwm": "#7b3fbf", "i2c": "#1f77b4",
         "uart": "#2ca02c", "enc": "#8c564b", "sig": "#555555"}
    fig, ax = plt.subplots(figsize=(16, 9.6))
    ax.set_xlim(0, 18); ax.set_ylim(0.3, 11.4); ax.axis("off"); ax.grid(False)

    def box(x, y, w, h, title, sub="", fc="#f4f3ee", ec="#333333"):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12",
                                    fc=fc, ec=ec, lw=1.4))
        ax.text(x + w / 2, y + h / 2 + (0.14 if sub else 0), title, ha="center", va="center",
                fontsize=10.5, weight="bold")
        if sub:
            ax.text(x + w / 2, y + h / 2 - 0.2, sub, ha="center", va="center", fontsize=8.3, color="#444")

    def wire(pts, color, label="", at=0, dx=0.0, dy=0.13, lw=2.6, ha="center"):
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color=color, lw=lw, solid_capstyle="round", zorder=1)
        if label:
            (x0, y0), (x1, y1) = pts[at], pts[at + 1]
            ax.text((x0 + x1) / 2 + dx, (y0 + y1) / 2 + dy, label, color=color, fontsize=8.6,
                    ha=ha, va="bottom", weight="bold")

    # boxes
    box(0.3, 8.8, 2.4, 1.2, "3S LiPo", "11.1 V · 2200 mAh · T-plug", fc="#fde2e1")
    box(3.6, 8.8, 2.2, 1.2, "SW2 switch", "+ C1 470 µF (logic)")
    box(6.6, 9.1, 2.6, 1.0, "MINI560 #2", "+5V_PI only (power board)", fc="#ffe9d4")
    box(6.6, 7.4, 2.6, 1.2, "MINI560 #1", "+5 V: ESP32, servo, encoder,\nTB6612 logic, shifter, 3.3 V reg.", fc="#ffe9d4")
    box(6.6, 5.8, 2.6, 1.0, "3.3 V regulator (J6)", "ToF ×4 · BNO085 · pull-ups", fc="#fff6c9")
    box(3.4, 3.4, 2.4, 1.4, "TB6612FNG", "ROB-14450 · VM = battery")
    box(0.3, 3.4, 2.4, 1.4, "N30 30:1 motor", "7 CPR Hall encoder")
    box(10.1, 8.2, 3.0, 2.0, "Raspberry Pi 5", "vision · map · route planner", fc="#e3f0e3")
    box(10.1, 3.0, 3.0, 3.8, "ESP32 DevKitC", "odometry · pure pursuit\nspeed loop · state machine", fc="#e1ecf7")
    box(10.1, 1.0, 3.0, 1.0, "Level shifter", "BOB-12009 · 5 V ↔ 3.3 V")
    box(14.7, 9.5, 3.0, 0.8, "Camera IMX219 NoIR", "")
    box(14.7, 8.7, 3.0, 0.6, "Start button SW1", "")
    box(14.7, 7.9, 3.0, 0.6, "READY LED D2", "")
    box(14.7, 6.0, 3.0, 0.8, "Servo SG90", "5 V · Dupont")
    box(14.7, 4.6, 3.0, 1.0, "VL53L1X ToF ×4", "3.3 V · JST-ZH 5-pin, 26 AWG")
    box(14.7, 3.3, 3.0, 0.8, "BNO085 IMU", "3.3 V · Dupont")
    box(14.7, 1.0, 3.0, 1.0, "HC-SR04 ×3", "5 V · JST-ZH 4-pin (cross-check)")

    # power
    wire([(2.7, 9.4), (3.6, 9.4)], C["bat"], "16 AWG", dy=0.1)
    wire([(5.8, 9.6), (6.6, 9.6)], C["bat"], "11.1 V", dy=0.06)
    wire([(5.8, 9.05), (6.2, 9.05), (6.2, 8.0), (6.6, 8.0)], C["bat"])
    wire([(4.6, 8.8), (4.6, 4.8)], C["bat"], "11.1 V → VM", dx=0.08, dy=0, ha="left")
    wire([(9.2, 9.6), (9.65, 9.6), (9.65, 9.2), (10.1, 9.2)], C["5v"])
    ax.text(9.65, 8.68, "5 V → Pi\nGPIO pins", color=C["5v"], fontsize=8.6, weight="bold", ha="center")
    wire([(9.2, 8.0), (10.6, 8.0), (10.6, 6.8)], C["5v"], "5 V", dy=0.04)
    wire([(7.9, 7.4), (7.9, 6.8)], C["5v"], "5 V", dx=0.08, dy=-0.1, ha="left")

    # Pi side
    wire([(11.6, 8.2), (11.6, 6.8)], C["uart"], "UART 460 800 baud\nESP 17/16 ↔ Pi 14/15", dx=0.1, dy=-0.4, ha="left")
    wire([(13.1, 9.9), (14.7, 9.9)], C["sig"], "CSI 22→15 pin")
    wire([(13.1, 9.0), (14.7, 9.0)], C["sig"], "GPIO 17")
    wire([(13.1, 8.5), (13.9, 8.5), (13.9, 8.2), (14.7, 8.2)], C["sig"], "GPIO 27", at=0, dy=0.05)

    # ESP32 side
    wire([(13.1, 6.4), (14.7, 6.4)], C["pwm"], "GPIO 13 · 50 Hz")
    wire([(13.1, 5.1), (14.7, 5.1)], C["i2c"], "I²C 21/22")
    ax.text(13.9, 4.9, "XSHUT 0/15/4/5", color=C["i2c"], fontsize=8.6, weight="bold", ha="center", va="top")
    wire([(14.7, 3.7), (13.1, 3.7)], C["uart"], "RVC → GPIO 25")
    wire([(10.1, 4.1), (5.8, 4.1)], C["pwm"], "PWM 23 · DIR 18/19")
    wire([(3.4, 4.1), (2.7, 4.1)], C["bat"], "A1/A2", dy=0.1)
    wire([(1.5, 3.4), (1.5, 2.6), (10.6, 2.6), (10.6, 3.0)], C["enc"],
         "Encoder A/B → GPIO 34/36 (PCNT ×4, 4k7 pull-ups) · JST-ZH 6-pin", at=1, dy=0.05)
    wire([(11.6, 3.0), (11.6, 2.0)], C["sig"], "ECHO 32/35/33", dx=0.1, dy=-0.1, ha="left")
    wire([(13.1, 1.5), (14.7, 1.5)], C["sig"], "ECHO (5 V)", dy=0.05)
    wire([(13.1, 3.1), (15.4, 3.1), (15.4, 2.0)], C["sig"], "TRIG 27/26/14 (direct)", at=1, dx=0.1, dy=-0.1, ha="left")

    # legend
    items = [("11.1 V battery", C["bat"]), ("5 V", C["5v"]), ("PWM / motor control", C["pwm"]),
             ("I²C", C["i2c"]), ("UART", C["uart"]), ("Encoder", C["enc"]), ("Other signals", C["sig"])]
    for i, (name, col) in enumerate(items):
        x = [0.35, 2.55, 3.85, 6.55, 7.65, 9.05, 10.65][i]
        ax.plot([x, x + 0.45], [0.55, 0.55], color=col, lw=3)
        ax.text(x + 0.55, 0.55, name, va="center", fontsize=8.8)
    ax.text(0.3, 11.05, "Wiring of the v2 car", fontsize=14, weight="bold")
    ax.text(0.3, 10.65, "Ground not drawn (common to all). Power board RevB_2: regulators, TB6612, sensor connectors; "
            "logic board RevB_1: ESP32, switch, level shifter. Schematics: electrical/export/.",
            fontsize=8.8, color="#444")
    fig.savefig(OUT / "wiring_diagram.png", bbox_inches="tight", dpi=120)
    print("wiring_diagram.png")


FIGS = {"route": fig_route, "strategy": fig_strategy, "parts": fig_parts, "steering": fig_steering,
        "sensors": fig_sensors, "tof": fig_tof, "wiring": fig_wiring}

if __name__ == "__main__":
    for name in sys.argv[1:] or FIGS:
        FIGS[name]()
