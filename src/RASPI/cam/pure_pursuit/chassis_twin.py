"""
Twin SIL del Fox: firmware real + PPRuntime + física.

  cd src/RASPI/cam
  ../../../.venv/bin/python -m pure_pursuit.chassis_twin --seed 3 --no-show
"""

from __future__ import annotations

import argparse
import ast
import math
import multiprocessing as mp
import time
from queue import Empty
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

from pure_pursuit import config as C
from pure_pursuit.centerline import draw_bev_debug
from pure_pursuit.twin.params import TwinParams
from pure_pursuit.twin.sim import RunResult, Sim, SimFrameView
from pure_pursuit.twin.world import body_corners, robot_to_world
from pure_pursuit.wro_field import (
    INNER_HALF_MM,
    LINE_MM,
    OUTER_HALF_MM,
    SEATS,
    SECTIONS_CW,
    SIGN_MM,
    blue_segments,
    orange_segments,
    randomize,
    section_to_world,
)

def _bev_window(q):
    import cv2
    cv2.namedWindow("BEV", cv2.WINDOW_NORMAL)
    while True:
        img = q.get()
        if img is None:
            break
        cv2.imshow("BEV", img)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cv2.destroyAllWindows()


def _send_bev(q, img) -> None:
    if q is None or img is None:
        return
    frame = img.copy()
    try:
        q.put_nowait(frame)
    except Exception:
        try:
            q.get_nowait()
        except Exception:
            pass
        try:
            q.put_nowait(frame)
        except Exception:
            pass


_VIEW = 700


def _field_payload(field) -> dict:
    return {
        "outer": [tuple(p) for p in field.outer],
        "inner": [tuple(p) for p in field.inner],
        "orange": [((a[0], a[1]), (b[0], b[1])) for a, b in orange_segments()],
        "blue": [((a[0], a[1]), (b[0], b[1])) for a, b in blue_segments()],
        "seats": [section_to_world(sec, h, w) for sec in SECTIONS_CW for h, w in SEATS.values()],
        "signs": [(s.x, s.y, s.color) for s in field.signs],
        "barriers": [[(x, y) for x, y in box] for box in field.barriers],
    }


def _to_px(x: float, y: float, size: int = _VIEW) -> tuple[int, int]:
    m = 24
    s = (size - 2 * m) / 3000.0
    return int(m + (x + 1500.0) * s), int(m + (1500.0 - y) * s)


def _offer(q, msg) -> None:
    if q is None:
        return
    try:
        q.put_nowait(msg)
        return
    except Exception:
        pass
    try:
        old = q.get_nowait()
    except Exception:
        old = None
    # No se tira el mapa: si se pierde, la ventana se queda en blanco.
    if old is not None and old[0] == "field":
        try:
            q.put_nowait(old)
        except Exception:
            pass
        return
    try:
        q.put_nowait(msg)
    except Exception:
        pass


def _fit_panel(img, size: int, label: str):
    """Mete img en un cuadrado sin aplastar, con el nombre arriba."""
    import cv2
    panel = np.full((size, size, 3), 255, np.uint8)
    if img is None:
        cv2.putText(panel, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (40, 40, 40), 2, cv2.LINE_AA)
        return panel
    bh, bw = img.shape[:2]
    # Deja una franja para el nombre.
    avail = size - 36
    scale = min(avail / bh, size / bw)
    resized = cv2.resize(img, (max(1, int(bw * scale)), max(1, int(bh * scale))))
    y0 = 36 + (avail - resized.shape[0]) // 2
    x0 = (size - resized.shape[1]) // 2
    panel[y0:y0 + resized.shape[0], x0:x0 + resized.shape[1]] = resized
    cv2.putText(panel, label, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (20, 20, 20), 2, cv2.LINE_AA)
    return panel


def _view_window(frame_q, key_q) -> None:
    """Una sola ventana: mapa + cámara (antes del BEV) + BEV."""
    import cv2
    field = None
    trail: list[tuple[float, float]] = []
    pose = None
    bev = None
    cam = None
    dig = None
    cv2.namedWindow("Fox", cv2.WINDOW_NORMAL)
    while True:
        try:
            msg = frame_q.get(timeout=0.03)
        except Empty:
            msg = None
        if msg is None:
            pass
        elif msg[0] == "stop":
            break
        elif msg[0] == "field":
            field = msg[1]
            trail = []
            pose = None
            bev = None
            cam = None
            dig = None
        elif msg[0] == "frame":
            pose = msg[1]
            trail.append(msg[1]["xy"])
            if msg[1]["bev"] is not None:
                bev = msg[1]["bev"]
            if msg[1].get("cam") is not None:
                cam = msg[1]["cam"]
            if msg[1].get("dig") is not None:
                dig = msg[1]["dig"]
        if field is None:
            k = cv2.waitKey(1) & 0xFF
            name = _key_name(k)
            if name:
                key_q.put(name)
            continue
        img = _paint(field, trail, pose, bev, cam, dig)
        cv2.imshow("Fox", img)
        k = cv2.waitKey(1) & 0xFF
        name = _key_name(k)
        if name:
            key_q.put(name)
    cv2.destroyAllWindows()


def _key_name(k: int) -> str | None:
    return {
        ord("r"): "r",
        ord("n"): "n",
        ord(" "): " ",
        27: "q",
        ord("+"): "+",
        ord("="): "+",
        ord("-"): "-",
        ord("_"): "-",
    }.get(k)


def _paint(field, trail, pose, bev, cam=None, dig=None):
    import cv2
    world = np.full((_VIEW, _VIEW, 3), (180, 196, 215), np.uint8)
    def poly(pts, color, fill=False, thick=1):
        p = np.array([_to_px(x, y) for x, y in pts], np.int32)
        if fill:
            cv2.fillPoly(world, [p], color)
        cv2.polylines(world, [p], True, (0, 0, 0) if fill else color, thick, cv2.LINE_AA)
    poly(field["outer"], (180, 196, 215), fill=True, thick=2)
    poly(field["inner"], (0, 0, 0), fill=True)
    for a, b in field["orange"]:
        cv2.line(world, _to_px(*a), _to_px(*b), (0, 140, 255), 1, cv2.LINE_AA)
    for a, b in field["blue"]:
        cv2.line(world, _to_px(*a), _to_px(*b), (255, 120, 0), 1, cv2.LINE_AA)
    for x, y in field["seats"]:
        cv2.circle(world, _to_px(x, y), 3, (90, 90, 90), -1, cv2.LINE_AA)
    half = SIGN_MM / 2.0
    for x, y, color in field["signs"]:
        box = [(x - half, y - half), (x + half, y - half), (x + half, y + half), (x - half, y + half)]
        poly(box, (0, 0, 255) if color == "Red" else (0, 200, 0), fill=True)
    for box in field["barriers"]:
        poly(box, (255, 0, 255), fill=True)
    if len(trail) > 1:
        pts = np.array([_to_px(x, y) for x, y in trail], np.int32)
        cv2.polylines(world, [pts], False, (140, 140, 140), 1, cv2.LINE_AA)
    if pose is not None:
        poly(pose["car"], (30, 30, 30), fill=True)
        y = 22
        for line in (pose.get("hud") or f"t={pose['t']:.1f}s {pose['est']}").split("\n"):
            cv2.putText(world, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (20, 20, 20), 1, cv2.LINE_AA)
            y += 18
    cv2.putText(world, "mapa real", (10, _VIEW - 12), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (20, 20, 20), 1, cv2.LINE_AA)
    if bev is None:
        side = _fit_panel(None, _VIEW, "BEV")
    else:
        side = _fit_panel(bev, _VIEW, "BEV")
    mid = _fit_panel(cam, _VIEW, "camara, antes del BEV")
    know = _fit_panel(dig, _VIEW, "mapa del carro")
    return np.hstack([world, know, mid, side])


_STATE_COLORS = {
    "S": "#2ecc71",
    "C": "#3498db",
    "G": "#e74c3c",
    "R": "#9b59b6",
    "T": "#95a5a6",
    "I": "#f39c12",
    "?": "#bdc3c7",
}


def _parse_kv(s: str) -> tuple[str, str]:
    name, val = s.split("=", 1)
    return name.strip(), val.strip()


def _closed(points):
    arr = np.array(points, dtype=float)
    return np.vstack([arr, arr[0]])


def _seat_dots():
    dots = []
    for section in SECTIONS_CW:
        for h, w in SEATS.values():
            dots.append(section_to_world(section, h, w))
    return dots


def _body_polygon(x, y, heading, params):
    c = body_corners(
        x,
        y,
        heading,
        params.vehicle.length_mm,
        params.vehicle.width_mm,
        params.vehicle.rear_overhang_mm,
    )
    return c


def _draw_static(ax, field):
    """Pista, una vez. El carro es otro parche y solo se le mueven los vértices."""
    ax.clear()
    outer = _closed(field.outer)
    inner = _closed(field.inner)
    ax.fill(outer[:, 0], outer[:, 1], color="#d7c4a8", zorder=0)
    ax.fill(inner[:, 0], inner[:, 1], color="k", zorder=1)
    ax.plot(outer[:, 0], outer[:, 1], color="k", lw=2, zorder=2)
    for (x0, y0), (x1, y1) in orange_segments():
        ax.plot([x0, x1], [y0, y1], color="darkorange", lw=1, zorder=2)
    for (x0, y0), (x1, y1) in blue_segments():
        ax.plot([x0, x1], [y0, y1], color="royalblue", lw=1, zorder=2)
    seats = np.array(_seat_dots())
    ax.scatter(seats[:, 0], seats[:, 1], s=8, c="0.35", zorder=3)
    half = SIGN_MM / 2.0
    for sign in field.signs:
        xs = [sign.x - half, sign.x + half, sign.x + half, sign.x - half]
        ys = [sign.y - half, sign.y - half, sign.y + half, sign.y - half]
        ax.fill(xs, ys, color="red" if sign.color == "Red" else "limegreen", zorder=4)
    for box in field.barriers:
        poly = _closed(box)
        ax.fill(poly[:, 0], poly[:, 1], color="magenta", zorder=4)
    ax.set_aspect("equal")
    ax.set_xlim(-OUTER_HALF_MM - 80, OUTER_HALF_MM + 80)
    ax.set_ylim(-OUTER_HALF_MM - 80, OUTER_HALF_MM + 80)
    trail_line, = ax.plot([], [], color="0.55", lw=0.8, zorder=5)
    car = Polygon([[0, 0]], closed=True, facecolor="0.15", edgecolor="k", lw=1, zorder=6)
    ax.add_patch(car)
    return car, trail_line


def _draw_world(ax, field, veh, trail, est, params):
    ax.clear()
    outer = _closed(field.outer)
    inner = _closed(field.inner)
    ax.fill(outer[:, 0], outer[:, 1], color="#d7c4a8", zorder=0)
    ax.fill(inner[:, 0], inner[:, 1], color="k", zorder=1)
    ax.plot(outer[:, 0], outer[:, 1], color="k", lw=2, zorder=2)
    for (x0, y0), (x1, y1) in orange_segments():
        ax.plot([x0, x1], [y0, y1], color="darkorange", lw=1, zorder=2)
    for (x0, y0), (x1, y1) in blue_segments():
        ax.plot([x0, x1], [y0, y1], color="royalblue", lw=1, zorder=2)
    seats = np.array(_seat_dots())
    ax.scatter(seats[:, 0], seats[:, 1], s=8, c="0.35", zorder=3)
    half = SIGN_MM / 2.0
    for sign in field.signs:
        xs = [sign.x - half, sign.x + half, sign.x + half, sign.x - half, sign.x - half]
        ys = [sign.y - half, sign.y - half, sign.y + half, sign.y + half, sign.y - half]
        ax.fill(xs, ys, color="red" if sign.color == "Red" else "limegreen", zorder=4)
    for box in field.barriers:
        poly = _closed(box)
        ax.fill(poly[:, 0], poly[:, 1], color="magenta", zorder=4)
    if len(trail) > 1:
        pts = np.array(trail)
        ax.plot(pts[:, 0], pts[:, 1], color="0.55", lw=0.8, zorder=5)
    poly = _body_polygon(veh.x, veh.y, veh.heading_deg, params)
    ax.fill(poly[:, 0], poly[:, 1], color="0.15", zorder=6, ec="k", lw=1)
    h = math.radians(veh.heading_deg)
    for side, col in ((-1, "0.3"), (1, "0.3")):
        ox, oy = robot_to_world(70 * side, 50, veh.x, veh.y, veh.heading_deg)
        dx = math.cos(h) * side
        dy = -math.sin(h) * side
        ax.plot([ox, ox + dx * 80], [oy, oy + dy * 80], color=col, lw=0.6, zorder=7)
    fov = 35
    hf = math.radians(fov)
    for ang in (-hf, hf):
        dx = math.sin(h + ang) * 200
        dy = math.cos(h + ang) * 200
        ax.plot([veh.x, veh.x + dx], [veh.y, veh.y + dy], color="cyan", lw=0.5, alpha=0.5)
    ax.set_aspect("equal")
    ax.set_xlim(-OUTER_HALF_MM - 80, OUTER_HALF_MM + 80)
    ax.set_ylim(-OUTER_HALF_MM - 80, OUTER_HALF_MM + 80)
    ax.set_title(f"est={est or '?'}")


def _run_once(args, seed: int, interactive: bool):
    params = TwinParams()
    for line in params.describe():
        print(line, flush=True)

    calib = None if args.calib == "none" else args.calib
    fw_overrides = {}
    for item in args.fw:
        k, v = _parse_kv(item)
        fw_overrides[k] = v
    pi_overrides = {}
    for item in args.pi:
        k, v = _parse_kv(item)
        pi_overrides[k] = ast.literal_eval(v)

    sim = Sim(
        seed,
        start=args.start or None,
        preset=args.preset,
        fw_overrides=fw_overrides or None,
        pi_overrides=pi_overrides or None,
        calib_npz=calib,
        max_time_s=args.max_time,
        log_dir=args.log_dir,
    )
    trail: list[tuple[float, float]] = []
    frames: list[np.ndarray] = []
    keys = {"cmd": None, "pause": False}

    view_q = key_q = view_proc = None
    if interactive:
        ctx = mp.get_context("spawn")
        view_q = ctx.Queue(maxsize=1)
        key_q = ctx.Queue()
        view_proc = ctx.Process(target=_view_window, args=(view_q, key_q), daemon=True)
        view_proc.start()

    need_fig = bool(args.save_gif)
    if need_fig:
        fig, (ax_world, ax_cam, ax_bev) = plt.subplots(1, 3, figsize=(14, 5))
    else:
        fig = ax_world = ax_cam = ax_bev = None

    frame_i = 0
    n_signs = -1
    field_sent = False
    speed = {"v": float(args.speed)}
    prev_t = None

    def _hud(est, t, ack, extra) -> str:
        def g(k):
            i = (ack or "").find("," + k + "=")
            return (ack[i + len(k) + 2:].split(",")[0] if i >= 0 else "-")
        return "\n".join([
            extra,
            f"semilla {seed}  t={t:.1f}s  {est}  vel x{speed['v']:g}",
            f"dL={g('dL')} dR={g('dR')} dF={g('dF')} ang={g('ang')} srv={g('srv')}",
            "espacio inicia/pausa   + - velocidad",
            "r reinicia   n semilla   esc sale",
        ])

    def _pace(dt) -> bool:
        budget = min(0.05, max(0.0, dt / max(speed["v"], 0.1)))
        end = time.perf_counter() + budget
        while time.perf_counter() < end:
            if not _poll_keys():
                return False
            time.sleep(0.005)
        return True

    def _poll_keys() -> bool:
        if key_q is None:
            return True
        try:
            while True:
                k = key_q.get_nowait()
                if k == " ":
                    keys["pause"] = not keys["pause"]
                elif k == "+":
                    speed["v"] = min(20.0, speed["v"] + 1.0)
                elif k == "-":
                    speed["v"] = max(1.0, speed["v"] - 1.0)
                elif k == "q":
                    keys["cmd"] = "quit"
                    return False
                elif k in ("r", "n"):
                    keys["cmd"] = "restart" if k == "r" else "new"
                    return False
        except Empty:
            return True

    def on_frame(view: SimFrameView) -> bool:
        nonlocal frame_i, n_signs, field_sent, prev_t
        frame_i += 1
        if not _poll_keys():
            return False
        while keys["pause"]:
            time.sleep(0.05)
            if not _poll_keys():
                return False
        fr = view.frame_result
        est = view.est or "?"
        show_cam = frame_i % args.render_every == 0
        if interactive:
            if not field_sent or len(view.field.signs) != n_signs:
                _offer(view_q, ("field", _field_payload(view.field)))
                n_signs = len(view.field.signs)
                field_sent = True
            veh = view.vehicle
            bev_dbg = None
            if show_cam and fr.bev_frame is not None:
                dx, dy = getattr(fr, "bev_shift", (0, 0))
                bev_dbg = draw_bev_debug(
                    fr.bev_frame, fr.path_points, fr.lookahead_pt,
                    fr.bev_obstacles, fr.steer_deg, fr.pp_active,
                    line_info=fr.line_info,
                    bev_obstacles_beyond=fr.bev_obstacles_beyond,
                    robot_xy=(C.ROBOT_BEV_X + dx, C.ROBOT_BEV_Y + dy),
                )
            ack = view.frame_result.serial_ack if view.frame_result is not None else ""
            _offer(view_q, ("frame", {
                "car": _body_polygon(veh.x, veh.y, veh.heading_deg, params).tolist(),
                "xy": (veh.x, veh.y),
                "est": est,
                "t": view.t,
                "bev": bev_dbg,
                "cam": None if fr.processed_frame is None else fr.processed_frame.copy(),
                "dig": None if fr.dig_map is None else fr.dig_map.copy(),
                "hud": _hud(est, view.t, ack, "pausa" if keys["pause"] else "corriendo"),
            }))
            dt = 0.0 if prev_t is None else max(0.0, view.t - prev_t)
            prev_t = view.t
            return _pace(dt)
        if fig is None:
            return True
        trail.append((view.vehicle.x, view.vehicle.y))
        _draw_world(ax_world, view.field, view.vehicle, trail, est, params)
        if show_cam:
            if fr.processed_frame is not None:
                ax_cam.imshow(cv2.cvtColor(fr.processed_frame, cv2.COLOR_BGR2RGB))
                ax_cam.set_title("cámara")
                ax_cam.axis("off")
            if fr.bev_frame is not None:
                ax_bev.imshow(cv2.cvtColor(fr.bev_frame, cv2.COLOR_BGR2RGB))
                ax_bev.set_title("BEV")
                ax_bev.axis("off")
        fig.suptitle(f"t={view.t:.1f}s est={est} tc={view.tc}", fontsize=9)
        if args.save_gif and show_cam:
            fig.canvas.draw()
            buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
            buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (4,))
            frames.append(buf[:, :, :3].copy())
        return True

    go = True
    if interactive and view_q is not None:
        preview = randomize(seed, start=sim.start)
        view_q.put(("field", _field_payload(preview)))
        field_sent = True
        n_signs = len(preview.signs)

        def _parked() -> None:
            car = _body_polygon(
                preview.start_x, preview.start_y, preview.start_heading, params)
            _offer(view_q, ("frame", {
                "car": car.tolist(),
                "xy": (preview.start_x, preview.start_y),
                "est": "WAIT",
                "t": 0.0,
                "bev": None,
                "hud": _hud("WAIT", 0.0, "", "espacio para iniciar"),
            }))

        _parked()
        print(f"[TWIN] semilla {seed}. espacio para iniciar.", flush=True)
        while True:
            try:
                k = key_q.get(timeout=0.1)
            except Empty:
                continue
            if k == " ":
                break
            if k == "+":
                speed["v"] = min(20.0, speed["v"] + 1.0)
                _parked()
            elif k == "-":
                speed["v"] = max(1.0, speed["v"] - 1.0)
                _parked()
            elif k == "q":
                keys["cmd"] = "quit"
                go = False
                break
            elif k in ("r", "n"):
                keys["cmd"] = "restart" if k == "r" else "new"
                go = False
                break
    if go:
        result = sim.run(frame_callback=on_frame if (interactive or need_fig) else None)
    else:
        result = RunResult(
            metrics={}, trace=[], field=None, stop_reason="no_inicio",
        )
    print(f"[TWIN] stop={result.stop_reason} metrics={result.metrics}", flush=True)
    if args.print_log and sim.log_dir:
        log = Path(sim.log_dir) / "pi.log"
        if log.is_file():
            print(log.read_text(encoding="utf-8")[-4000:], flush=True)

    if args.save_gif and frames:
        import imageio.v2 as imageio

        frame_dt = sim.pi_period_s * args.render_every / max(args.speed, 0.1)
        imageio.mimsave(args.save_gif, frames, duration=frame_dt)
        print(f"[TWIN] GIF {args.save_gif}", flush=True)
    if fig is not None:
        plt.close(fig)
    return result, keys, view_q, key_q, view_proc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--preset", default="baseline")
    ap.add_argument("--start", default="",
                    help="centro, zona o cajon; vacío = el del preset")
    ap.add_argument("--calib", default="pure_pursuit/twin/calib/bev_calib_main_2026-08-19.npz")
    ap.add_argument("--fw", action="append", default=[])
    ap.add_argument("--pi", action="append", default=[])
    ap.add_argument("--max-time", type=float, default=180.0)
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--save-gif", type=str, default="")
    ap.add_argument("--render-every", type=int, default=3)
    ap.add_argument("--speed", type=float, default=5.0,
                    help="reproducción: 5 = la ventana va 5 veces más rápido que la carrera")
    ap.add_argument("--print-log", action="store_true")
    ap.add_argument("--log-dir", type=str, default="")
    args = ap.parse_args()
    if args.log_dir:
        args.log_dir = Path(args.log_dir)

    interactive = not args.no_show and not args.save_gif
    if not interactive:
        import matplotlib

        matplotlib.use("Agg")

    seed = args.seed
    view_q = key_q = view_proc = None
    try:
        while True:
            if view_proc is not None:
                try:
                    view_q.put_nowait(("stop",))
                except Exception:
                    pass
                view_proc.join(timeout=1.0)
            result, keys, view_q, key_q, view_proc = _run_once(args, seed, interactive)
            if not interactive:
                break
            print("[TWIN] r=reinicio  n=nueva semilla  espacio=pausa  esc=salir", flush=True)
            keys["cmd"] = None
            while keys["cmd"] is None:
                try:
                    k = key_q.get(timeout=0.1)
                except Empty:
                    continue
                if k == " ":
                    continue
                keys["cmd"] = {"r": "restart", "n": "new", "q": "quit"}.get(k, k)
            if keys["cmd"] == "quit":
                break
            if keys["cmd"] == "new":
                seed = randomize(None).seed
                print(f"[TWIN] semilla {seed}", flush=True)
            else:
                print(f"[TWIN] reinicio semilla {seed}", flush=True)
    finally:
        if view_q is not None:
            try:
                view_q.put_nowait(("stop",))
            except Exception:
                pass
        if view_proc is not None:
            view_proc.join(timeout=1.0)


if __name__ == "__main__":
    main()
