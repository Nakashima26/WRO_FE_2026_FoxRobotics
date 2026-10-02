"""Métricas JSON-serializables a partir del trace de Sim."""

from __future__ import annotations

import math
from typing import Any

from ..wro_field import INNER_HALF_MM, OUTER_HALF_MM, SIGN_MM, SECTIONS_CW, section_to_world
from .world import body_corners, robot_to_world


def _parse_ack_field(ack: str, key: str) -> str | None:
    if not ack:
        return None
    needle = f"{key}="
    idx = ack.find(needle)
    if idx < 0:
        return None
    rest = ack[idx + len(needle) :]
    return rest.split(",")[0]


def _parse_est(ack: str) -> str | None:
    v = _parse_ack_field(ack, "est")
    if v in ("C", "I", "E", "T"):
        return v
    if v in ("G", "R", "S"):
        return v
    return None


def _section_along_mm(section: str, x: float, y: float) -> float:
    """Coordenada a lo largo de la recta (0 = inicio CW en esa sección)."""
    if section == "N":
        return y + OUTER_HALF_MM
    if section == "S":
        return OUTER_HALF_MM - y
    if section == "E":
        return OUTER_HALF_MM - x
    if section == "W":
        return x + OUTER_HALF_MM
    return 0.0


def _lateral_side(section: str, x: float, y: float) -> float:
    """Positivo = izquierda respecto al sentido CW en la recta."""
    if section == "N":
        return x
    if section == "S":
        return -x
    if section == "E":
        return -y
    if section == "W":
        return y
    return 0.0


def _min_body_clearance(
    x: float,
    y: float,
    heading_deg: float,
    length_mm: float,
    width_mm: float,
    rear_overhang_mm: float,
    signs: list,
) -> tuple[float, float]:
    corners = body_corners(x, y, heading_deg, length_mm, width_mm, rear_overhang_mm)
    min_wall = float("inf")
    for cx, cy in corners:
        d_out = OUTER_HALF_MM - max(abs(cx), abs(cy))
        min_wall = min(min_wall, d_out)
        if abs(cx) < INNER_HALF_MM and abs(cy) < INNER_HALF_MM:
            min_wall = min(min_wall, INNER_HALF_MM - max(abs(cx), abs(cy)))
    min_sign = float("inf")
    half = SIGN_MM / 2.0
    for sign in signs:
        dx = sign.x - x
        dy = sign.y - y
        d = math.hypot(dx, dy) - half - width_mm * 0.5
        min_sign = min(min_sign, d)
    return float(min_wall), float(min_sign if math.isfinite(min_sign) else 0.0)


def compute_metrics(
    trace: list[dict[str, Any]],
    field,
    vehicle_params,
    *,
    armed_t: float | None,
    stop_reason: str,
    collision_cause: str | None,
    collision_t: float | None,
) -> dict[str, Any]:
    if not trace:
        return {
            "finished": False,
            "stop_reason": stop_reason,
            "total_time_s": 0.0,
            "turns_completed": 0,
            "lap_times_s": [],
            "corners": [],
            "time_per_state_s": {},
            "collisions": [],
            "min_clearance_wall_mm": 0.0,
            "min_clearance_sign_mm": 0.0,
            "pass_side_violations": [],
            "signs_passed": 0,
            "distance_forward_mm": 0.0,
            "distance_backward_mm": 0.0,
            "mean_speed_mm_s": 0.0,
            "frames_processed": 0,
            "mean_pi_frame_ms": 0.0,
        }

    t0 = armed_t if armed_t is not None else trace[0]["t"]
    last = trace[-1]
    t_end = last["t"]

    state_time: dict[str, float] = {}
    prev_t = trace[0]["t"]
    prev_est = trace[0].get("est") or "?"
    reverse_s = 0.0
    dist_fwd = 0.0
    dist_back = 0.0
    min_wall = float("inf")
    min_sign = float("inf")
    frame_times: list[float] = []
    frames = 0
    tc_max = 0
    tpr = 12

    sign_cross_prev: dict[int, float] = {}
    violations: list[dict[str, Any]] = []
    signs_passed = 0

    corners: list[dict[str, Any]] = []
    corner_open: dict[str, Any] | None = None
    lap_times: list[float] = []
    last_tc_lap = 0
    lap_t0 = t0

    for i, row in enumerate(trace):
        t = row["t"]
        dt = t - prev_t if i > 0 else 0.0
        est = row.get("est") or prev_est
        state_time[est] = state_time.get(est, 0.0) + dt
        if row.get("motor_dir", 0) < 0:
            reverse_s += dt
        if i > 0:
            p0 = trace[i - 1]
            dx = row["x"] - p0["x"]
            dy = row["y"] - p0["y"]
            ds = math.hypot(dx, dy)
            if row.get("motor_dir", 0) < 0:
                dist_back += ds
            else:
                dist_fwd += ds

        if row.get("pi_frame"):
            frames += 1
            ft = row.get("pi_frame_ms")
            if ft is not None:
                frame_times.append(float(ft))

        ack = row.get("ack") or ""
        tc_s = _parse_ack_field(ack, "tc")
        if tc_s is not None:
            try:
                tc_val = int(tc_s)
                tc_max = max(tc_max, tc_val)
                if tc_val // 4 > last_tc_lap and tc_val > 0:
                    lap_times.append(t - lap_t0)
                    lap_t0 = t
                    last_tc_lap = tc_val // 4
            except ValueError:
                pass
        tpr_s = _parse_ack_field(ack, "tpr")
        if tpr_s is not None:
            try:
                tpr = int(tpr_s)
            except ValueError:
                pass

        # Esquinas: C … G … S
        if est == "C" and corner_open is None:
            corner_open = {"t_c": t, "t_g": None, "t_s": None, "time_g_s": 0.0, "reverse_s": 0.0}
        if corner_open is not None:
            if est == "G":
                if corner_open["t_g"] is None:
                    corner_open["t_g"] = t
            if est == "G":
                corner_open["time_g_s"] += dt
                if row.get("motor_dir", 0) < 0:
                    corner_open["reverse_s"] += dt
            if est == "S" and corner_open.get("t_g") is not None:
                corner_open["t_s"] = t
                corner_open["duration_s"] = t - corner_open["t_c"]
                corners.append(corner_open)
                corner_open = None

        w, s = _min_body_clearance(
            row["x"],
            row["y"],
            row["heading"],
            vehicle_params.length_mm,
            vehicle_params.width_mm,
            vehicle_params.rear_overhang_mm,
            field.signs,
        )
        min_wall = min(min_wall, w)
        min_sign = min(min_sign, s)

        for si, sign in enumerate(field.signs):
            along = _section_along_mm(sign.section, row["x"], row["y"])
            prev_along = sign_cross_prev.get(si)
            sign_cross_prev[si] = along
            if prev_along is None:
                continue
            if prev_along < along - 50.0 and along >= 400.0:
                signs_passed += 1
                lat = _lateral_side(sign.section, row["x"], row["y"])
                want_left = sign.color == "Red"
                ok = lat > 0 if want_left else lat < 0
                if not ok:
                    violations.append(
                        {
                            "sign": f"{sign.color} {sign.section}/{sign.seat}",
                            "t": t,
                            "lateral_mm": lat,
                        }
                    )

        prev_t = t
        prev_est = est

    finished = stop_reason in ("race_finished", "terminado") or (
        tc_max >= tpr and state_time.get("T", 0.0) > 0.1
    )

    corner_durations = [c["duration_s"] for c in corners if "duration_s" in c]
    mean_corner = sum(corner_durations) / len(corner_durations) if corner_durations else 0.0
    mean_g = sum(c.get("time_g_s", 0.0) for c in corners) / len(corners) if corners else 0.0

    total_time = max(0.0, t_end - t0) if armed_t is not None else 0.0
    mean_speed = (dist_fwd + dist_back) / total_time if total_time > 1e-6 else 0.0

    return {
        "finished": finished,
        "stop_reason": stop_reason,
        "total_time_s": round(total_time, 3),
        "turns_completed": tc_max,
        "tpr": tpr,
        "lap_times_s": [round(x, 3) for x in lap_times],
        "corners": corners,
        "mean_corner_time_s": round(mean_corner, 3),
        "mean_time_in_g_s": round(mean_g, 3),
        "time_per_state_s": {k: round(v, 3) for k, v in sorted(state_time.items())},
        "collisions": (
            [{"cause": collision_cause, "t": collision_t}] if collision_cause else []
        ),
        "min_clearance_wall_mm": round(min_wall if math.isfinite(min_wall) else 0.0, 2),
        "min_clearance_sign_mm": round(min_sign if math.isfinite(min_sign) else 0.0, 2),
        "pass_side_violations": violations,
        "signs_passed": signs_passed,
        "distance_forward_mm": round(dist_fwd, 1),
        "distance_backward_mm": round(dist_back, 1),
        "mean_speed_mm_s": round(mean_speed, 1),
        "frames_processed": frames,
        "mean_pi_frame_ms": round(sum(frame_times) / len(frame_times), 2) if frame_times else 0.0,
        "reverse_motor_s": round(reverse_s, 3),
    }
