"""Contrasta la pose del plan de estacionamiento con la verdad de la traza SIL."""

import argparse
import csv
import math
import re
import sys
from pathlib import Path

import numpy as np

from pure_pursuit.twin.tools.banco.optimizar_park import fuera_mm, ok_t, parse_out
from pure_pursuit.wro_field import (
    PARK_BARRIER_THICK_MM, PARK_GAP_MM, _heading, world_to_section,
)

PHASE_40 = re.compile(r"^\s*(\d+\.\d+) PARK fase 40:.*\(ref=(-?[\d.]+)", re.M)
T1 = re.compile(
    r"^\s*(\d+\.\d+) PARK plan T1 w0=(-?\d+) h0=(-?\d+).*\n"
    r"\s*\d+\.\d+ PARK plan 1 REVERSA EXT th=(-?[\d.]+)", re.M,
)
SEGMENT = re.compile(r"PARK plan (\d+) (?:ADELANTE|REVERSA)")
TRANSFORM = re.compile(r"^TX: .*?,rtx=(-?[\d.]+),rty=(-?[\d.]+),rth=(-?[\d.]+)", re.M)


def ack_fields(row):
    return dict(field.split("=", 1) for field in row["ack"].split(",")[1:] if "=" in field)


def entry_pose(rows, phase_time, ref, t1_time, w_t1, field, direct, run):
    """Reconstruye la pose en el primer ACK estacionario de f40.

    El ACK tiene along y altura del ancla. w_t1 elimina la necesidad de
    adivinar las correcciones de inclinación y cono del firmware de ese banco.
    """
    entries = [r for r in rows if ",pkf=40,pli=0," in r["ack"]]
    if not entries or not (run / "pi.log").is_file():
        return None
    entry = entries[0]
    entry_t = float(entry["t"])
    if not phase_time <= entry_t < t1_time:
        return None
    transforms = TRANSFORM.findall((run / "pi.log").read_text(errors="replace"))
    if not transforms:
        return None
    tx, ty, heading = map(float, transforms[-1])
    bearing = math.radians(heading)
    entry_ack = ack_fields(entry)
    at_t1 = min(entries, key=lambda r: abs(float(r["t"]) - t1_time))
    t1_ack = ack_fields(at_t1)
    px, py = (float(entry_ack[key]) for key in ("px", "py"))
    x = tx + px * math.sin(bearing) - py * math.cos(bearing)
    y = ty + px * math.cos(bearing) + py * math.sin(bearing)
    h_pose, _ = world_to_section(field[1], x, y)
    left_wall = (field[0] == "CW") == direct
    # El primer campo del ACK incluye el prefijo ACK:V2 antes de la coma.
    theta = (ref - float(entry_ack["ang"])) * (1 if left_wall else -1)
    w = w_t1 + (float(entry_ack["pal"]) - float(t1_ack["pal"])) * (1 if direct else -1)
    h = float(entry_ack["pks"]) + h_pose - float(entry_ack["pkh"])
    return entry, w, h, theta


def pose_error(row, field, direct, w, h, theta):
    direction, section, rear = field
    forward = (direction == "CW") == direct
    truth_h, canonical_w = world_to_section(section, float(row["x"]), float(row["y"]))
    near_face = rear + PARK_BARRIER_THICK_MM
    truth_w = canonical_w - near_face if forward else near_face + PARK_GAP_MM - canonical_w
    lane = _heading(section, "CW" if forward else "CCW")
    yaw = (float(row["heading"]) - lane + 180) % 360 - 180
    truth_theta = yaw if forward else -yaw
    return w - truth_w, h - truth_h, theta - truth_theta


def run_metrics(out):
    run = out.with_suffix("")
    key = f"{out.parent.name}/{out.stem}"
    stop = parse_out(out)
    outside = fuera_mm(run)
    ok, elapsed = ok_t(stop, run)
    row = dict(caso=key, ok=int(ok), fuera_mm=outside, tiempo_s=elapsed,
               parada=stop["stop_reason"] if stop else "")
    if not (run / "fw_debug.log").exists():
        return row
    log = (run / "fw_debug.log").read_text(errors="replace")
    row["zigzag"] = sum(int(i) >= 3 for i in SEGMENT.findall(log))
    plan = T1.search(log)
    phase = PHASE_40.search(log)
    # El plan publica w/h/theta al arrancar el primer segmento. Antes mantiene
    # el motor en coast. Se informa también el desfase desde la entrada a f40.
    if not plan or not phase:
        return row
    with (run / "field.txt").open() as stream:
        direction, section, rear = re.search(
            r"dir=(CW|CCW) parking=([NSEW]) parkw=([\d.]+)", stream.readline()
        ).groups()
    with (run / "trace.csv").open(newline="") as stream:
        t, w, h, theta = map(float, plan.groups())
        samples = list(csv.DictReader(stream))
        sample = min(samples, key=lambda item: abs(float(item["t"]) - t))
    if abs(float(sample["t"]) - t) > 0.05:
        return row
    lane = _heading(section, direction)
    direct = abs((float(sample["heading"]) - lane + 180) % 360 - 180) < 90
    row["modo"] = "directo" if direct else "U"
    row["t_desde_f40_s"] = round(t - float(phase[1]), 3)
    field = direction, section, float(rear)
    errors = pose_error(sample, field, direct, w, h, theta)
    row.update(zip(("error_w_mm", "error_h_mm", "error_yaw_deg"),
                   (round(error, 3) for error in errors)))
    entry = entry_pose(samples, float(phase[1]), float(phase[2]), t, w, field, direct, run)
    if entry is not None:
        entry_sample, w, h, theta = entry
        row["t_ack_f40_s"] = round(float(entry_sample["t"]) - float(phase[1]), 3)
        errors = pose_error(entry_sample, field, direct, w, h, theta)
        row.update(zip(("f40_w_mm", "f40_h_mm", "f40_yaw_deg"),
                       (round(error, 3) for error in errors)))
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("banco", type=Path)
    args = parser.parse_args()
    rows = [run_metrics(out) for out in sorted(args.banco.glob("n*/s*.out"))]
    fields = list(dict.fromkeys(key for row in rows for key in row))
    writer = csv.DictWriter(sys.stdout, fieldnames=fields)
    writer.writeheader()
    writer.writerows(rows)
    for column in ("f40_w_mm", "f40_h_mm", "f40_yaw_deg", "error_w_mm", "error_h_mm", "error_yaw_deg"):
        measured = [row for row in rows if column in row]
        for label, sample in (("total", measured), ("ok", [r for r in measured if r["ok"]]),
                              ("falla", [r for r in measured if not r["ok"]])):
            if not sample:
                continue
            error = np.array([row[column] for row in sample])
            print(f"{column} {label} n={len(error)} sesgo={error.mean():.2f} "
                  f"MAE={np.abs(error).mean():.2f} RMSE={np.sqrt(np.mean(error**2)):.2f}",
                  file=sys.stderr)
        for outcome in ("fuera_mm", "zigzag", "ok"):
            pairs = [(abs(r[column]), r[outcome]) for r in measured if math.isfinite(r[outcome])]
            if len(pairs) >= 2 and all(np.ptp(pairs, axis=0)):
                print(f"{column} vs {outcome} (Pearson |error|): "
                      f"{np.corrcoef(np.array(pairs).T)[0, 1]:.3f}", file=sys.stderr)


if __name__ == "__main__":
    main()
