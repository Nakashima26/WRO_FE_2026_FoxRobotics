"""Mide la vuelta 1 en telemetría Orillas y trazas de twin/tools/drive.py."""

import argparse
import ast
import csv
import json
import math
import re
from bisect import bisect_right
from collections import Counter
from pathlib import Path

from pure_pursuit.wro_field import SEATS, section_to_world

REAL_FIELD = {
    ("W", "T2", "Green"), ("S", "T3", "Red"), ("S", "T4", "Green"),
    ("E", "T3", "Red"), ("N", "X2", "Red"),
}
ACK_FIELDS = re.compile(r"(?<=,)([a-zA-Z]+)=([^,*|\s]*)")
TX_RV = re.compile(r",rv=(-?\d+)(?=[,*|\s]|$)")


def parse_ack(raw):
    fields = dict(ACK_FIELDS.findall(raw))
    if not {"tc", "od", "srv", "pw"} <= fields.keys():
        return {}
    for name in ("tc", "od", "srv", "pw", "rn"):
        if name in fields:
            fields[name] = float(fields[name])
    return fields


def read_status(path):
    rows = []
    events = []
    for line in path.read_text().splitlines():
        item = json.loads(line)
        status = item.get("s", {})
        if status.get("armed"):
            rows.append((item["t"], status))
        events.extend(item.get("ev", []))
    return rows, sorted(events)


def confirmations(rows):
    seen = set()
    result = []
    for t, status in rows:
        m = status.get("map", {})
        if "pose" not in m:
            continue
        x, y, _ = m["pose"]
        for signal in map(tuple, m.get("confirmed", [])):
            if signal in seen:
                continue
            seen.add(signal)
            sec, seat, _ = signal
            sx, sy = section_to_world(sec, *SEATS[seat])
            result.append((t, signal, math.hypot(sx - x, sy - y), signal in REAL_FIELD))
    return result


def load_real(root, run):
    recent = run >= 252
    folder = root / ("real1125" if recent else "real1124")
    name = f"orillas{run}"
    rows, events = read_status(folder / f"{name}_tele.jsonl")
    if not rows:
        return [], [], [], [], [], []
    acks = [(t, s["ack"]) for t, s in rows if {"od", "tc", "pw", "srv"} <= s.get("ack", {}).keys()]
    plans = [(t, e) for t, e in events if e.startswith("[ROUTE] plan#")]
    conf = confirmations(rows)
    tx = []
    uploads = []
    stops = []
    if not recent:
        log = folder / f"{name}.log.txt"
        if log.exists():
            # El log y la telemetría tienen relojes distintos. Alinear por GPIO GO.
            matches = []
            start = next((t for t, e in events if "[GPIO]" in e and "GO" in e), None)
            for line in log.read_text(errors="replace").splitlines():
                if "\t" not in line:
                    continue
                stamp, msg = line.split("\t", 1)
                try:
                    t = float(stamp)
                except ValueError:
                    continue
                matches.append((t, msg))
            log_start = next((t for t, msg in matches if "[GPIO]" in msg and "GO" in msg), None)
            if start is not None and log_start is not None:
                offset = start - log_start
                log_acks = {}
                up_start = None
                for t, msg in matches:
                    t += offset
                    if "[SERIAL] RX: ACK:" in msg:
                        ack = parse_ack(msg.split("[SERIAL] RX: ACK:", 1)[-1])
                        if ack:
                            log_acks[t] = ack
                    if "TX: V2," in msg:
                        command = msg.split("TX: V2,", 1)[1].split("|", 1)[0]
                        match = TX_RV.search(command)
                        tx.append((t, int(match[1]) if match else None))
                    if "[SERIAL] TX: RTB," in msg:
                        up_start = t
                    if "[SERIAL] TX: RTE," in msg and up_start is not None:
                        uploads.append((up_start, t))
                        up_start = None
                if log_acks:
                    acks = sorted(log_acks.items())
    else:
        ackfile = folder / "acks" / f"o{run}.tsv"
        if ackfile.exists():
            # Sin sincronización publicada entre las dos capturas, usar el
            # primer ACK con potencia positiva como referencia común.
            with ackfile.open() as f:
                full = [(float(r["t"]), {k: float(r[k]) for k in ("tc", "od", "pw", "srv")}
                         | {"est": r["est"]}) for r in csv.DictReader(f, delimiter="\t")]
            a0 = first_power(acks)
            f0 = first_power(full)
            if a0 is not None and f0 is not None:
                acks = [(t + a0 - f0, a) for t, a in full]
    return acks, tx, plans, conf, uploads, stops


def first_power(samples):
    return next((t for t, a in samples if a["pw"] > 0), None)


def load_twin(folder):
    with (folder / "trace.csv").open() as f:
        acks = [(float(r["t"]), a) for r in csv.DictReader(f) if (a := parse_ack(r["ack"]))]
    decisions = folder / "decisions.csv"
    tx, plans, status, uploads, stops, corner, pending = [], [], [], [], [], [], []
    if decisions.exists():
        with decisions.open() as f:
            prev = None
            up_start = hold_start = None
            last = None
            for r in csv.DictReader(f):
                t = float(r["t"])
                if last is not None:
                    if last[1]:
                        corner.append((last[0], t))
                    if last[2]:
                        pending.append((last[0], t))
                last = (t, r.get("corner") == "1", r["pending"] == "1")
                tx.append((t, int(float(r["rv"])) if r["rv"] else None))
                if r["plan"] != prev and r["plan"] != "0":
                    plans.append((t, r["plan"]))
                prev = r["plan"]
                if int(r["upload"]) and up_start is None:
                    up_start = t
                elif not int(r["upload"]) and up_start is not None:
                    uploads.append((up_start, t))
                    up_start = None
                if int(r["hold"]) and hold_start is None:
                    hold_start = t
                elif not int(r["hold"]) and hold_start is not None:
                    stops.append((hold_start, t))
                    hold_start = None
                confirmed = [(*key, value) for key, value in ast.literal_eval(r["conf"])]
                status.append((t, {"map": {"pose": [float(r["x"]), float(r["y"]), 0],
                                          "confirmed": confirmed}}))
    return acks, tx, plans, confirmations(status), uploads, stops, corner, pending


def overlap(intervals, start, end):
    return sum(max(0, min(b, end) - max(a, start)) for a, b in intervals)


def breakdown(acks, tx, plans, confirmed, uploads, stops, corner=None, pending=None):
    t0 = first_power(acks)
    if t0 is None:
        return {}
    acks = [(t, a) for t, a in acks if t >= t0]
    times = [t for t, _ in tx]
    state = {}
    for tc in range(5):
        state[tc] = next(((i, t) for i, (t, a) in enumerate(acks) if a["tc"] >= tc), None)
    results = {"segments": [], "confirmed": [], "exit": None, "v2_v3": []}
    for tc in (4, 8):
        start = next((t for t, a in acks if a["tc"] >= tc), None)
        end = next((t for t, a in acks if a["tc"] >= tc + 4), None)
        if start is not None and end is not None and end > start:
            side = [1 if a["srv"] > 80 else -1 for t, a in acks
                    if start <= t < end and not 70 <= a["srv"] <= 80]
            crosses = sum(a != b for a, b in zip(side, side[1:]))
            results["v2_v3"].append([round(end - start, 3), round(crosses / (end - start), 2)])
    settled = next(((t, a) for t, a in acks if a["est"] == "S"), None)
    if settled:
        results["exit"] = [round(settled[0] - t0, 3), settled[1]["od"] - acks[0][1]["od"]]
    for tc in range(4):
        if state[tc] is None:
            break
        i, start = state[tc]
        complete = state[tc + 1] is not None
        j, end = state[tc + 1] or (len(acks) - 1, acks[-1][0])
        if j <= i:
            continue
        duration = end - start
        crosses = side = 0
        pwm = Counter()
        rv = Counter()
        plant = Counter()
        plant_time = Counter()
        spans = [(t, a) for t, a in tx if start < t < end]
        k = bisect_right(times, start) - 1
        if k >= 0:
            spans.insert(0, (start, tx[k][1]))
        for (t, cap), (u, _) in zip(spans, spans[1:] + [(end, None)]):
            if cap is not None:
                rv[cap] += u - t
        for (t, a), (u, b) in zip(acks[i:j], acks[i + 1:j + 1]):
            s = (1 if a["srv"] > 80 else -1 if a["srv"] < 70 else 0)
            if s:
                crosses += bool(side and side != s)
                side = s
            pwm[a["pw"]] += u - t
            if a["est"] == "S" and a["pw"] > 0 and 0 < u - t <= 0.15:
                cell = (int(a["pw"]) // 25 * 25, "recto" if abs(a["srv"] - 75) <= 15 else "giro")
                plant[cell] += b["od"] - a["od"]
                plant_time[cell] += u - t
        ds = acks[j][1]["od"] - acks[i][1]["od"]
        results["segments"].append({
            "tc": tc, "complete": complete, "s": round(duration, 3), "od_mm": ds,
            "v_mm_s": round(ds / duration) if duration else None,
            "crosses": crosses, "crosses_s": round(crosses / duration, 2) if duration else None,
            "pwm_s": {int(k): round(v, 3) for k, v in pwm.most_common(5)},
            "rv_s": {k: round(v, 3) for k, v in rv.most_common(5)},
            "rv_avg": round(sum(k * v for k, v in rv.items()) / sum(rv.values()), 1) if rv else None,
            "plant": {f"{pwm_bin}:{steer}": [round(seconds, 3), round(plant[(pwm_bin, steer)] / seconds)]
                      for (pwm_bin, steer), seconds in sorted(plant_time.items())},
            "upload_s": round(overlap(uploads, start, end), 3) if uploads else None,
            "hold_s": round(overlap(stops, start, end), 3) if stops else None,
            "corner_s": round(overlap(corner, start, end), 3) if corner is not None else None,
            "pending_s": round(overlap(pending, start, end), 3) if pending is not None else None,
            "plans": sum(start <= t < end for t, _ in plans),
        })
    for t, (sec, seat, color), dist, true in confirmed:
        if state[4] and t >= state[4][1]:
            continue
        results["confirmed"].append({
            "signal": f"{sec}/{seat} {color}", "s": round(t - t0, 3),
            "distance_mm": round(dist), "matches_s20": true,
        })
    return results


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--real", type=Path, required=True, help="Directorio con real1124 y real1125")
    p.add_argument("twin", type=Path, nargs="*")
    args = p.parse_args()
    data = {f"o{run}": breakdown(*load_real(args.real, run)) for run in range(247, 262)}
    data.update({folder.name: breakdown(*load_twin(folder)) for folder in args.twin})
    print(json.dumps(data, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
