"""Replay ParkRefine with recorded poses and ACKs. Run with PYTHONPATH=src/RASPI/cam."""

import argparse
import json
from pathlib import Path

from pure_pursuit.park_refine import ParkRefine
from pure_pursuit.route_mode import _stall_far_along


def replay(path):
    ref = None
    estimates = {}
    last = None
    with path.open() as f:
        for line in f:
            sample = json.loads(line).get("s", {})
            ack = sample.get("ack", {})
            loc = sample.get("loc", {})
            field = sample.get("map", {})
            tc = ack.get("tc")
            if not loc.get("fixed") or not field.get("direction") or tc is None:
                continue
            if ref is None:
                ref = ParkRefine(field["parking"], field["direction"], field["park_w"])
            scan = tc == 12 and ack.get("est") == "E" and ack.get("pkf") == 0
            if ack.get("est") not in ("S", "C", "R") and not scan:
                continue
            odom = tuple(ack.get(k) for k in ("px", "py", "yaw"))
            if None in odom or odom == last or not 3 <= tc <= 12:
                continue
            last = odom
            line_ack = ",".join(f"{key}={val}" for key, val in ack.items())
            ref.update(tc, line_ack, field["pose"], odom)
            if tc in (11, 12):
                result = ref.estimate()
                if result is not None:
                    estimates[int(tc)] = result
    if ref is None:
        raise ValueError(f"{path}: sin pose fijada y ACK")
    prior = _stall_far_along(ref.section, ref.direction, ref.prior)
    print(path.name, ref.direction, f"pka nominal={prior:.0f}",
          f"tc11={estimates.get(11)}", f"tc12={estimates.get(12)}")


if __name__ == "__main__":
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("telemetry", type=Path, nargs="+")
    for capture in cli.parse_args().telemetry:
        replay(capture)
