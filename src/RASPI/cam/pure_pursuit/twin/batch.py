"""Corridas en lote del twin (multiproceso)."""

from __future__ import annotations

import argparse
import csv
import json
import multiprocessing as mp
from pathlib import Path

from pure_pursuit.twin.sim import Sim, _DEFAULT_CALIB


def _parse_seeds(spec: str) -> list[int]:
    out: list[int] = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            out.extend(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def _run_one(args: tuple) -> dict:
    seed, preset, max_time, calib, out_dir = args
    log_dir = Path(out_dir) / f"seed_{seed}" if out_dir else None
    if calib == "none":
        calib_path = None
    elif calib == "default":
        calib_path = _DEFAULT_CALIB
    else:
        calib_path = calib
    sim = Sim(
        seed,
        preset=preset,
        calib_npz=calib_path,
        max_time_s=max_time,
        log_dir=log_dir,
    )
    result = sim.run()
    row = {"seed": seed, **result.metrics}
    if out_dir:
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        with open(Path(out_dir) / f"seed_{seed}.json", "w", encoding="utf-8") as f:
            json.dump(
                {"metrics": result.metrics, "stop_reason": result.stop_reason},
                f,
                indent=2,
            )
    return row


def main() -> None:
    ap = argparse.ArgumentParser(description="Batch twin Fox")
    ap.add_argument("--preset", default="baseline")
    ap.add_argument("--seeds", default="1-20")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--max-time", type=float, default=180.0)
    ap.add_argument("--calib", default="default", help="ruta npz, none o default")
    ap.add_argument("--out", required=True, help="CSV de salida")
    args = ap.parse_args()

    seeds = _parse_seeds(args.seeds)
    out_path = Path(args.out)
    out_dir = out_path.parent / (out_path.stem + "_runs")

    tasks = [
        (s, args.preset, args.max_time, args.calib or "default", str(out_dir))
        for s in seeds
    ]
    ctx = mp.get_context("spawn")
    with ctx.Pool(args.jobs) as pool:
        rows = pool.map(_run_one, tasks)

    fieldnames = sorted({k for r in rows for k in r.keys()})
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)

    n = len(rows)
    finished = sum(1 for r in rows if r.get("finished"))
    times = [r["total_time_s"] for r in rows if r.get("finished")]
    corners = [r.get("mean_corner_time_s", 0) for r in rows if r.get("finished")]
    g_times = [r.get("mean_time_in_g_s", 0) for r in rows if r.get("finished")]
    collisions: dict[str, int] = {}
    violations = 0
    for r in rows:
        for c in r.get("collisions") or []:
            if isinstance(c, dict):
                cause = c.get("cause", "?")
            else:
                cause = str(c)
            collisions[cause] = collisions.get(cause, 0) + 1
        violations += len(r.get("pass_side_violations") or [])

    def _median(xs: list[float]) -> float:
        if not xs:
            return 0.0
        xs = sorted(xs)
        m = len(xs) // 2
        return xs[m] if len(xs) % 2 else 0.5 * (xs[m - 1] + xs[m])

    print(f"seeds={n} finished={finished}/{n} ({100*finished/n:.0f}%)")
    if times:
        print(f"total_time_s mean={sum(times)/len(times):.1f} median={_median(times):.1f}")
    if corners:
        print(f"mean_corner_time_s mean={sum(corners)/len(corners):.2f}")
    if g_times:
        print(f"mean_time_in_g_s mean={sum(g_times)/len(g_times):.2f}")
    if collisions:
        print("collisions:", collisions)
    print(f"pass_side_violations total={violations}")
    print(f"CSV {out_path}")


if __name__ == "__main__":
    main()
