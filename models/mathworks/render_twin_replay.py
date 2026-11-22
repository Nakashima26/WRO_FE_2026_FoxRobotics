"""Render Simulink poses with the Fox twin's map, camera and BEV."""
import argparse
import csv
import json
import shutil
import sys
import tempfile
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src/RASPI/cam"))

from pure_pursuit.chassis_twin import _field_payload, _paint
from pure_pursuit.twin.camera import CameraModel
from pure_pursuit.twin.params import TwinParams
from pure_pursuit.wro_field import randomize

FPS = 20


def render(case_path: Path, replay_path: Path, output: Path) -> None:
    case = json.loads(case_path.read_text())
    with replay_path.open(newline="") as source:
        rows = list(csv.DictReader(source))
    stride = round(1 / (FPS * case["sample_s"]))
    assert stride >= 1 and len(rows) == len(case["commands"]), "Replay and case are not aligned."
    assert all(abs(float(row["time_s"]) - index * case["sample_s"]) < 1e-9
               for index, row in enumerate(rows)), "Unexpected replay sample times."
    params = TwinParams()
    camera = CameraModel(params.camera, params)
    bev, _ = camera.build_bev()
    field = randomize(case["field_seed"], park_pos="random")
    payload = _field_payload(field)
    trail = [(float(row["x_mm"]), float(row["y_mm"])) for row in rows]
    indices = sorted(set(range(0, len(rows), stride)) | {len(rows) - 1})

    with tempfile.TemporaryDirectory() as work:
        root = Path(work)
        (root / "frames").mkdir()
        frame_times = []
        for frame, index in enumerate(indices, start=1):
            row = rows[index]
            x, y = trail[index]
            t, heading = float(row["time_s"]), float(row["heading_deg"])
            cam = camera.render_camera(field, x, y, heading)
            hud = f"t={t:.2f} s | {float(row['speed_mm_s']):.0f} mm/s"
            strip = _paint(payload, trail[:index+1:5] + [(x, y)],
                           {"xy": (x, y), "h": heading, "hud": hud}, None)
            # Keep the original track artwork and omit its live-simulation footer.
            course = strip[:-22, :strip.shape[0]]
            for label, image in (("map", course), ("camera", cam), ("bev", bev.warp(cam))):
                path = root / "frames" / f"{label}_{frame:04d}.jpg"
                if not cv2.imwrite(str(path), image):
                    raise OSError(f"Could not write {path}")
            frame_times.append(t)
        (root / "replay_manifest.json").write_text(json.dumps({
            "seed": case["field_seed"], "times_s": frame_times,
        }))
        shutil.copyfile(Path(__file__).with_name("view_twin_replay.m"), root / "view_twin_replay.m")
        output.parent.mkdir(parents=True, exist_ok=True)
        shutil.make_archive(str(output.with_suffix("")), "zip", root)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="Folder with the validated Simulink inputs and CSV")
    args = parser.parse_args()
    render(args.run_dir / "twin_case.json", args.run_dir / "twin_replay.csv",
           args.run_dir / "fox_twin_viewer.zip")
