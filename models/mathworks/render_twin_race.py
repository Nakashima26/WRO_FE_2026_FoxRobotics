"""Record an autonomous SIL race for the MATLAB replay viewer."""
import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import cv2
import numpy as np

from render_twin_replay import _field_payload, _paint
from pure_pursuit import config
from pure_pursuit.centerline import draw_bev_debug
from pure_pursuit.twin.sim import Sim


def write_video(frames: Path, output: Path, times: list[float]) -> None:
    """Encode synchronized panels at the simulator's recorded frame cadence."""
    period = (times[-1] - times[0]) / (len(times) - 1)
    if not np.allclose(np.diff(times), period, atol=1e-6, rtol=0):
        raise ValueError("The video export requires uniformly spaced camera frames.")
    with subprocess.Popen([
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
        "-pixel_format", "bgr24", "-video_size", "1120x720",
        "-framerate", f"{1 / period:.9f}", "-i", "-", "-c:v", "libx264",
        "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output),
    ], stdin=subprocess.PIPE) as encoder:
        assert encoder.stdin is not None
        for index in range(1, len(times) + 1):
            frame = np.full((720, 1120, 3), (27, 23, 20), dtype=np.uint8)
            for label, heading, x, y, width, height in (
                ("map", "Pista", 12, 12, 680, 696),
                ("camera", "Camara", 708, 12, 400, 338),
                ("bev", "BEV", 708, 370, 400, 338),
            ):
                image = cv2.imread(str(frames / f"{label}_{index:04d}.jpg"))
                if image is None:
                    raise OSError(f"Missing {label} frame {index}")
                scale = min(width / image.shape[1], (height - 26) / image.shape[0])
                resized = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                h, w = resized.shape[:2]
                left = x + (width - w) // 2
                top = y + 26 + (height - 26 - h) // 2
                frame[top:top+h, left:left+w] = resized
                cv2.putText(frame, heading, (x, y + 18), cv2.FONT_HERSHEY_SIMPLEX,
                            0.56, (240, 240, 240), 1, cv2.LINE_AA)
            encoder.stdin.write(frame.tobytes())
        encoder.stdin.close()
        if encoder.wait() != 0:
            raise RuntimeError("FFmpeg did not finish encoding the replay.")


def record(seed: int, output: Path, setup: dict) -> None:
    every = 1 if output.suffix.lower() == ".mp4" else 3
    output = output.resolve()
    with tempfile.TemporaryDirectory() as work:
        root = Path(work)
        frames = root / "frames"
        frames.mkdir()
        times, trail = [], []
        field = None

        def wants_images():
            return wants_images.frame % every == 0

        wants_images.frame = 0

        def record_frame(view):
            nonlocal field
            wants_images.frame += 1
            if (wants_images.frame - 1) % every != 0:
                return True
            result = view.frame_result
            if result.processed_frame is None or result.bev_frame is None:
                return True
            if field is None:
                field = _field_payload(view.field)
            car = view.vehicle
            trail.append((car.x, car.y))
            elapsed = view.t - (times[0][0] if times else view.t)
            hud = f"t={elapsed:.2f} s | {car.v:.0f} mm/s | esquinas {view.tc or 0}"
            strip = _paint(field, trail, {"xy": trail[-1], "h": car.heading_deg, "hud": hud}, None)
            dx, dy = result.bev_shift
            bev = draw_bev_debug(
                result.bev_frame, result.path_points, result.lookahead_pt,
                result.bev_obstacles, result.steer_deg, result.pp_active,
                line_info=result.line_info, bev_obstacles_beyond=result.bev_obstacles_beyond,
                robot_xy=(config.ROBOT_BEV_X + dx, config.ROBOT_BEV_Y + dy),
            )
            index = len(times) + 1
            for label, image in (("map", strip[:-22, :strip.shape[0]]),
                                 ("camera", result.processed_frame), ("bev", bev)):
                if not cv2.imwrite(str(frames / f"{label}_{index:04d}.jpg"), image,
                                   [cv2.IMWRITE_JPEG_QUALITY, 76]):
                    raise OSError(f"Could not write {label} frame {index}")
            times.append((view.t, elapsed))
            return True

        record_frame.wants_images = wants_images
        result = Sim(seed, preset="ruta_perfil", park_pos="random", max_time_s=120,
                     pi_overrides=setup.get("pi"), fw_overrides=setup.get("fw")).run(record_frame)
        if len(times) < 2:
            raise RuntimeError("The twin did not produce enough camera and BEV frames.")
        summary = {"seed": seed, "times_s": [elapsed for _, elapsed in times],
                   "title": "Carrera autónoma del twin", "stop_reason": result.stop_reason,
                   "metrics": result.metrics}
        (root / "replay_manifest.json").write_text(json.dumps(summary))
        shutil.copyfile(Path(__file__).with_name("view_twin_replay.m"), root / "view_twin_replay.m")
        output.parent.mkdir(parents=True, exist_ok=True)
        if every == 1:
            write_video(frames, output, summary["times_s"])
            output.with_suffix(".json").write_text(json.dumps(summary, indent=2))
        else:
            shutil.make_archive(str(output.with_suffix("")), "zip", root)
        print(f"{output}: {len(times)} frames, {times[-1][1]:.2f} s, {result.stop_reason}")
        print(result.metrics)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Viewer ZIP or full-frame MP4")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--setup", type=Path, help="Optional twin benchmark JSON")
    args = parser.parse_args()
    record(args.seed, args.output, json.loads(args.setup.read_text()) if args.setup else {})
