"""Benchmark de etapas del twin: render, detect_on, collision, sensors, widest, firmware."""

import argparse
import json
import os
import platform
import sys
from time import perf_counter

import cv2
import numpy as np

from pure_pursuit.twin.camera import CameraModel
from pure_pursuit.twin.params import TwinParams
from pure_pursuit.twin.world import World, collision
from pure_pursuit.twin.sensors import Ultrasonic, ToF, Pose
from pure_pursuit.wro_field import randomize
from pure_pursuit.centerline import _widest_free_segment_bounds
from vision import Vision


def bench_render(repeats: int) -> float:
    """Benchmark render_camera de 60 poses sobre campo aleatorio."""
    tp = TwinParams()
    cam = CameraModel(tp.camera, tp)
    field = randomize(14)

    rng = np.random.default_rng(0)
    poses = [
        (float(rng.choice([-1, 1]) * rng.uniform(600, 1400)),
         float(rng.uniform(-1400, 1400)),
         float(rng.choice([0.0, 90.0, 180.0, 270.0]) + rng.normal(0, 20)))
        for _ in range(60)
    ]

    # Un render de calentamiento
    cam.render_camera(field, *poses[0])

    times = []
    for _ in range(repeats):
        t0 = perf_counter()
        for pose in poses:
            cam.render_camera(field, *pose)
        t1 = perf_counter()
        times.append((t1 - t0) / len(poses) * 1000)  # ms por llamada

    return min(times)


def bench_detect_on(repeats: int) -> float:
    """Benchmark Vision.detect_on() sobre 60 frames renderizados."""
    tp = TwinParams()
    cam = CameraModel(tp.camera, tp)
    field = randomize(14)

    rng = np.random.default_rng(0)
    poses = [
        (float(rng.choice([-1, 1]) * rng.uniform(600, 1400)),
         float(rng.uniform(-1400, 1400)),
         float(rng.choice([0.0, 90.0, 180.0, 270.0]) + rng.normal(0, 20)))
        for _ in range(60)
    ]

    # Renderizar los 60 frames
    frames = [cam.render_camera(field, *pose) for pose in poses]

    v = Vision(open_cam=False)

    times = []
    for _ in range(repeats):
        t0 = perf_counter()
        for frame in frames:
            v.detect_on(frame.copy())
        t1 = perf_counter()
        times.append((t1 - t0) / len(frames) * 1000)  # ms por llamada

    return min(times)


def bench_collision(repeats: int) -> float:
    """Benchmark collision() con 2000 poses aleatorias (ignore=() y ('pared exterior',))."""
    w = World(randomize(14))
    tp = TwinParams()

    rng = np.random.default_rng(1)
    poses = [
        (float(rng.uniform(-1450, 1450)),
         float(rng.uniform(-1450, 1450)),
         float(rng.uniform(0, 360)))
        for _ in range(2000)
    ]

    times = []
    ignores = [(), ("pared exterior",)]

    for _ in range(repeats):
        t0 = perf_counter()
        for x, y, heading in poses:
            for ignore in ignores:
                collision(w, x, y, heading,
                         tp.vehicle.length_mm,
                         tp.vehicle.width_mm,
                         tp.vehicle.rear_overhang_mm,
                         ignore=ignore)
        t1 = perf_counter()
        n_calls = len(poses) * len(ignores)
        times.append((t1 - t0) / n_calls * 1000)  # ms por llamada

    return min(times)


def bench_sensors(repeats: int) -> dict[str, float]:
    """Benchmark Ultrasonic.echo_us() y ToF.read_mm() con 200 poses.

    read_mm retiene la lectura si `t` no avanzó al menos un sample_period
    desde la última muestra de ese sensor_id (sensors.py:112-113). Pasar
    siempre t=0.0 hace que solo la primera llamada por sensor_id haga un
    cast real y el resto sean cache hits casi gratis, con lo que la etapa
    "sensors" terminaría midiendo casi solo echo_us. Por eso t avanza un
    sample_period por pose (k * sample_period_s) y echo_us/read_mm se
    miden en bucles separados, reportando además 'sensors_echo' y
    'sensors_tof' junto al agregado 'sensors'. Cada repetición usa un ToF
    (con su caché _last_sample_t/_held_mm) y un rng nuevos para que las
    repeticiones sean idénticas entre sí.
    """
    w = World(randomize(14))
    tp = TwinParams()

    sonar_ids = list(range(len(tp.ultrasonic.mounts)))
    tof_ids = list(range(len(tp.tof.mounts)))

    echo_times = []
    tof_times = []

    for _ in range(repeats):
        rng = np.random.default_rng(3)
        poses = [
            Pose(float(rng.uniform(-1450, 1450)),
                 float(rng.uniform(-1450, 1450)),
                 float(rng.uniform(0, 360)))
            for _ in range(200)
        ]

        sonar = Ultrasonic(tp.ultrasonic, rng)
        tof = ToF(tp.tof, rng)

        t0 = perf_counter()
        for pose in poses:
            for sid in sonar_ids:
                sonar.echo_us(sid, w, pose)
        t1 = perf_counter()
        echo_times.append((t1 - t0) / (len(poses) * len(sonar_ids)) * 1000)  # ms por llamada

        t0 = perf_counter()
        for k, pose in enumerate(poses):
            t = k * tp.tof.sample_period_s
            for idx in tof_ids:
                tof.read_mm(idx, w, pose, t)
        t1 = perf_counter()
        tof_times.append((t1 - t0) / (len(poses) * len(tof_ids)) * 1000)  # ms por llamada

    n_echo = len(sonar_ids)
    n_tof = len(tof_ids)
    sensors_echo = min(echo_times)
    sensors_tof = min(tof_times)
    # Agregado ponderado por número de llamadas, para que "sensors" siga
    # siendo comparable con el ms/llamada que medía la versión anterior.
    sensors_agg = (sensors_echo * n_echo + sensors_tof * n_tof) / (n_echo + n_tof)

    return {
        "sensors": sensors_agg,
        "sensors_echo": sensors_echo,
        "sensors_tof": sensors_tof,
    }


def bench_widest(repeats: int) -> float:
    """Benchmark _widest_free_segment_bounds con 2000 máscaras."""
    rng = np.random.default_rng(5)

    # Generar 2000 máscaras siguiendo literalmente el coder_spec:
    # r = (rng.random(400) < 0.8).astype(np.uint8)*255 -> ~80% libre (255), ~20% obstáculo (0),
    # fragmentado pixel a pixel (free_cols = row_mask > 0 en centerline.py).
    # 30% de filas tienen además un tramo libre central contiguo de ancho aleatorio.
    masks = []
    for _ in range(2000):
        row = (rng.random(400) < 0.8).astype(np.uint8) * 255

        if rng.random() < 0.3:
            # Para esta fila, crear un tramo libre central de ancho aleatorio
            width = int(rng.uniform(1, 100))
            start = int(rng.uniform(0, max(1, 400 - width)))
            row[start:start + width] = 255

        masks.append(row)

    min_widths = [1, 8, 40]
    times = []

    for _ in range(repeats):
        t0 = perf_counter()
        for mask in masks:
            for min_w in min_widths:
                _widest_free_segment_bounds(mask, min_w)
        t1 = perf_counter()
        n_calls = len(masks) * len(min_widths)
        times.append((t1 - t0) / n_calls * 1000)  # ms por llamada

    return min(times)


def bench_firmware(repeats: int) -> dict[str, float]:
    """Benchmark firmware SIL: servo_angle+motor_pwm+motor_dir y, si existe, actuators().

    Se miden en dos bucles separados ('firmware_getters' y
    'firmware_actuators') en vez de sumar ambos en un solo número. Si se
    midieran juntos, el candidato que añade actuators() pasaría de 3 a 4
    llamadas por iteración y la etapa combinada subiría aunque
    actuators() sea más rápido que los 3 getters que reemplaza, mostrando
    una regresión falsa en --compare justo para esa optimización.
    """
    from pure_pursuit.twin.firmware import build
    from pure_pursuit.twin.firmware.fw import FirmwareSIL

    # Build firmware una sola vez
    _ = build.build(force=False)

    getters_times = []
    actuators_times = []
    has_actuators = False

    for _ in range(repeats):
        fw = FirmwareSIL()
        fw.set_callbacks(
            advance=lambda _us: None,
            sonar=lambda _sid: 2915,  # 50 cm simulado
            gyro=lambda: 0.0
        )
        fw.push_serial2("READY")
        fw.setup()

        t0 = perf_counter()
        for _ in range(100_000):
            _ = fw.servo_angle()
            _ = fw.motor_pwm()
            _ = fw.motor_dir()
        t1 = perf_counter()
        getters_times.append((t1 - t0) / 100_000 * 1000)  # ms por llamada

        if hasattr(fw, 'actuators'):
            has_actuators = True
            t0 = perf_counter()
            for _ in range(100_000):
                _ = fw.actuators()
            t1 = perf_counter()
            actuators_times.append((t1 - t0) / 100_000 * 1000)  # ms por llamada

        fw.close()

    result = {"firmware_getters": min(getters_times)}
    if has_actuators:
        result["firmware_actuators"] = min(actuators_times)
    return result


def main():
    parser = argparse.ArgumentParser(description="Benchmark de etapas del twin")
    parser.add_argument("--out", type=str, help="Archivo JSON de salida")
    parser.add_argument("--compare", nargs=2, help="Comparar dos JSON (A B)")
    parser.add_argument("--repeats", type=int, default=5, help="Repeticiones por etapa")
    parser.add_argument("--fw", action="store_true", help="Incluir benchmark del firmware")

    args = parser.parse_args()

    # --compare sin --out solo lee JSON ya existentes: no hay que volver a
    # medir (bench_stages.py --compare A B debe ser instantáneo, no una
    # corrida pesada de ~decenas de segundos).
    if args.compare and not args.out:
        _print_compare(args.compare)
        return

    stages = {}

    print("Benchmarking stages...")

    print("  render...", end="", flush=True)
    stages["render"] = bench_render(args.repeats)
    print(f" {stages['render']:.3f} ms")

    print("  detect_on...", end="", flush=True)
    stages["detect_on"] = bench_detect_on(args.repeats)
    print(f" {stages['detect_on']:.3f} ms")

    print("  collision...", end="", flush=True)
    stages["collision"] = bench_collision(args.repeats)
    print(f" {stages['collision']:.3f} ms")

    print("  sensors...", end="", flush=True)
    sensors_stages = bench_sensors(args.repeats)
    stages.update(sensors_stages)
    print(f" {stages['sensors']:.3f} ms"
          f" (echo={sensors_stages['sensors_echo']:.3f} tof={sensors_stages['sensors_tof']:.3f})")

    print("  widest...", end="", flush=True)
    stages["widest"] = bench_widest(args.repeats)
    print(f" {stages['widest']:.3f} ms")

    if args.fw:
        print("  firmware...", end="", flush=True)
        fw_stages = bench_firmware(args.repeats)
        stages.update(fw_stages)
        parts = " ".join(f"{k.removeprefix('firmware_')}={v:.3f}" for k, v in fw_stages.items())
        print(f" {parts} ms")

    result = {
        "host": platform.platform(),
        "machine": platform.machine(),
        "python": sys.version.split()[0],
        "numpy": np.__version__,
        "cv2": cv2.__version__,
        "FOX_NATIVE": os.environ.get("FOX_NATIVE", "1"),
        "stages": stages,
    }

    if args.out:
        with open(args.out, "w") as f:
            json.dump(result, f, indent=2)
        print(f"\nEscrito: {args.out}")

    if args.compare:
        _print_compare(args.compare)


def _print_compare(paths: list[str]) -> None:
    path_a, path_b = paths
    with open(path_a) as f:
        data_a = json.load(f)
    with open(path_b) as f:
        data_b = json.load(f)

    print("\nComparación:")
    print("Stage            A_ms       B_ms       ratio")
    for stage in sorted(data_a.get("stages", {}).keys()):
        a = data_a["stages"][stage]
        b = data_b["stages"].get(stage)
        if b is not None:
            ratio = b / a
            print(f"{stage:16} {a:10.3f} {b:10.3f} {ratio:10.3f}")


if __name__ == "__main__":
    main()
