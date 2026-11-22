"""Paired SIL obstacle-round comparison of a stiff pack and Simscape-based sag."""

import argparse
import csv
import json
from pathlib import Path

from ..battery import BatteryDrive
from ..sim import Sim


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('seed', type=int)
    parser.add_argument('output', type=Path)
    parser.add_argument('--setup', type=Path, help='Optional SIL benchmark JSON')
    args = parser.parse_args()
    setup = json.loads(args.setup.read_text()) if args.setup else {}

    args.output.mkdir(parents=True, exist_ok=True)
    for name, resistance in (('stiff', 0.0), ('sag', 0.12)):
        sim = Sim(args.seed, preset='ruta_perfil', park_pos='random',
                  max_time_s=120, pi_overrides=setup.get('pi'),
                  fw_overrides=setup.get('fw'),
                  battery=BatteryDrive(pack_resistance_ohm=resistance))
        result = sim.run()
        (args.output / f'{name}_metrics.json').write_text(json.dumps(result.metrics, indent=2))
        columns = ('t', 'x', 'y', 'heading', 'motor_dir', 'motor_pwm', 'speed_mm_s',
                   'pack_voltage_v', 'pack_current_a')
        with (args.output / f'{name}_trace.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, columns, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(result.trace)
        print(name, json.dumps(result.metrics))


if __name__ == '__main__':
    main()
