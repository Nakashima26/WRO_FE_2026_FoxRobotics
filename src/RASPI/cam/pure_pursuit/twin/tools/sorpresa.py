"""Banco de regla sorpresa; el detector solo recibe imágenes, nunca el Field.

Desde src/RASPI/cam:
  python -m pure_pursuit.twin.tools.sorpresa --seed 7 --height 50 --out /tmp/sorpresa-7
  python -m pure_pursuit.twin.tools.sorpresa --seed 7 --height 150 --out /tmp/sorpresa-7-alta
"""
import argparse
import json
from pathlib import Path
import random

from pure_pursuit.twin.sim import Sim
from pure_pursuit.wro_field import randomize, surprise_candidates


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--height', type=int, choices=(50, 150), default=50)
    parser.add_argument('--index', type=int, help='índice de señal; se rechaza la primera del recorrido')
    parser.add_argument('--normal', action='store_true', help='referencia sin modificación ni intérprete')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    field = randomize(args.seed, start='cajon', park_pos='random')
    index = args.index if args.index is not None else random.Random(args.seed).choice(surprise_candidates(field))
    cfg = json.loads(Path(__file__).with_name('carrera_bench.json').read_text())
    pi = dict(cfg['pi'], SURPRISE_RULE=not args.normal, ROUTE_SEAT_ROI=False, DIGITAL_MAP_SEED_CONFIRMED=False)
    seen = {}
    def observe(view):
        for item in view.frame_result.surprise_observations or ():
            if item['confirmed_mm'] in (50,150):
                key = str((item['seat'], item['visible'], item['driving'], item['confirmed_mm']))
                seen[key] = seen.get(key, 0)+1
        return True
    observe.wants_images = lambda: False
    args.out.mkdir(parents=True, exist_ok=True)
    sim = Sim(args.seed, preset='ruta_perfil', field=field, pi_overrides=pi,
              fw_overrides=cfg.get('fw'), fw_defines=cfg.get('def'),
              log_dir=args.out, ignore_collisions=('pared exterior',),
              surprise=None if args.normal else (index,args.height))
    result = sim.run(frame_callback=observe)
    special = result.field.signs[index]
    report = dict(seed=args.seed, index=index, section=special.section, seat=special.seat,
                  visible=special.color, original=special.driving_color, height_mm=special.height_mm,
                  detected=seen, stop_reason=result.stop_reason, metrics=result.metrics)
    (args.out/'surprise.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
    print(json.dumps(report,ensure_ascii=False))


if __name__ == '__main__':
    main()
