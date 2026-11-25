"""Sonda del banco de latencia. Ejecutar desde src/RASPI/cam."""
import json
import os
import runpy
import sys
from collections import defaultdict

def run(seed, out):
    import inspect
    import textwrap
    from pure_pursuit import route_mode as rm

    def instrument(name, edits):
        source = textwrap.dedent(inspect.getsource(getattr(rm.RouteMode, name)))
        for old, new in edits:
            assert source.count(old) == 1, old
            source = source.replace(old, new)
        exec(compile(source, inspect.getfile(rm), "exec"), rm.__dict__)
        setattr(rm.RouteMode, name, rm.__dict__.pop(name))

    def no_corner(self, signs, tc, conf, parking):
        corner = rm.C.ROUTE_L1_CORNER_SIGN
        rm.C.ROUTE_L1_CORNER_SIGN = None
        try:
            return self._profile_pwm(signs, l1=tc is not None and 1 <= tc < 4,
                                     conf=conf, parking=parking)
        finally:
            rm.C.ROUTE_L1_CORNER_SIGN = corner

    rm._l1_no_corner = no_corner
    rm._l1_samples = []
    instrument("_speed", [
        ('    if not unknown:', '    self._diag_unseen = ("abierto", None, None)\n    if not unknown:'),
        ('    def wcap(v):', '    kind = "abierto"\n    def wcap(v):'),
        ('        return int(wall_pwm) if v is None else min(v, int(wall_pwm))',
         '        self._diag_unseen = (kind, v, int(wall_pwm))\n        return int(wall_pwm) if v is None else min(v, int(wall_pwm))'),
        ('            return v\n', '            self._diag_unseen = (kind, v, None)\n            return v\n'),
        ('            return wcap(unseen)', '            kind = "asiento"\n            return wcap(unseen)'),
        ('        return wcap(int(round', '        kind = "rampa"\n        return wcap(int(round'),
    ])
    instrument("step", [
        ('    if self.fixed:\n        # Los asientos',
         '    self._diag_unseen = ("abierto", None, None)\n    if self.fixed:\n        # Los asientos'),
        ('    if abs(self.follower.cte) >', '    pre = self.rv\n    if abs(self.follower.cte) >'),
        ('    prof = self._profile_pwm(', '    after = self.rv\n    prof = self._profile_pwm('),
        ('    self.dbg = (round(x)', '''    if (tcf or 0) < 4:
        no_corner = (_l1_no_corner(self, self._sign_xy(conf, empty), tc, conf, dm.parking)
                     if self.fixed else prof)
        _l1_sink(tcf or 0, _ack_num(ack or "", "od"), pre, after, prof, no_corner,
                 self.rv, self._diag_unseen)
    self.dbg = (round(x)'''),
    ])

    def sink(*vals):
        import inspect
        frame = inspect.currentframe()
        while frame is not None and "t_k" not in frame.f_locals:
            frame = frame.f_back
        if frame is not None:
            rm._l1_samples.append([frame.f_locals["t_k"], *vals])
    rm._l1_sink = sink
    path = "pure_pursuit/twin/tools/latencia/costo.py"
    sys.argv = [path, seed, out]
    try:
        runpy.run_path(path, run_name="__main__")
    finally:
        with open(out + ".caps.json", "w") as fh:
            json.dump(rm._l1_samples, fh)


def summary(base_dir):
    from pathlib import Path
    stats = defaultdict(lambda: [0.0, 0.0])
    files = list(Path(base_dir).glob("n*/s*.caps.json"))
    count = 0
    ceiling = 458
    for path in files:
        rows = json.loads(path.read_text())
        if len(rows) < 2:
            continue
        count += 1
        for a, b in zip(rows, rows[1:]):
            t, tc, od, pre, after, profile, plain, final, (kind, unseen, wall) = a
            dt = b[0] - t
            if dt <= 0 or dt > 0.5:
                continue
            cap = final if final is not None else ceiling
            limiting = {"perfil restante": plain if plain is not None else ceiling}
            for label, value in (
                (kind, unseen if kind != "abierto" else None), ("pared sin ver", wall),
                ("esquina", profile if plain is not None and profile is not None and profile < plain else None),
                ("prefijo", pre if pre is not None and unseen is None and wall is None else None),
                ("CTE/subida/espera", after if after is not None and (pre is None or after < pre) else None),
            ):
                if value is not None:
                    limiting[label] = value
            winners = [k for k, v in limiting.items() if v == cap]
            cause = " + ".join(winners) if winners else "otro"
            if cap >= ceiling:
                cause = "sin tope Pi"
            if len(winners) == 1 and cause not in ("otro", "sin tope Pi"):
                next_cap = min((v for k, v in limiting.items() if k != cause), default=ceiling)
                # Tiempo teórico para la misma distancia con velocidad proporcional a PWM - PWM0.
                loss = dt * max(0, 1 - max(cap - 18.7, 0) / max(next_cap - 18.7, 1))
            else:
                loss = 0.
            entry = stats[int(tc), cause]
            entry[0] += dt
            entry[1] += loss
    print(f"Carreras con muestras: {count}. Media por carrera en segundos.")
    for (tc, label), (duration, loss) in sorted(stats.items()):
        print(f"tc{tc} | {label} | {duration/count:.5f} | {loss/count:.5f}")


if __name__ == "__main__":
    if sys.argv[1] == "summary":
        summary(sys.argv[2])
    else:
        sys.path.insert(0, os.getcwd())
        run(*sys.argv[1:])
