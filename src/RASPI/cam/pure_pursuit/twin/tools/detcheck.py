#!/usr/bin/env python3
"""Detección de cambios bit-idénticos en runs de simulación."""

import argparse
import json
import sys
import time
from pathlib import Path
from hashlib import sha256

# Agregar el package root para importar pure_pursuit
cam_dir = Path(__file__).resolve().parent.parent.parent.parent
sys.path.insert(0, str(cam_dir))

from pure_pursuit.twin.sim import Sim


_TIMING_KEYS = ("pi_frame_ms", "wall_time_s", "wall_per_sim_s", "mean_pi_frame_ms", "render_s", "pi_s", "other_s")


def _strip_timing(x):
    # pi_frame_ms y similares son perf_counter (reloj real): cambian en cada corrida.
    if isinstance(x, dict):
        return {k: _strip_timing(v) for k, v in x.items() if k not in _TIMING_KEYS}
    if isinstance(x, (list, tuple)):
        return [_strip_timing(v) for v in x]
    return x


def compute_fingerprint(sim_result):
    """Generar SHA256 de la traza sin campos de reloj real."""
    trace_data = json.dumps(_strip_timing(sim_result.trace), default=str, sort_keys=True)
    return sha256(trace_data.encode()).hexdigest()


def extract_metrics(sim_result):
    """Extraer métricas sin wall_time_s."""
    return _strip_timing(dict(sim_result.metrics))


def _norm(m):
    return json.loads(json.dumps(_strip_timing(m), default=str, sort_keys=True))


def save_golden(output_path, seed, preset, max_time_s):
    """Ejecutar sim y guardar línea base."""
    print(f"Ejecutando Sim(seed={seed}, preset={preset}, max_time_s={max_time_s})...", file=sys.stderr)
    t_start = time.perf_counter()
    sim = Sim(seed=seed, preset=preset, max_time_s=max_time_s)
    result = sim.run()
    wall_time = time.perf_counter() - t_start

    metrics = extract_metrics(result)
    fingerprint = compute_fingerprint(result)

    golden = {
        'seed': seed,
        'preset': preset,
        'max_time_s': max_time_s,
        'metrics': metrics,
        'fingerprint': fingerprint,
        'wall_time_s': wall_time,
    }

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(golden, f, indent=2, default=str)

    print(f"Guardado: {output_path} ({wall_time:.3f}s)", file=sys.stderr)
    return golden, wall_time


def compare_with_golden(golden_path, seed, preset, max_time_s):
    """Ejecutar sim y comparar con dorada."""
    golden_data = json.load(open(golden_path))

    # Validar parámetros contra el golden (ignorar los que sean None)
    for field, val in [('seed', seed), ('preset', preset), ('max_time_s', max_time_s)]:
        if val is not None:
            golden_val = golden_data.get(field)
            if golden_val is not None and golden_val != val:
                print(f"PARAMS DISTINTOS: {field} golden={golden_val} arg={val}")
                return False, 0.0

    print(f"Ejecutando Sim para comparar (seed={seed}, preset={preset})...", file=sys.stderr)
    t_start = time.perf_counter()
    sim = Sim(seed=seed, preset=preset, max_time_s=max_time_s)
    result = sim.run()
    wall_time = time.perf_counter() - t_start

    metrics = extract_metrics(result)
    fingerprint = compute_fingerprint(result)

    golden_fp = golden_data['fingerprint']
    golden_metrics = golden_data['metrics']

    cur = _norm(metrics)
    gold = _norm(golden_metrics)

    if fingerprint == golden_fp and cur == gold:
        print("IDENTICO")
    else:
        # Reportar diferencias
        if fingerprint != golden_fp:
            print(f"Fingerprint diferente:", file=sys.stderr)
            print(f"  Esperado: {golden_fp}", file=sys.stderr)
            print(f"  Actual:   {fingerprint}", file=sys.stderr)

        diff_keys = set(gold.keys()) | set(cur.keys())
        for k in sorted(diff_keys):
            if gold.get(k) != cur.get(k):
                print(f"  {k}: {gold.get(k)} -> {cur.get(k)}", file=sys.stderr)

    # Imprimir timing (solo informativo)
    render_s = result.metrics.get('render_s')
    pi_s = result.metrics.get('pi_s')
    other_s = result.metrics.get('other_s')
    print(f"timing: render_s={render_s} pi_s={pi_s} other_s={other_s}")

    return fingerprint == golden_fp and cur == gold, wall_time


def main():
    parser = argparse.ArgumentParser(description='Detcheck: verificar resultados idénticos')
    parser.add_argument('--seed', type=int, default=3170839, help='Seed aleatorio')
    parser.add_argument('--preset', default='hw_nuevo', help='Preset de simulación')
    parser.add_argument('--max-time', type=float, default=20, help='Tiempo máximo de sim')
    parser.add_argument('--save', help='Guardar línea base a archivo JSON')
    parser.add_argument('--compare', help='Comparar con línea base guardada')

    args = parser.parse_args()

    if args.save:
        _, wall_time = save_golden(args.save, args.seed, args.preset, args.max_time)
        print(f"Wall time: {wall_time:.3f}s")
        sys.exit(0)

    if args.compare:
        ok, wall_time = compare_with_golden(args.compare, args.seed, args.preset, args.max_time)
        print(f"Wall time: {wall_time:.3f}s")
        sys.exit(0 if ok else 1)

    parser.print_help()
    sys.exit(1)


if __name__ == '__main__':
    main()
