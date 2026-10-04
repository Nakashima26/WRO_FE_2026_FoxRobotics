#!/usr/bin/env python3
"""Detección de cambios bit-idénticos en runs de simulación."""

import argparse
import json
import sys
import time
from pathlib import Path
from hashlib import sha256

# Agregar el package root para importar pure_pursuit
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from twin.sim import Sim


def compute_fingerprint(sim_result):
    """Generar SHA256 de la traza (determinístico)."""
    trace_data = json.dumps(sim_result.trace, default=str, sort_keys=True)
    return sha256(trace_data.encode()).hexdigest()


def extract_metrics(sim_result):
    """Extraer métricas sin wall_time_s."""
    metrics = dict(sim_result.metrics)
    # Remover métricas dependientes del timing
    for key in ['wall_time_s', 'wall_per_sim_s', 'mean_pi_frame_ms']:
        metrics.pop(key, None)
    return metrics


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

    print(f"Ejecutando Sim para comparar (seed={seed}, preset={preset})...", file=sys.stderr)
    t_start = time.perf_counter()
    sim = Sim(seed=seed, preset=preset, max_time_s=max_time_s)
    result = sim.run()
    wall_time = time.perf_counter() - t_start

    metrics = extract_metrics(result)
    fingerprint = compute_fingerprint(result)

    golden_fp = golden_data['fingerprint']
    golden_metrics = golden_data['metrics']

    if fingerprint == golden_fp and metrics == golden_metrics:
        print("IDENTICO")
        return True, wall_time
    else:
        # Reportar diferencias
        if fingerprint != golden_fp:
            print(f"Fingerprint diferente:", file=sys.stderr)
            print(f"  Esperado: {golden_fp}", file=sys.stderr)
            print(f"  Actual:   {fingerprint}", file=sys.stderr)

        diff_keys = set(golden_metrics.keys()) | set(metrics.keys())
        for k in sorted(diff_keys):
            if golden_metrics.get(k) != metrics.get(k):
                print(f"  {k}: {golden_metrics.get(k)} -> {metrics.get(k)}", file=sys.stderr)

        return False, wall_time


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
