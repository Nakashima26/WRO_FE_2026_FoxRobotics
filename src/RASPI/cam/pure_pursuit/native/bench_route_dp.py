"""Microbench pareado de RR.plan con cachés calientes sobre los cuatro golden.

Desde src/RASPI/cam:
  PYTHONPATH=. python -m pure_pursuit.native.bench_route_dp --baseline /ruta/libbase.dylib
La biblioteca base debe compilarse antes de modificar route_dp.c.
"""
import argparse
import ctypes
import os
import statistics
import time

from pure_pursuit import ring_route as RR
from pure_pursuit.twin.tests.test_route_dp import _cases, _run, _same_route


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', required=True)
    parser.add_argument('--repeats', type=int, default=100)
    parser.add_argument('--batches', type=int, default=9)
    parser.add_argument('--threads', type=int, nargs='+', choices=(1, 2, 3, 4), default=[1, 2, 4])
    args = parser.parse_args()
    lib = ctypes.CDLL(args.baseline)
    current = RR._dp_sparse_t_kernel()
    if current is None:
        parser.error('kernel transpuesto no disponible')
    base = lib.fox_route_dp_sparse_t
    base.argtypes, base.restype = current.argtypes, current.restype
    kernels = (base,) + (current,) * len(args.threads)
    counts = [1] + args.threads
    saved_threads = os.environ.get("FOX_DP_THREADS")
    key = ('fox_route_dp_sparse_t', True)
    cases = _cases()
    samples = [[[] for _ in cases] for _ in kernels]
    cpu = [[[] for _ in cases] for _ in kernels]
    try:
        for i, case in enumerate(cases):
            routes = []
            for j, kernel in enumerate(kernels):
                os.environ["FOX_DP_THREADS"] = str(counts[j])
                RR._KERNELS[key] = kernel
                for _ in range(8):
                    route = _run(case)
                routes.append(route)
            for j, route in enumerate(routes[1:], 1):
                _same_route(routes[0], route, f'caso {i}, hilos {counts[j]}')
        for batch in range(args.batches):
            # Alterna el orden para repartir calentamiento y carga externa.
            order = list(range(len(kernels)))
            order = order[batch % len(order):] + order[:batch % len(order)]
            for j in order:
                os.environ["FOX_DP_THREADS"] = str(counts[j])
                RR._KERNELS[key] = kernels[j]
                for i, case in enumerate(cases):
                    wall0, cpu0 = time.perf_counter_ns(), time.process_time_ns()
                    for _ in range(args.repeats):
                        _run(case)
                    cpu[j][i].append((time.process_time_ns() - cpu0) / args.repeats / 1e6)
                    samples[j][i].append((time.perf_counter_ns() - wall0) / args.repeats / 1e6)
        for j, name in enumerate(['base'] + [f'nuevo/{t} hilos' for t in args.threads]):
            wall = [statistics.median(s) for s in samples[j]]
            proc = [statistics.median(s) for s in cpu[j]]
            print(f'{name}: ms/caso={wall}; media={statistics.mean(wall):.6f}; '
                  f'CPU ms/caso={proc}; media={statistics.mean(proc):.6f}')
    finally:
        if saved_threads is None:
            os.environ.pop("FOX_DP_THREADS", None)
        else:
            os.environ["FOX_DP_THREADS"] = saved_threads
        RR._KERNELS[key] = current


if __name__ == '__main__':
    main()
