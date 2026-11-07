#!/usr/bin/env python3
"""Ejecutar detcheck para múltiples configuraciones."""

import argparse
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


CONFIGS = [
    ('s14', 14, 100.0),
    ('s2', 2, 100.0),
    ('s6', 6, 100.0),
    ('s3170839', 3170839, 100.0),
    ('s9_30', 9, 30.0),
]


def run_config(name, seed, max_time, args):
    """Ejecutar detcheck para una configuración."""
    detcheck_script = Path(__file__).with_name('detcheck.py')

    cmd = [
        sys.executable,
        str(detcheck_script),
        '--seed', str(seed),
        '--max-time', str(max_time),
        '--save' if args.save else '--compare',
        str(Path(args.gold) / f'{name}.json')
    ]

    # Lanzar por el semáforo global si existe (limita la concurrencia real
    # entre todas las sesiones); si no existe (Mac/Pi/otro host), cmd directo.
    slot = os.environ.get('FOX_SLOT', 'C:/Users/jbanda/fox_local/slot.py')
    if Path(slot).is_file():
        cmd = [sys.executable, slot, '--tag', f't17_{args.tag}_{name}', '--'] + cmd

    # Crear directorio de salida
    out_dir = Path('runs/t17/cmp') / args.tag
    out_dir.mkdir(parents=True, exist_ok=True)

    out_file = out_dir / f'{name}.out'
    err_file = out_dir / f'{name}.err'

    with open(out_file, 'w') as fo, open(err_file, 'w') as fe:
        result = subprocess.run(
            cmd,
            stdin=subprocess.DEVNULL,
            stdout=fo,
            stderr=fe,
            env=dict(os.environ)
        )

    # Leer stdout para extraer resultado y wall time
    with open(out_file) as f:
        out_text = f.read()

    rc = result.returncode
    ok = rc == 0 and (args.save or 'IDENTICO' in out_text)

    # Extraer wall time
    wall_match = re.search(r'Wall time: ([0-9.]+)', out_text)
    wall = wall_match.group(1) if wall_match else 'N/A'

    status = 'GUARDADO' if args.save else ('IDENTICO' if ok else f'DIFF rc={rc}')

    return name, status, wall, ok


def main():
    parser = argparse.ArgumentParser(description='Ejecutar detcheck para múltiples configuraciones')
    parser.add_argument('--gold', default='runs/t17/gold', help='Directorio con golden files')
    parser.add_argument('--tag', required=True, help='Tag para identificar las corridas')
    parser.add_argument('--save', action='store_true', help='Guardar golden files')
    parser.add_argument('--jobs', type=int, default=4, help='Número de workers (máximo 4)')
    parser.add_argument('--only', nargs='*', help='Filtrar por nombre de configuración')

    args = parser.parse_args()

    # Recortar jobs a máximo 4
    args.jobs = min(4, args.jobs)

    # Filtrar configs si se especifica --only
    configs = CONFIGS
    if args.only:
        known = {n for n, _, _ in CONFIGS}
        unknown = sorted(set(args.only) - known)
        if unknown:
            print(f"error: --only desconocido: {' '.join(unknown)} "
                  f"(validos: {' '.join(n for n, _, _ in CONFIGS)})", file=sys.stderr)
            sys.exit(2)
        configs = [(n, s, t) for n, s, t in CONFIGS if n in args.only]

    results = []

    with ThreadPoolExecutor(max_workers=args.jobs) as executor:
        futures = {
            executor.submit(run_config, name, seed, max_time, args): (name, seed, max_time)
            for name, seed, max_time in configs
        }

        # Recopilar resultados en el orden de CONFIGS
        name_to_result = {}
        for future in as_completed(futures):
            name, status, wall, ok = future.result()
            name_to_result[name] = (status, wall, ok)

        # Imprimir en el orden de CONFIGS
        n_ok = 0
        for name, seed, max_time in configs:
            if name in name_to_result:
                status, wall, ok = name_to_result[name]
                print(f"{name}: {status} wall={wall}")
                if ok:
                    n_ok += 1

    # Resumen final
    n_total = len(configs)
    word = 'GUARDADO' if args.save else 'IDENTICO'
    fox_native = os.environ.get('FOX_NATIVE', '1')
    print(f"resumen: {n_ok}/{n_total} {word} FOX_NATIVE={fox_native}")

    sys.exit(0 if n_ok == n_total else 1)


if __name__ == '__main__':
    main()
