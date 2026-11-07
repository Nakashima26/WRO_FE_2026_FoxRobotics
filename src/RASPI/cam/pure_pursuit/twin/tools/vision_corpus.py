#!/usr/bin/env python3
"""Vision corpus recorder: captura muestras de funciones de visión durante sim."""

import argparse
import json
import sys
import platform
from pathlib import Path
from hashlib import sha256
import numpy as np
import cv2

# Agregar el package root para importar pure_pursuit
cam_dir = Path(__file__).resolve().parent.parent.parent.parent
sys.path.insert(0, str(cam_dir))

import vision
import pure_pursuit.centerline
import pure_pursuit.corner_lines
import pure_pursuit.runtime_nuevo
from pure_pursuit import config as C
from pure_pursuit.twin.sim import Sim, _MAP_TRUTH_KEYS


class VisionCorpusRecorder:
    """Envuelve funciones de visión para grabar muestras durante una sim."""

    # Variable de clase para acceso desde closures
    _instance = None

    def __init__(self, every: int, out_dir: Path, config_keys: set[str] | None = None):
        self.every = every
        self.out_dir = Path(out_dir)
        self.counters = {
            'detect_on': 0,
            'detect_centerline': 0,
            'detect_lines': 0,
        }
        self.samples = {}

        # Claves de pure_pursuit.config que la sim sobreescribe (pi_overrides
        # del preset + verdad del mapa inyectada) y que, por lo tanto, deben
        # quedar iguales en el replay de `fingerprint` para que éste sea fiel
        # a lo que vio detect_centerline/detect_lines durante el record (T17
        # auditoría infra-vision-corpus: con los defaults de config.py en vez
        # de hw_nuevo, 0/171 muestras de detect_centerline reproducían).
        self.config_keys = set(config_keys or ())
        self.config_snapshot: dict | None = None

        # Wrappers instalados (se restauran al finalizar)
        self._original_detect_on = None
        self._original_detect_centerline = None
        self._original_detect_lines = None

        VisionCorpusRecorder._instance = self

    def install_wrappers(self):
        """Instala los wrappers sobre las funciones originales."""
        self._original_detect_on = vision.Vision.detect_on
        self._original_detect_centerline = pure_pursuit.centerline.detect_centerline
        self._original_detect_lines = pure_pursuit.corner_lines.detect_lines

        # Crear closures que usan la instancia
        def wrapped_detect_on(vision_self, frame):
            recorder = VisionCorpusRecorder._instance
            recorder.counters['detect_on'] += 1
            count = recorder.counters['detect_on']

            if count % recorder.every == 0:
                k = count // recorder.every - 1
                frame_copy = frame.copy()
                color_ranges_copy = {
                    name: [(lo.copy(), hi.copy()) for lo, hi in ranges]
                    for name, ranges in vision_self.color_ranges.items()
                }

                result_frame, positions = recorder._original_detect_on(vision_self, frame)

                sample = {
                    'frame': frame_copy,
                    'color_ranges': color_ranges_copy,
                    'result_frame': result_frame,
                    'positions': positions,
                }
                recorder._save_sample(k, 'detect_on', sample)

                return result_frame, positions
            else:
                return recorder._original_detect_on(vision_self, frame)

        def wrapped_detect_centerline(bev_bgr, bev_obstacles, bev_hsv=None, obstacle_conf=None, stats_out=None):
            recorder = VisionCorpusRecorder._instance
            recorder.counters['detect_centerline'] += 1
            count = recorder.counters['detect_centerline']

            if count % recorder.every == 0:
                k = count // recorder.every - 1

                args_copy = {
                    'bev_bgr': bev_bgr.copy() if isinstance(bev_bgr, np.ndarray) else bev_bgr,
                    'bev_obstacles': list(bev_obstacles),
                    'bev_hsv': bev_hsv.copy() if isinstance(bev_hsv, np.ndarray) else bev_hsv,
                    'obstacle_conf': list(obstacle_conf) if obstacle_conf is not None else None,
                }

                result = recorder._original_detect_centerline(
                    bev_bgr, bev_obstacles, bev_hsv, obstacle_conf, stats_out
                )

                if stats_out:
                    args_copy['stats_out'] = dict(stats_out)

                sample = {
                    'args': args_copy,
                    'result': result,
                }
                recorder._save_sample(k, 'detect_centerline', sample)

                return result
            else:
                return recorder._original_detect_centerline(
                    bev_bgr, bev_obstacles, bev_hsv, obstacle_conf, stats_out
                )

        def wrapped_detect_lines(bev_bgr, bev_hsv=None):
            recorder = VisionCorpusRecorder._instance
            recorder.counters['detect_lines'] += 1
            count = recorder.counters['detect_lines']

            if count % recorder.every == 0:
                k = count // recorder.every - 1

                args_copy = {
                    'bev_bgr': bev_bgr.copy() if isinstance(bev_bgr, np.ndarray) else bev_bgr,
                    'bev_hsv': bev_hsv.copy() if isinstance(bev_hsv, np.ndarray) else bev_hsv,
                }

                result = recorder._original_detect_lines(bev_bgr, bev_hsv)

                sample = {
                    'args': args_copy,
                    'result': result,
                }
                recorder._save_sample(k, 'detect_lines', sample)

                return result
            else:
                return recorder._original_detect_lines(bev_bgr, bev_hsv)

        # Reemplazar métodos de clase / funciones
        vision.Vision.detect_on = wrapped_detect_on
        pure_pursuit.centerline.detect_centerline = wrapped_detect_centerline
        pure_pursuit.runtime_nuevo.detect_centerline = wrapped_detect_centerline
        pure_pursuit.corner_lines.detect_lines = wrapped_detect_lines

    def remove_wrappers(self):
        """Restaura las funciones originales."""
        if self._original_detect_on is not None:
            vision.Vision.detect_on = self._original_detect_on
        if self._original_detect_centerline is not None:
            pure_pursuit.centerline.detect_centerline = self._original_detect_centerline
            pure_pursuit.runtime_nuevo.detect_centerline = self._original_detect_centerline
        if self._original_detect_lines is not None:
            pure_pursuit.corner_lines.detect_lines = self._original_detect_lines

    def _save_sample(self, k, func_name, sample):
        """Guarda una muestra en NPZ y JSON."""
        # Snapshot perezoso de la config activa (pi_overrides ya aplicados por
        # Sim.run() a esta altura): es constante durante toda la sim, así que
        # basta tomarlo una vez. Se escribe a out_dir/meta.json al terminar
        # record_corpus() y fingerprint lo reaplica antes de reproducir.
        if self.config_snapshot is None and self.config_keys:
            self.config_snapshot = {
                key: getattr(C, key) for key in sorted(self.config_keys) if hasattr(C, key)
            }

        stage_dir = self.out_dir / func_name
        stage_dir.mkdir(parents=True, exist_ok=True)

        npz_path = stage_dir / f"{k:06d}.npz"
        json_path = stage_dir / f"{k:06d}.json"

        # Separar arrays y crear estructura JSON con referencias
        arrays_dict = {}
        array_counter = [0]  # Contador para nombres únicos

        def extract_arrays(obj):
            """Extrae arrays y devuelve estructura con referencias."""
            if isinstance(obj, dict):
                return {k: extract_arrays(v) for k, v in obj.items()}
            elif isinstance(obj, (list, tuple)):
                return [extract_arrays(v) for v in obj]
            elif isinstance(obj, np.ndarray):
                # Guardar array con nombre único
                arr_name = f"arr_{array_counter[0]}"
                array_counter[0] += 1
                arrays_dict[arr_name] = obj
                return {'__npz__': arr_name}
            else:
                return obj

        json_structure = extract_arrays(sample)

        # Guardar NPZ (si hay arrays)
        if arrays_dict:
            np.savez_compressed(str(npz_path), **arrays_dict)

        # Guardar JSON con floats canonizados
        with open(json_path, 'w') as f:
            json.dump(self._canonize(json_structure), f, indent=2, default=str)

        self.samples[(func_name, k)] = {
            'npz': str(npz_path),
            'json': str(json_path),
        }

    def _canonize(self, obj):
        """Canoniza floats y tipos numpy recursivamente para el JSON de la MUESTRA.

        A diferencia de _canonize_for_hash (una via, solo para hashear la
        salida), esto debe ser REVERSIBLE: fingerprint reconstruye estos
        argumentos y los pasa a la función original, así que un float no
        puede quedar como string suelto (se pasaría un str a código que
        espera float). Se envuelve en {'__float__': hex} -- ver
        _reconstruct_from_json, que lo revierte con float.fromhex().
        Los ndarray ya salieron de aquí vía extract_arrays() antes de
        llamar a _canonize; esta rama es defensiva.
        """
        if isinstance(obj, dict):
            return {k: self._canonize(v) for k, v in sorted(obj.items())}
        elif isinstance(obj, (list, tuple)):
            return [self._canonize(v) for v in obj]
        elif isinstance(obj, np.ndarray):
            return ['nd', obj.dtype.str, list(obj.shape),
                    sha256(np.ascontiguousarray(obj).tobytes()).hexdigest()]
        elif isinstance(obj, float):
            return {'__float__': obj.hex()}
        elif isinstance(obj, (np.floating, np.integer)):
            return self._canonize(obj.item())
        elif isinstance(obj, np.bool_):
            return bool(obj)
        else:
            return obj


def record_corpus(seed: int, max_time: float, every: int, out_dir: Path):
    """Ejecuta una sim y graba muestras de funciones de visión."""
    print(f"Ejecutando Sim(seed={seed}, max_time_s={max_time}) con sampleo cada {every} llamadas...",
          file=sys.stderr)

    out_dir = Path(out_dir)

    # Construir la Sim ANTES de instalar los wrappers: __init__ no corre
    # ningún frame (eso es .run()) y ya resuelve pi_overrides / verdad del
    # mapa inyectada, que es justo lo que necesitamos para saber qué claves
    # de config snapshotear durante el record.
    sim = Sim(seed=seed, preset='hw_nuevo', max_time_s=max_time)
    map_truth_attrs = {_MAP_TRUTH_KEYS[key][0] for key in sim.inject_map_truth}
    config_keys = set(sim.pi_overrides.keys()) | map_truth_attrs

    recorder = VisionCorpusRecorder(every, out_dir, config_keys)
    recorder.install_wrappers()

    try:
        result = sim.run()

        print(f"Sim completada. {len(recorder.samples)} muestras grabadas en {out_dir}", file=sys.stderr)

        # meta.json: config que estaba activa durante el record (para que
        # fingerprint la reaplique) + datos de procedencia.
        out_dir.mkdir(parents=True, exist_ok=True)
        meta = {
            'seed': seed,
            'max_time_s': max_time,
            'every': every,
            'preset': 'hw_nuevo',
            'config_overrides': recorder.config_snapshot or {},
        }
        with open(out_dir / 'meta.json', 'w') as f:
            json.dump(meta, f, indent=2, sort_keys=True)

        return recorder.samples
    finally:
        recorder.remove_wrappers()


def _canonize_for_hash(obj):
    """Canoniza recursivamente para hashing (igual que VisionCorpusRecorder._canonize)."""
    if isinstance(obj, dict):
        return {k: _canonize_for_hash(v) for k, v in sorted(obj.items())}
    elif isinstance(obj, (list, tuple)):
        return [_canonize_for_hash(v) for v in obj]
    elif isinstance(obj, np.ndarray):
        return ['nd', obj.dtype.str, list(obj.shape),
                sha256(np.ascontiguousarray(obj).tobytes()).hexdigest()]
    elif isinstance(obj, float):
        return obj.hex()
    elif isinstance(obj, (np.floating, np.integer)):
        return _canonize_for_hash(obj.item())
    elif isinstance(obj, np.bool_):
        return bool(obj)
    else:
        return obj


def _reconstruct_arrays_from_npz(npz_path):
    """Carga arrays desde archivo NPZ."""
    if not Path(npz_path).exists():
        return {}
    npz_data = np.load(npz_path)
    return {k: v for k, v in npz_data.items()}


def _reconstruct_from_json(json_data, npz_arrays):
    """Reconstruye estructura reemplazando referencias {'__npz__': name} con arrays."""
    def restore_arrays(obj):
        if isinstance(obj, dict):
            if '__npz__' in obj and len(obj) == 1:
                # Reemplazar referencia con array
                arr_name = obj['__npz__']
                return npz_arrays.get(arr_name)
            elif '__float__' in obj and len(obj) == 1:
                # Revertir el float.hex() de _canonize (ver vision_corpus.py:_canonize).
                return float.fromhex(obj['__float__'])
            else:
                return {k: restore_arrays(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [restore_arrays(v) for v in obj]
        else:
            return obj

    return restore_arrays(json_data)


def _compute_sample_hash(output_data):
    """Computa hash SHA256 canónico de datos de salida."""
    canonical = _canonize_for_hash(output_data)
    json_str = json.dumps(canonical, sort_keys=True)
    return sha256(json_str.encode()).hexdigest()


def _call_original_and_hash(func_name, json_data, npz_arrays):
    """Reproduce la llamada original y hashea la salida."""
    # Reconstruir datos completos desde JSON + NPZ
    full_data = _reconstruct_from_json(json_data, npz_arrays)

    if func_name == 'detect_on':
        # full_data = {'frame': array, 'color_ranges': dict_of_ranges, ...}
        frame = full_data.get('frame')
        color_ranges_data = full_data.get('color_ranges', {})

        # Reconstruir Vision
        v = vision.Vision(open_cam=False)
        # color_ranges_data ya debería ser la estructura correcta
        v.color_ranges = color_ranges_data

        # Llamar original. El frame anotado SÍ es parte del fingerprint: no
        # es un subproducto descartable -- runtime_nuevo.py pasa processed_frame
        # (el resultado de detect_on) a bev.warp(), _park_pink() y
        # _check_parking_search(), es decir, es entrada de control, no solo un
        # dibujo. (Antes esta función devolvía solo `positions`; T17 auditoría
        # infra-vision-corpus: una mutación que anula cv2.putText en vision.py
        # cambia 106/171 frames anotados sin mover ni un hash porque no se
        # hasheaban.)
        result_frame, positions = v.detect_on(frame.copy())
        return {'result_frame': result_frame, 'positions': positions}

    elif func_name == 'detect_centerline':
        # full_data = {'args': {'bev_bgr': array, 'bev_obstacles': list, ...}, 'result': ...}
        args = full_data.get('args', {})
        bev_bgr = args.get('bev_bgr')
        bev_obstacles = args.get('bev_obstacles', [])
        bev_hsv = args.get('bev_hsv')
        obstacle_conf = args.get('obstacle_conf')

        stats_out = {}
        result = pure_pursuit.centerline.detect_centerline(
            bev_bgr, bev_obstacles, bev_hsv, obstacle_conf, stats_out
        )

        output = {
            'result': result,
            'stats_out': stats_out,
        }
        return output

    elif func_name == 'detect_lines':
        # full_data = {'args': {'bev_bgr': array, 'bev_hsv': array}, 'result': ...}
        args = full_data.get('args', {})
        bev_bgr = args.get('bev_bgr')
        bev_hsv = args.get('bev_hsv')

        result = pure_pursuit.corner_lines.detect_lines(bev_bgr, bev_hsv)
        return result

    else:
        raise ValueError(f"Función desconocida: {func_name}")


def _recorded_output(func_name, full_data):
    """Extrae de la muestra GRABADA el mismo valor que _call_original_and_hash
    devuelve para la llamada REPRODUCIDA, para el self-check replay==grabado."""
    if func_name == 'detect_on':
        return {'result_frame': full_data.get('result_frame'), 'positions': full_data.get('positions')}
    elif func_name == 'detect_centerline':
        return {
            'result': full_data.get('result'),
            'stats_out': full_data.get('args', {}).get('stats_out', {}),
        }
    elif func_name == 'detect_lines':
        return full_data.get('result')
    else:
        raise ValueError(f"Función desconocida: {func_name}")


def _apply_config_overrides(corpus_dir: Path) -> dict:
    """Aplica a pure_pursuit.config los overrides grabados en meta.json
    (si existe) y devuelve el snapshot previo para poder restaurarlo.

    Sin esto, fingerprint reproduce las llamadas con los defaults de
    config.py en vez de los pi_overrides que la sim aplicó durante el
    record (p.ej. hw_nuevo: CENTERLINE_TOP_Y 30 en vez de 133,
    OBS_INFLATE_R 56 en vez de 40, CENTERLINE_RAMP_PX 220 en vez de 130),
    y detect_centerline/detect_lines no reproducen lo grabado (T17
    auditoría infra-vision-corpus).
    """
    meta_file = corpus_dir / 'meta.json'
    restore_cfg: dict = {}
    if not meta_file.exists():
        print(f"WARN: no hay {meta_file}; fingerprint usa los defaults de "
              f"pure_pursuit.config (puede no coincidir con los pi_overrides "
              f"usados al grabar este corpus)", file=sys.stderr)
        return restore_cfg

    with open(meta_file) as f:
        meta = json.load(f)

    for key, value in meta.get('config_overrides', {}).items():
        if hasattr(C, key):
            restore_cfg[key] = getattr(C, key)
            setattr(C, key, value)
        else:
            print(f"WARN: meta.json referencia config.{key}, que no existe "
                  f"en esta versión de pure_pursuit.config; se ignora", file=sys.stderr)
    return restore_cfg


def fingerprint_corpus(corpus_dir: Path, out_file: Path) -> int:
    """Calcula fingerprint canónico de un corpus. Devuelve 0 si no hubo
    errores ni muestras saltadas/discrepantes, 1 si los hubo (el caller
    decide si eso debe ser el exit code del proceso)."""
    corpus_dir = Path(corpus_dir)
    samples_data = {}
    errors = 0
    skipped = 0

    restore_cfg = _apply_config_overrides(corpus_dir)
    try:
        # Para cada función en el corpus
        for func_dir in sorted(corpus_dir.iterdir()):
            if not func_dir.is_dir():
                continue

            func_name = func_dir.name
            func_samples = {}

            # Para cada muestra (k)
            for json_file in sorted(func_dir.glob('*.json')):
                k_str = json_file.stem
                npz_file = json_file.parent / f"{k_str}.npz"

                if not npz_file.exists():
                    print(f"WARN: No NPZ for {json_file}", file=sys.stderr)
                    skipped += 1
                    continue

                # Cargar datos
                with open(json_file) as f:
                    json_data = json.load(f)

                npz_arrays = _reconstruct_arrays_from_npz(npz_file)

                # Reproducir llamada y hashear salida
                try:
                    output = _call_original_and_hash(func_name, json_data, npz_arrays)
                    fp = _compute_sample_hash(output)

                    # Self-check: el replay debe reproducir bit a bit lo
                    # grabado en el host de grabación. Si no, el corpus no es
                    # fiel a lo que vio el código (config distinta, bug de
                    # reconstrucción, etc.) y hay que fallar, no solo anotar
                    # un hash que nadie va a poder comparar con nada real.
                    full_data = _reconstruct_from_json(json_data, npz_arrays)
                    recorded_fp = _compute_sample_hash(_recorded_output(func_name, full_data))
                    if recorded_fp != fp:
                        raise AssertionError(
                            f"replay ({fp[:12]}...) != grabado ({recorded_fp[:12]}...); "
                            f"revisa meta.json/config_overrides de {corpus_dir}"
                        )

                    func_samples[k_str] = fp
                except Exception as e:
                    print(f"ERROR en {func_name}/{k_str}: {e}", file=sys.stderr)
                    import traceback
                    traceback.print_exc(file=sys.stderr)
                    func_samples[k_str] = None
                    errors += 1

            samples_data[func_name] = func_samples
    finally:
        for key, value in restore_cfg.items():
            setattr(C, key, value)

    # Crear resultado con metadata
    result = {
        'host': platform.platform(),
        'numpy': np.__version__,
        'cv2': cv2.__version__,
        'samples': samples_data,
    }

    # Escribir resultado (incluso si hubo errores: sirve para diagnosticar)
    with open(out_file, 'w') as f:
        json.dump(result, f, indent=2)

    if errors or skipped:
        print(f"FINGERPRINT INCOMPLETO: {errors} error(es)/discrepancia(s), "
              f"{skipped} muestra(s) sin NPZ -- ver {out_file}", file=sys.stderr)
        return 1

    print(f"Fingerprint escrito en {out_file}", file=sys.stderr)
    return 0


def compare_corpus_fingerprints(corpus_dir: Path, fingerprint_file: Path):
    """Compara corpus actual con fingerprint guardado."""
    # Generar fingerprint del corpus actual
    current_fp_file = Path(fingerprint_file.parent) / 'current_fp.json'
    fp_exit = fingerprint_corpus(corpus_dir, current_fp_file)
    if fp_exit != 0:
        # fingerprint ya logueó el detalle (errores/discrepancias/NPZ
        # faltantes) en stderr y en current_fp.json; no tiene sentido seguir
        # comparando un fingerprint que no se pudo calcular con confianza.
        print(f"ERROR: no se pudo calcular el fingerprint actual de {corpus_dir} "
              f"sin errores (ver arriba) -- compare aborta", file=sys.stderr)
        return 1

    # Cargar ambos
    with open(fingerprint_file) as f:
        expected = json.load(f)

    with open(current_fp_file) as f:
        current = json.load(f)

    # Comparar
    diffs = []
    exp_samples = expected.get('samples', {})
    cur_samples = current.get('samples', {})

    all_keys = set(exp_samples.keys()) | set(cur_samples.keys())

    for func_name in sorted(all_keys):
        expected_samples = exp_samples.get(func_name, {})
        current_samples = cur_samples.get(func_name, {})

        all_sample_keys = set(expected_samples.keys()) | set(current_samples.keys())
        for k in sorted(all_sample_keys):
            exp_fp = expected_samples.get(k)
            cur_fp = current_samples.get(k)

            # None (fingerprint esperado con error, o muestra que no existe
            # en un lado) NUNCA cuenta como "igual", aunque ambos lados sean
            # None: eso es "no se pudo verificar", no "idéntico" (T17
            # auditoría infra-vision-corpus).
            if exp_fp is None or cur_fp is None or exp_fp != cur_fp:
                diffs.append((func_name, k, exp_fp, cur_fp))

    # Reportar
    total_samples = sum(len(f) for f in exp_samples.values())
    if total_samples == 0:
        print("ERROR: el fingerprint esperado no tiene muestras (0/0) -- "
              "nada que comparar, no es un IDENTICO válido")
        return 1
    elif not diffs:
        print(f"IDENTICO {total_samples}/{total_samples}")
        return 0
    else:
        print(f"DIFF {len(diffs)}/{total_samples}")
        for func_name, k, exp_fp, cur_fp in diffs[:20]:
            print(f"  {func_name} {k}: {exp_fp} -> {cur_fp}")
        return 1


def main():
    parser = argparse.ArgumentParser(description='Vision corpus: record, fingerprint, compare.')

    subparsers = parser.add_subparsers(dest='command', help='Subcomando')

    # RECORD
    record_parser = subparsers.add_parser('record', help='Grabar muestras durante sim')
    record_parser.add_argument('--seed', type=int, default=3170839, help='Seed aleatorio')
    record_parser.add_argument('--max-time', type=float, default=20, help='Tiempo máximo de sim')
    record_parser.add_argument('--every', type=int, default=1, help='Grabar cada N llamadas')
    record_parser.add_argument('--out', required=True, help='Directorio de salida')

    # FINGERPRINT
    fp_parser = subparsers.add_parser('fingerprint', help='Calcular fingerprint canónico')
    fp_parser.add_argument('--corpus', required=True, help='Directorio con muestras')
    fp_parser.add_argument('--out', required=True, help='Archivo de salida JSON')

    # COMPARE
    cmp_parser = subparsers.add_parser('compare', help='Comparar corpus con fingerprint')
    cmp_parser.add_argument('--corpus', required=True, help='Directorio con muestras')
    cmp_parser.add_argument('--fp', required=True, help='Archivo de fingerprint esperado')

    args = parser.parse_args()

    if args.command == 'record':
        record_corpus(args.seed, args.max_time, args.every, Path(args.out))
        sys.exit(0)
    elif args.command == 'fingerprint':
        sys.exit(fingerprint_corpus(Path(args.corpus), Path(args.out)))
    elif args.command == 'compare':
        exit_code = compare_corpus_fingerprints(Path(args.corpus), Path(args.fp))
        sys.exit(exit_code)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == '__main__':
    main()
