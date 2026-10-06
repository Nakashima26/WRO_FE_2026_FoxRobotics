"""Native library loader for fox_native."""
import os
import sys
import ctypes
import numpy as np

from .build import build

_LIB = None
_TRIED = False
_DISABLED = set()


def enabled():
    """Check if native library is enabled."""
    return os.environ.get('FOX_NATIVE', '1') != '0'


def _warn(msg):
    """Print warning to stderr."""
    print(f'[native] {msg}', file=sys.stderr, flush=True)


def get_lib():
    """Load and return native library, or None if disabled/failed."""
    global _LIB, _TRIED

    if not enabled():
        return None

    if _TRIED:
        return _LIB

    _TRIED = True

    try:
        lib = ctypes.CDLL(str(build()))

        # Declare ABI functions
        lib.fox_abi_version.argtypes = []
        lib.fox_abi_version.restype = ctypes.c_int32
        lib.fox_flt_eval_method.argtypes = []
        lib.fox_flt_eval_method.restype = ctypes.c_int32
        lib.fox_probe_contract_d.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double]
        lib.fox_probe_contract_d.restype = ctypes.c_double
        lib.fox_probe_contract_f.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
        lib.fox_probe_contract_f.restype = ctypes.c_float

        # Check ABI version
        abi_version = lib.fox_abi_version()
        if abi_version != 1:
            raise RuntimeError(f'ABI version mismatch: {abi_version}')

        # Check FLT_EVAL_METHOD
        flt_eval = lib.fox_flt_eval_method()
        if flt_eval != 0:
            raise RuntimeError(f'FLT_EVAL_METHOD={flt_eval}, esperado 0')

        # Probe double contraction
        a = 1.0 + 2.0**-30
        b = 1.0 - 2.0**-30
        result_d = lib.fox_probe_contract_d(a, b, -1.0)
        expected_d = a * b + -1.0
        if result_d != expected_d:
            raise RuntimeError(f'double probe failed: {result_d} != {expected_d}')

        # Probe float contraction
        af = 1.0 + 2.0**-13
        bf = 1.0 - 2.0**-13
        result_f = lib.fox_probe_contract_f(af, bf, -1.0)
        expected_f = float(np.float32(af) * np.float32(bf) + np.float32(-1.0))
        if result_f != expected_f:
            raise RuntimeError(f'float probe failed: {result_f} != {expected_f}')

        _LIB = lib
    except Exception as e:
        _warn(f'deshabilitado: {e}')
        _LIB = None

    return _LIB


def get_kernel(name, argtypes, restype):
    """Get a kernel function by name, or None if unavailable."""
    lib = get_lib()
    if lib is None or name in _DISABLED:
        return None

    try:
        fn = getattr(lib, name)
    except AttributeError:
        _warn(f'kernel no encontrado: {name}')
        return None

    fn.argtypes = argtypes
    fn.restype = restype
    return fn


def disable(name, reason):
    """Disable a kernel function."""
    _DISABLED.add(name)
    _warn(f'deshabilitado: {name} ({reason})')


def _reset():
    """Reset state (for testing)."""
    global _LIB, _TRIED
    _LIB = None
    _TRIED = False
    _DISABLED.clear()
