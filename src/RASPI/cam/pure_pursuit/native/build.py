"""Build native library for fox_native."""
import os
import sys
import platform
import subprocess
import hashlib
import shutil
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_SRC = _HERE / 'src'
_OUT = Path(os.environ.get('FOX_NATIVE_BUILD_DIR', _HERE / '_build'))

FLAGS = ['-std=c99', '-O2', '-fno-fast-math', '-ffp-contract=off', '-shared']


def flags():
    """Return compilation flags."""
    f = list(FLAGS)
    if platform.system() != 'Windows':
        f.append('-fPIC')
    if platform.system() == 'Darwin':
        f.extend(['-arch', platform.machine()])
    if platform.system() == 'Linux':
        f.append('-fopenmp')
    return f


def compiler():
    """Determine the C compiler."""
    if 'FOX_CC' in os.environ:
        return os.environ['FOX_CC'].split()

    if platform.system() == 'Windows':
        try:
            import ziglang  # noqa: F401
            return [sys.executable, '-m', 'ziglang', 'cc', '-target', 'x86_64-windows-gnu']
        except ImportError:
            pass

    for cc in ['cc', 'gcc', 'clang']:
        if shutil.which(cc):
            return [cc]

    raise RuntimeError('No C compiler found (cc, gcc, clang)')


def _ext():
    """Return platform-specific extension."""
    if platform.system() == 'Windows':
        return '.dll'
    elif platform.system() == 'Darwin':
        return '.dylib'
    else:
        return '.so'


def build_hash(cc, fl):
    """Compute build hash from compiler, flags, and sources."""
    h = hashlib.sha256()
    for p in sorted(_SRC.glob('*.[ch]')):
        h.update(p.name.encode())
        h.update(p.read_bytes())
    key = repr((cc[1:] if 'ziglang' in cc else cc, fl, platform.system(), platform.machine()))
    h.update(key.encode())
    return h.hexdigest()[:16]


def _replace_or_keep(src: Path, dst: Path) -> None:
    """os.replace; en Windows falla (WinError 5) si otro proceso del batch tiene
    dst abierto. El nombre va por hash: si dst ya existe, es el mismo contenido."""
    try:
        os.replace(src, dst)
    except PermissionError:
        if not dst.is_file():
            raise
        src.unlink(missing_ok=True)


def build():
    """Build native library. Returns Path to .dll/.dylib/.so."""
    cc = compiler()
    fl = flags()
    ext = _ext()
    # Con OpenMP (Linux) primero; sin libgomp, la versión serial (mismos resultados).
    serial = [f for f in fl if f != '-fopenmp']
    variants = [fl] if serial == fl else [fl, serial]
    outs = [_OUT / f'libfox_native_{build_hash(cc, v)}{ext}' for v in variants]
    for out in outs:
        if out.exists():
            return out

    _OUT.mkdir(parents=True, exist_ok=True)
    for i, (v, out) in enumerate(zip(variants, outs)):
        tmp = _OUT / f'{out.stem}.{os.getpid()}.tmp{ext}'

        cmd = cc + v + ['-I', str(_SRC), '-o', str(tmp)]
        if platform.system() == 'Windows':
            # El linker nombra el import-lib según el primer .c (p.ej. 'abi.lib'),
            # no según -o: sin esto, builds concurrentes a _OUT colisionan en el
            # mismo nombre. Forzamos el nombre a tmp.lib para que la limpieza de
            # abajo y _replace_or_keep operen sobre una ruta única por proceso.
            cmd += ['-Wl,--out-implib,' + str(tmp.with_suffix('.lib'))]
        cmd += [str(s) for s in sorted(_SRC.glob('*.c'))]
        result = subprocess.run(cmd, stdin=subprocess.DEVNULL, capture_output=True, text=True)

        if result.returncode != 0:
            if i + 1 < len(variants):
                continue                 # sin libgomp: reintenta sin -fopenmp
            raise RuntimeError(result.stderr[-2000:])

        tmp.with_suffix('.lib').unlink(missing_ok=True)
        tmp.with_suffix('.pdb').unlink(missing_ok=True)
        out.with_suffix('.lib').unlink(missing_ok=True)
        out.with_suffix('.pdb').unlink(missing_ok=True)

        _replace_or_keep(tmp, out)
        return out


if __name__ == '__main__':
    print(build())
