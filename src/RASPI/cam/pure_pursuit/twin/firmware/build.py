"""Build PurePursuit.ino as a host shared library for SIL."""

from __future__ import annotations

import hashlib
import os
import platform
import re
import shutil
import subprocess
from pathlib import Path

_PKG = Path(__file__).resolve().parent
_SIL = _PKG / "sil"
_BUILD = _PKG / "_build"
_REPO = Path(__file__).resolve().parents[6]
_INO = Path(__file__).resolve().parents[5] / "ESP32" / "PurePursuit" / "PurePursuit.ino"
_INO_GIT = "src/ESP32/PurePursuit/PurePursuit.ino"

_SHIM_SOURCES = (
    _SIL / "sil_core.cpp",
    _SIL / "Arduino.h",
    _SIL / "Wire.h",
    _SIL / "MPU6050_tockn.h",
)

_CONST_RE = re.compile(
    r"^(const\s+[\w\s]+?\s+)(?P<name>[A-Za-z_]\w*)(\s*=\s*)(?P<val>[^;]+)(;)",
    re.MULTILINE,
)


def _apply_overrides(text: str, overrides: dict[str, str]) -> str:
    for name, new_val in overrides.items():
        count = 0

        def repl(m: re.Match[str]) -> str:
            nonlocal count
            if m.group("name") != name:
                return m.group(0)
            count += 1
            return f"{m.group(1)}{name}{m.group(3)}{new_val}{m.group(5)}"

        text = _CONST_RE.sub(repl, text)
        if count != 1:
            raise ValueError(f"override {name!r}: expected exactly one match, got {count}")
    return text


def load_ino_text(source: str = "worktree") -> str:
    """head = firmware en HEAD del repo; worktree = archivo local."""
    if source == "worktree":
        return _INO.read_text(encoding="utf-8")
    if source == "head":
        proc = subprocess.run(
            ["git", "show", f"HEAD:{_INO_GIT}"],
            cwd=_REPO,
            capture_output=True,
            text=True,
            check=True,
        )
        return proc.stdout
    raise ValueError(f"fw source desconocido: {source!r} (use 'head' o 'worktree')")


def _build_hash(
    ino_text: str,
    source: str,
    overrides: dict[str, str] | None,
    defines: dict[str, str] | None,
) -> str:
    h = hashlib.sha256()
    h.update(source.encode("utf-8"))
    h.update(ino_text.encode("utf-8"))
    h.update(repr(sorted((overrides or {}).items())).encode())
    h.update(repr(sorted((defines or {}).items())).encode())
    for p in _SHIM_SOURCES:
        h.update(p.read_bytes())
    return h.hexdigest()[:16]


def _cxx() -> list[str]:
    """FOX_CXX > c++ del PATH > zig (pip install ziglang) en Windows."""
    env = os.environ.get("FOX_CXX")
    if env:
        return env.split()
    if shutil.which("c++"):
        return ["c++"]
    try:
        import ziglang  # noqa: F401
    except ImportError:
        raise RuntimeError("sin compilador C++: instala uno o `pip install ziglang`")
    import sys
    return [sys.executable, "-m", "ziglang", "c++"]


def build(
    overrides: dict[str, str] | None = None,
    defines: dict[str, str] | None = None,
    force: bool = False,
    source: str = "worktree",
) -> Path:
    overrides = dict(overrides or {})
    defines = dict(defines or {})

    ino_text = load_ino_text(source)
    digest = _build_hash(ino_text, source, overrides, defines)
    system = platform.system()
    ext = {"Darwin": ".dylib", "Windows": ".dll"}.get(system, ".so")
    out = _BUILD / f"libfw_{digest}{ext}"

    if out.is_file() and not force:
        return out

    _BUILD.mkdir(parents=True, exist_ok=True)
    patched = _apply_overrides(ino_text, overrides)
    # Varios procesos (batch en paralelo) pueden compilar a la vez: fuente por
    # hash escrita de forma atómica y salida temporal por proceso.
    # En Windows os.replace falla (WinError 5) si otro proceso tiene abierto
    # el destino: cada proceso compila su propia copia de la fuente.
    sil_cpp = _BUILD / f"PurePursuit_sil_{digest}.{os.getpid()}.cpp"
    sil_cpp.write_text('#include "Arduino.h"\n' + patched, encoding="utf-8")
    tmp_out = _BUILD / f"libfw_{digest}.{os.getpid()}.tmp{ext}"

    cmd = [
        *_cxx(),
        "-std=c++17",
        "-O1",
        "-g",
        "-fPIC",
        "-shared",
        "-Wno-everything",
        f"-I{_SIL}",
        "-DFOX_SIL=1",
    ]
    if system == "Windows":
        # lld-link no exporta nada sin __declspec; ctypes necesita los sil_*.
        cmd.extend(["-target", "x86_64-windows-gnu", "-Wl,--export-all-symbols"])
    for k, v in defines.items():
        cmd.append(f"-D{k}={v}")
    cmd.extend([str(_SIL / "sil_core.cpp"), str(sil_cpp), "-o", str(tmp_out)])

    try:
        # stdin cerrado: en Windows zig se colgaba heredando el stdin de la terminal.
        subprocess.run(cmd, check=True, stdin=subprocess.DEVNULL)
        try:
            os.replace(tmp_out, out)
        except PermissionError:
            # Otro proceso ya dejó la misma .dll (mismo hash) y la tiene cargada.
            if not out.is_file():
                raise
    finally:
        tmp_out.unlink(missing_ok=True)
        sil_cpp.unlink(missing_ok=True)
    return out
