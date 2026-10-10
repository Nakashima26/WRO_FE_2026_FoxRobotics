"""Compila CalibFox.ino contra el mock de Arduino y lo expone como un 'puerto serie'.

Sirve para (a) probar el parser/CSV del firmware sin ESP32 y (b) ensayar
calib_capture.py en seco (`--port mock`). Necesita un compilador C++ (FOX_CXX, c++ del
PATH o `pip install ziglang`). La primera compilacion con zig tarda ~5 min (libc++);
las siguientes ~15 s.  NO valida timing real, I2C, ISR ni el core de ESP32.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

from . import calib_common as cc

HERE = Path(__file__).resolve().parent


def ino_path() -> Path:
    return cc.repo_root(HERE) / "src" / "ESP32" / "CalibFox" / "CalibFox.ino"


def _cxx() -> list[str]:
    if os.environ.get("FOX_CXX"):
        return [os.environ["FOX_CXX"]]
    if shutil.which("c++"):
        return ["c++"]
    try:
        import ziglang  # noqa: F401
    except ImportError as e:
        raise RuntimeError("sin compilador C++: instala uno o `pip install ziglang`") from e
    return [sys.executable, "-m", "ziglang", "c++"]


def build(out_dir: Path, extra_flags: list[str] | None = None) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    exe = out_dir / ("calibfox_mock.exe" if os.name == "nt" else "calibfox_mock")
    ino = ino_path()
    srcs = [ino, HERE / "mock" / "mock_main.cpp"]
    if exe.exists() and exe.stat().st_mtime >= max(p.stat().st_mtime for p in
                                                   [*srcs, *(HERE / "mock").glob("*.h")]):
        return exe
    cmd = [*_cxx(), "-std=c++17", "-O1", "-Wall", "-Wextra", "-Wno-nullability-completeness",
           "-I", str(HERE / "mock"), *(extra_flags or []),
           "-x", "c++", str(ino), "-x", "c++", str(HERE / "mock" / "mock_main.cpp"), "-o", str(exe)]
    r = subprocess.run(cmd, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    if r.returncode != 0:
        raise RuntimeError("fallo la compilacion del mock:\n" + r.stderr[-4000:])
    return exe


class MockPort:
    """Misma interfaz minima que pyserial: write(bytes), readline() -> bytes, close()."""

    def __init__(self, exe: Path, env: dict[str, str] | None = None):
        e = dict(os.environ)
        e.update(env or {})
        self.proc = subprocess.Popen([str(exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     env=e, bufsize=0)

    def write(self, data: bytes) -> int:
        self.proc.stdin.write(data)
        self.proc.stdin.flush()
        return len(data)

    def readline(self) -> bytes:
        return self.proc.stdout.readline()

    def reset_input_buffer(self) -> None:
        pass

    def close(self) -> None:
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        self.proc.wait(timeout=5)
