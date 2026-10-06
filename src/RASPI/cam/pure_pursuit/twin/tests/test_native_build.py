"""Tests for native library build infrastructure."""
import subprocess
import sys
import tempfile
import os
from pathlib import Path

import pytest

from pure_pursuit.native import (
    get_lib,
    get_kernel,
    _reset,
)
from pure_pursuit.native.build import build


def test_native_load_and_abi():
    """Test (a): get_lib() loads successfully and ABI checks pass."""
    lib = get_lib()
    assert lib is not None, "Native library should load"
    assert lib.fox_abi_version() == 1, "ABI version must be 1"
    assert lib.fox_flt_eval_method() == 0, "FLT_EVAL_METHOD must be 0"


def test_native_disable_env(monkeypatch):
    """Test (b): FOX_NATIVE=0 disables native library."""
    monkeypatch.setenv('FOX_NATIVE', '0')
    _reset()
    assert get_lib() is None, "Library should be None when FOX_NATIVE=0"
    _reset()


def test_native_build_caching():
    """Test (c): build() returns same path when called twice, second call fast."""
    import time
    path1 = build()
    assert path1.exists(), f"Built library {path1} should exist"

    t0 = time.perf_counter()
    path2 = build()
    elapsed = time.perf_counter() - t0

    assert path1 == path2, "Second build() call should return same path"
    assert elapsed < 0.2, f"Second build() should be fast but took {elapsed}s"


def test_native_concurrent_build(tmp_path):
    """Test (d): 4 concurrent build() calls with FOX_NATIVE_BUILD_DIR work correctly."""
    script = """
import sys
from pure_pursuit.native.build import build
print(build())
"""
    env = os.environ.copy()
    env['FOX_NATIVE_BUILD_DIR'] = str(tmp_path)
    env['PYTHONPATH'] = '.'

    procs = []
    for _ in range(4):
        p = subprocess.Popen(
            [sys.executable, '-c', script],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdin=subprocess.DEVNULL,
            text=True,
            env=env,
        )
        procs.append(p)

    outputs = []
    for p in procs:
        stdout, stderr = p.communicate()
        assert p.returncode == 0, f"Subprocess failed: {stderr}"
        outputs.append(stdout.strip())

    # All should print the same path
    assert len(set(outputs)) == 1, f"All builds should produce same path: {outputs}"

    # No leftover .tmp* files
    tmp_files = list(tmp_path.glob('*.tmp*'))
    assert len(tmp_files) == 0, f"Found leftover tmp files: {tmp_files}"
