"""Tests de equivalencia entre render_camera y la referencia extraída."""

from __future__ import annotations

import dataclasses
import os
import platform
import sys

import cv2
import numpy as np
import pytest

from pure_pursuit.twin.tests._render_ref import reference_render
from pure_pursuit.twin.camera import CameraModel
from pure_pursuit.twin.params import TwinParams
from pure_pursuit.wro_field import randomize


@pytest.fixture
def tp():
    """TwinParams base."""
    return TwinParams()


@pytest.fixture
def cam(tp):
    """CameraModel base."""
    return CameraModel(tp.camera, tp)


@pytest.fixture
def field(tp):
    """Field con track aleatorio (la mayoría de tests la modifican)."""
    field = randomize(11)  # seed 11 fijo para reproducibilidad
    return field


def _rng_of(seed_or_rng):
    """Acepta un seed entero o un Generator ya construido (para encadenar uniform+runner)."""
    if isinstance(seed_or_rng, np.random.Generator):
        return seed_or_rng
    return np.random.default_rng(seed_or_rng)


def _poses_uniform(seed_or_rng, n=30):
    """30 poses uniformes por seed (o rng compartido)."""
    rng = _rng_of(seed_or_rng)
    poses = []
    for _ in range(n):
        x = float(rng.uniform(-1450, 1450))
        y = float(rng.uniform(-1450, 1450))
        h = float(rng.uniform(-720, 720))
        poses.append((x, y, h))
    return poses


def _poses_runner(seed_or_rng, n=30):
    """30 poses de corredor (cercanas a paredes laterales), por seed o rng compartido."""
    rng = _rng_of(seed_or_rng)
    poses = []
    for _ in range(n):
        x = float(rng.choice([-1, 1]) * rng.uniform(600, 1400))
        y = float(rng.uniform(-1400, 1400))
        h = float(rng.choice([0.0, 90.0, 180.0, 270.0]) + rng.normal(0, 20))
        poses.append((x, y, h))
    return poses


@pytest.mark.parametrize("native", ["1", "0"])
@pytest.mark.parametrize("field_seed", [1, 2, 6, 14, 3170839])
def test_equiv(monkeypatch, native, field_seed, tp):
    """Test (a): equivalencia pixel-a-pixel con FOX_NATIVE=0 y 1, para cada campo de los goldens.

    Cada field_seed coincide con un golden de runs/t17/gold (s2, s6, s14, s3170839) y pone el
    cajon contra una pared distinta (ver wro_field.randomize), lo que importa por el orden del
    pintor en camera.py:225-229.
    """
    monkeypatch.setenv("FOX_NATIVE", native)

    # Reset native module si existe
    try:
        import pure_pursuit.native as N
        N._reset()
    except (ImportError, AttributeError):
        pass

    # CameraModel NUEVO por caso
    cam = CameraModel(tp.camera, tp)
    field = randomize(field_seed)

    # Mismo rng(11) para las 30 uniformes + 30 de corredor de este campo.
    rng = np.random.default_rng(11)
    poses = _poses_uniform(rng) + _poses_runner(rng)

    for x, y, h in poses:
        ref = reference_render(cam, field, x, y, h)
        actual = cam.render_camera(field, x, y, h)
        assert np.array_equal(actual, ref), (
            f"Mismatch at pose ({x:.2f}, {y:.2f}, {h:.2f}) "
            f"field_seed={field_seed} FOX_NATIVE={native}"
        )


def test_derribo(tp, field):
    """Test (b): derribo de signos."""
    cam = CameraModel(tp.camera, tp)

    # 5 poses
    poses = _poses_uniform(1, n=5)

    # Render antes de derribo
    renders_before = [cam.render_camera(field, x, y, h) for x, y, h in poses]
    refs_before = [reference_render(cam, field, x, y, h) for x, y, h in poses]

    # Derribo
    field.signs.pop(0)

    # Render después de derribo
    renders_after = [cam.render_camera(field, x, y, h) for x, y, h in poses]
    refs_after = [reference_render(cam, field, x, y, h) for x, y, h in poses]

    # Verificar equivalencia antes y después
    for i, (before, after) in enumerate(zip(refs_before, refs_after)):
        assert np.array_equal(renders_before[i], before)
        assert np.array_equal(renders_after[i], after)


def test_derribo_barrier(tp, field):
    """Test (b2): quitar una sola barrera (sin tocar signs) invalida cualquier caché de faces."""
    cam = CameraModel(tp.camera, tp)

    poses = _poses_uniform(14, n=5)

    renders_before = [cam.render_camera(field, x, y, h) for x, y, h in poses]
    refs_before = [reference_render(cam, field, x, y, h) for x, y, h in poses]

    # Quita una barrera (p.ej. una pared del cajón) sin tocar signs
    field.barriers.pop(0)

    renders_after = [cam.render_camera(field, x, y, h) for x, y, h in poses]
    refs_after = [reference_render(cam, field, x, y, h) for x, y, h in poses]

    for i, (before, after) in enumerate(zip(refs_before, refs_after)):
        assert np.array_equal(renders_before[i], before)
        assert np.array_equal(renders_after[i], after)


def test_clear_signs_barriers(tp, field):
    """Test (c): field.signs.clear() y field.barriers.clear(), con el mismo cam antes y después."""
    cam = CameraModel(tp.camera, tp)

    poses = _poses_uniform(2, n=10)

    # Render ANTES del clear, con el mismo cam que se reusará después. Esto expone
    # una caché de faces (keyed por signs/barriers) que no invalide al vaciar el campo.
    renders_before = [cam.render_camera(field, x, y, h) for x, y, h in poses]
    refs_before = [reference_render(cam, field, x, y, h) for x, y, h in poses]

    # Clear
    field.signs.clear()
    field.barriers.clear()

    # Render después de clear, con ese MISMO cam
    renders_after = [cam.render_camera(field, x, y, h) for x, y, h in poses]
    refs_after = [reference_render(cam, field, x, y, h) for x, y, h in poses]

    for render, ref in zip(renders_before, refs_before):
        assert np.array_equal(render, ref)
    for render, ref in zip(renders_after, refs_after):
        assert np.array_equal(render, ref)


def test_equidistant(tp, field):
    """Test (d): modo equidistant."""
    # Parámetros con equidistant=True
    tp2 = TwinParams()
    tp2.camera = dataclasses.replace(tp2.camera, equidistant=True)
    cam2 = CameraModel(tp2.camera, tp2)

    poses = _poses_uniform(6, n=10)

    for x, y, h in poses:
        actual = cam2.render_camera(field, x, y, h)
        ref = reference_render(cam2, field, x, y, h)
        assert np.array_equal(actual, ref), \
            f"Equidistant mismatch at ({x:.2f}, {y:.2f}, {h:.2f})"


def test_numpy_assumptions():
    """Test (e): suposiciones de numpy."""
    rng = np.random.default_rng(5)

    # Suposición 1: np.mean por eje
    a = rng.normal(0, 1000, (2000, 4, 3))
    z = a[..., 2]
    expected = np.array([np.mean(z[i]) for i in range(2000)])
    actual = np.mean(z, axis=1)
    assert np.array_equal(actual, expected), "np.mean by axis failed"

    # Suposición 2: matrix mult @ vs column_stack + manual
    M = rng.normal(size=(3, 3))
    a_subset = a[:, :, :3]

    expected = np.stack(
        [np.column_stack([a_subset[i, :, 0], a_subset[i, :, 1], a_subset[i, :, 2]]) @ M.T for i in range(2000)]
    )
    actual = a_subset @ M.T

    if not np.array_equal(actual, expected):
        # En Mac arm64 puede haber diferencias numéricas
        if platform.system() == "Darwin":
            pytest.xfail("Matrix multiply differs on Darwin; self-check will handle it")
            print(f"Warning: matrix mult precision differs; max diff = {np.max(np.abs(actual - expected))}")
        else:
            assert False, f"Matrix multiply failed; max diff = {np.max(np.abs(actual - expected))}"

    # Suposición 3: flip(flip(x, 1), 1) == x
    img = rng.integers(0, 256, (240, 320, 3), dtype=np.uint8)
    flipped = cv2.flip(cv2.flip(img, 1), 1)
    assert np.array_equal(flipped, img), "Double flip failed"
