"""Equivalencia exacta del kernel transpuesto, incluidos back y costes saturados."""
import numpy as np
import pytest

from pure_pursuit import ring_route as RR


@pytest.mark.parametrize('m', [1, 2, 5, 17, 64, 256])
@pytest.mark.parametrize('scenario', ['normal', 'saturado', 'empates', 'nan'])
def test_sparse_t_matches_sparse_buffers(m, scenario):
    sparse, transposed = RR._dp_sparse_kernel(), RR._dp_sparse_t_kernel()
    if sparse is None or transposed is None:
        pytest.skip('biblioteca nativa no disponible')
    rng = np.random.default_rng(209 + m)
    n, steps = 3, 6
    g = rng.uniform(0, 3, (n, m, m, m))
    mask = rng.random(g.shape) < 0.7
    ds = rng.uniform(0, 5, (n, m, m))
    pen = rng.uniform(0, 2, ds.shape)
    bonus = rng.uniform(0, 2, ds.shape)
    bad = (rng.random(ds.shape) < 0.1).astype(np.uint8)
    allowed = (rng.random((n, m)) > 0.1).astype(np.uint8)
    initial = rng.uniform(0, 10, (m, m))
    initial[rng.random(initial.shape) < 0.5] = 1e18
    if scenario == 'saturado':
        g *= 1e18
        initial += 1e18
    elif scenario == 'empates':
        g.fill(0)
        pen.fill(0)
        bonus.fill(0)
        initial.fill(1e18)
    elif scenario == 'nan':
        initial.ravel()[::3] = np.nan
    valid = ~mask
    any_valid = valid.any(axis=1)
    lo = np.where(any_valid, valid.argmax(axis=1), m).astype(np.int16)
    hi = np.where(any_valid, m - 1 - valid[:, ::-1].argmax(axis=1), -1).astype(np.int16)
    gt = np.ascontiguousarray(g.transpose(0, 2, 3, 1))
    mt = np.ascontiguousarray(mask.transpose(0, 2, 3, 1)).view(np.uint8)
    outputs = []
    for kernel, geometry, masks in ((sparse, g, mask.view(np.uint8)), (transposed, gt, mt)):
        v = initial.copy()
        tmp = np.full((m, m), -1.0)
        back = np.full((steps, m, m), -1, dtype=np.int16)
        arrays = [geometry, ds, masks, pen, bonus, bad, allowed, lo, hi, v, tmp, back]
        if kernel is transposed:
            arrays.append(np.empty((m, m)))
        kernel(n, m, steps, *[x.ctypes.data for x in arrays])
        outputs.append((v, tmp, back))
    for reference, actual in zip(*outputs):
        # Comparación de bytes: también detecta diferencias en ceros con signo.
        assert reference.tobytes() == actual.tobytes()
