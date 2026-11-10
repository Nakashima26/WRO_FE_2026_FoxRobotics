"""Ray distances and hit identities agree with the NumPy reference."""
import numpy as np
import pytest
from pure_pursuit.twin.world import World
from pure_pursuit.wro_field import randomize

@pytest.mark.parametrize('seed',[1,7,19,3170839])
def test_native_cast_reference(seed):
    w=World(randomize(seed))
    assert w._cast_kernel is not None
    rng=np.random.default_rng(seed)
    origins=rng.uniform(-1600,1600,(1000,2))
    angles=rng.uniform(-np.pi,np.pi,1000)
    dirs=np.column_stack((np.sin(angles),np.cos(angles)))
    for limit in (0,240,1300,4000):
        a=w.cast(origins,dirs,limit)
        b=w._cast_python(origins,dirs,limit)
        np.testing.assert_allclose(a[0],b[0],rtol=1e-13,atol=1e-10)
        for i in (1,2,3):np.testing.assert_array_equal(a[i],b[i])
