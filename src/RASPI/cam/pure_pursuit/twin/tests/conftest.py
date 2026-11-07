"""Los tests de runtime/twin están afinados a FPS_NOMINAL (14); el default de config
es 40 (cámara de la Pi 5), así que se fija aquí para que no cambien las ventanas."""
import pytest

from pure_pursuit import config as C


@pytest.fixture(autouse=True)
def _pi_fps_nominal(monkeypatch):
    monkeypatch.setattr(C, "PI_FPS", C.FPS_NOMINAL)
