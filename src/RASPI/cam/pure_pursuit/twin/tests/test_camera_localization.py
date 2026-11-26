import math

import numpy as np

from pure_pursuit import config as C
from pure_pursuit.camera_localization import wall_edges
from pure_pursuit.localizer import Localizer
from pure_pursuit.twin.camera import CameraModel
from pure_pursuit.twin.params import TwinParams
from pure_pursuit.wro_field import OUTER_HALF_MM, randomize


def test_rendered_wall_corrects_position_and_heading(monkeypatch):
    monkeypatch.setattr(C, "LOC_CAM_WALL", True)
    cam, bev, _, _ = CameraModel.twin_camera_and_bev(TwinParams())
    field = randomize(2)
    field.signs.clear()
    field.barriers.clear()
    x = OUTER_HALF_MM - 240
    walls = wall_edges(cam.render_camera(field, x, 0, 6), bev)
    loc = Localizer({}, None, x - 75, 0, 10)
    loc.P = np.diag([70**2, 70**2, math.radians(4)**2])

    assert loc.update_camera(walls, (), {}) == (True, False)
    assert x - 75 < loc.x[0] <= x - 45 + 1e-9
    assert 9 - 1e-9 <= math.degrees(loc.x[2]) < 10
    assert loc.n_cam_wall == 1


def test_sign_requires_confirmed_matching_color_and_plausible_innovation(monkeypatch):
    monkeypatch.setattr(C, "LOC_CAM_SIGN", True)
    sign = randomize(1).signs[0]
    loc = Localizer({}, None, sign.x + 50, sign.y - 260, 0)
    other_color = "Red" if sign.color == "Green" else "Green"
    key = (sign.section, sign.seat)

    for signs, confirmed in [
        ([(260, 0, sign.color)], {}),
        ([(260, 0, other_color)], {key: sign.color}),
        ([(650, 0, sign.color)], {key: sign.color}),
    ]:
        assert loc.update_camera((), signs, confirmed) == (False, False)
    assert loc.update_camera((), [(260, 0, sign.color)], {key: sign.color}) == (False, True)
    assert sign.x < loc.x[0] < sign.x + 50
