"""Localización del visor durante las maniobras del ESP."""

from types import SimpleNamespace

import numpy as np
import pytest

from pure_pursuit import config as C
from pure_pursuit.digital_map import DigitalMap
from pure_pursuit.localizer import Localizer
from pure_pursuit.route_mode import RouteMode
from pure_pursuit.runtime_nuevo import PPRuntime


@pytest.mark.parametrize(
    "x,y,heading,reading,axis",
    [
        (-1380.0, 0.0, 0.0, "tL=75", 0),      # pared lateral W
        (-1210.0, -600.6, 0.0, "tL=45", 0),  # cara interior de madera W
        (-1200.0, -600.0, 270.0, "tF=180", 0),  # frontal Long contra el fondo
    ],
)
def test_maneuver_corrects_position(x, y, heading, reading, axis):
    loc = Localizer({}, "W", x, y, heading, park_w=1000.0)
    loc.update_maneuver(f"ACK:V2,est=E,px=0,py=0,yaw=0,{reading}")

    assert loc.n_used == 1
    assert loc.pose()[axis] < (x, y)[axis]
    assert loc.pose()[2] == pytest.approx(heading)


@pytest.mark.parametrize(
    "x,y,heading,reading",
    [
        (-1210.0, -592.0, 0.0, "tL=45"),  # el cono cruza la arista de madera
        (-1350.0, 0.0, 0.0, "tL=20"),    # innovación mayor que el límite
        (-1300.0, 0.0, 45.0, "tL=160"),  # incidencia oblicua
    ],
)
def test_maneuver_rejects_ambiguous_echo(x, y, heading, reading):
    loc = Localizer({}, "W", x, y, heading, park_w=1000.0)
    loc.update_maneuver(f"ACK:V2,est=E,px=0,py=0,yaw=0,{reading}")

    assert loc.n_used == 0
    assert loc.pose() == pytest.approx((x, y, heading))


def test_maneuver_view_survives_missing_ack_and_keeps_control_pose(monkeypatch):
    monkeypatch.setattr(C, "ROUTE_FOLLOW", True)
    monkeypatch.setattr(C, "LOC_MANEUVER_VIEW", True)
    route = RouteMode()
    route.fixed = True
    route.loc = Localizer({}, "W", -1380.0, 0.0, 0.0, park_w=1000.0)
    route.loc.update("ACK:V2,est=S,px=0,py=0,yaw=0", correct=False)
    original = route.loc.transform()
    ack = "ACK:V2,est=G,px=1,py=0,yaw=0,tL=75"
    assert route.step(None, "G", ack, armed=True) == (None, "", None)
    view = route.maneuver_loc
    assert view.n_used == 1
    x, y, heading = view.pose()
    assert view.pose("ACK:V2,est=G,px=4,py=0,yaw=-1") == pytest.approx((x, y + 3, heading + 1))
    assert view.pose() == (x, y, heading)

    route.step(None, "G", None, armed=True)
    assert route.maneuver_loc is view
    assert route.loc.transform() == original

    route.step(None, None, "ACK:V2,est=S", armed=True)
    assert route.maneuver_loc is None

    monkeypatch.setattr(C, "LOC_MANEUVER_VIEW", False)
    route.step(None, "G", ack, armed=True)
    assert route.maneuver_loc is None


def test_known_map_draws_maneuver_view_without_moving_map_state(monkeypatch):
    dm = DigitalMap()
    original = dm.pose_xy
    route = RouteMode()
    route.maneuver_loc = Localizer({}, "W", -1380.0, 0.0, 0.0, park_w=1000.0)
    route.maneuver_loc.update("ACK:V2,px=0,py=0,yaw=0", correct=False)
    ack = "ACK:V2,est=E,px=25,py=0,yaw=0"
    runtime = SimpleNamespace(_rmode=route, digital=dm, _prev_ack=ack)
    render = dm.render
    calls = []

    def record_render(**kwargs):
        calls.append(kwargs)
        return render(**kwargs)

    monkeypatch.setattr(dm, "render", record_render)

    actual = PPRuntime._render_digital_map(runtime)
    expected = render(**(calls[0] | {"view_pose": (-1380.0, 25.0, 0.0)}))
    np.testing.assert_array_equal(actual, expected)
    assert dm.pose_xy == original
