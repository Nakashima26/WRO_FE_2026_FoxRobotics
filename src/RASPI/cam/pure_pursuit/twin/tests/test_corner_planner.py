"""corner_planner continuo: apex por side_pass; rojo trailing no abre."""
import pytest

from pure_pursuit import config as C
from pure_pursuit import corner_planner as cp
from pure_pursuit import digital_map as dmod
from pure_pursuit import wro_field as wf


@pytest.fixture(scope="module")
def field():
    return wf.randomize(3170839, start="cajon")


def test_suggest_s_to_w_not_open(field):
    """S (cajón) termina en rojo → apex adentro, no open."""
    pil = [(s.x, s.y, s.color) for s in field.signs]
    plan = cp.plan_corner(field.direction, "S", pil, parking=field.parking_section)
    assert plan.mode != "open"
    assert not plan.opens_out
    assert plan.apex_lat_mm > 0  # CW: adentro (+)
    assert plan.hold_dF_cm >= 42.0


def test_mid_green_not_full_open():
    """Verde a media pista → apex medio, no full-open."""
    from pure_pursuit.wro_field import SEATS, section_to_world
    # Asiento cerca del centro de carril en N (side_pass ≈ seat_r - 200 ≈ −120).
    h, w = SEATS["T2"]
    x, y = section_to_world("N", h, w)
    pil = [(float(x), float(y), "Green")]
    plan = cp.plan_corner("CW", "W", pil, parking="S")
    assert plan.apex_lat_mm < 0
    assert plan.apex_lat_mm > -180.0  # no full-open (−260)
    assert -150.0 <= plan.apex_lat_mm <= -40.0 or plan.mode == "mid"


def test_outer_green_opens():
    """Verde muy afuera → apex abierto (pero acotado)."""
    # side_inner ≤ −280 → apex −260
    assert cp.apex_from_side(-280.0) <= -250.0
    assert cp.apex_from_side(-120.0) == pytest.approx(-80.0)
    assert cp.apex_from_side(0.0) == pytest.approx(40.0)
    assert cp.apex_from_side(120.0) == pytest.approx(110.0)


def test_digital_map_uses_corner_plan(field, monkeypatch):
    monkeypatch.setattr(C, "CORNER_PLANNER", True, raising=False)
    monkeypatch.setattr(C, "DIGITAL_MAP_STEER", True, raising=False)
    dm = dmod.DigitalMap()
    dm.set_direction(field.direction)
    dm.confirmed = {(s.section, s.seat): s.color for s in field.signs}
    dm.in_stall = False
    dm._aligned = True
    dm.section = "S"
    dm.along_mm = 2100.0
    dm.lat_mm = 0.0
    dm.heading = dmod._heading("S", field.direction)
    dm.pose_xy = dmod._lane_pose("S", field.direction, 2100.0, 0.0)[:2]
    dm._dF = 90.0
    plan = dm.corner_plan()
    assert plan is not None
    assert not plan.opens_out
    assert dm._hold_cm() == plan.hold_dF_cm
    assert dm.holds_for_center() is True  # dF 90 > hold, along < release
    dm.along_mm = plan.release_along_mm + 10.0
    assert dm.holds_for_center() is False


def test_closed_plan_line_not_open_for_next_green(field, monkeypatch):
    """W termina en rojo: aunque N abre en verde, no knee a la pared."""
    monkeypatch.setattr(C, "CORNER_PLANNER", True, raising=False)
    monkeypatch.setattr(C, "DIGITAL_MAP_STEER", True, raising=False)
    dm = dmod.DigitalMap()
    dm.set_direction(field.direction)
    dm.confirmed = {(s.section, s.seat): s.color for s in field.signs}
    dm.parking = field.parking_section
    dm.in_stall = False
    dm._aligned = True
    dm.section = "W"
    dm.along_mm = 2000.0
    dm.lat_mm = 80.0
    dm.heading = dmod._heading("W", field.direction)
    dm.pose_xy = dmod._lane_pose("W", field.direction, 2000.0, 80.0)[:2]
    dm._dF = 100.0
    assert dm._left_pass("N") is not None
    pts = dm._build_line()
    assert pts
    lats = []
    for x, y in pts:
        a, lat = dm._section_frame(x, y, "W")
        if a >= 1800.0:
            lats.append(lat)
    assert lats, "sin puntos en cola de W"
    assert min(lats) > -120.0, f"línea abierta a exterior: min_lat={min(lats):.0f}"


def test_render_draws_line(field, monkeypatch):
    """El mapa del carro pinta la línea planificada."""
    monkeypatch.setattr(C, "CORNER_PLANNER", True, raising=False)
    monkeypatch.setattr(C, "DIGITAL_MAP_STEER", True, raising=False)
    dm = dmod.DigitalMap()
    dm.set_direction(field.direction)
    dm.confirmed = {(s.section, s.seat): s.color for s in field.signs}
    dm.in_stall = False
    dm._aligned = True
    dm.section = "S"
    dm.along_mm = 1800.0
    dm.lat_mm = 0.0
    dm.heading = dmod._heading("S", field.direction)
    dm.pose_xy = dmod._lane_pose("S", field.direction, 1800.0, 0.0)[:2]
    dm.line_world = dm._build_line()
    assert len(dm.line_world) >= 2
    img = dm.render(size=200)
    # BGR (40, 90, 220): naranja/rojo de la polilínea planificada
    mask = ((img[:, :, 0] < 80) & (img[:, :, 1] > 60) & (img[:, :, 1] < 140)
            & (img[:, :, 2] > 180))
    assert mask.any(), "línea no visible en render"
