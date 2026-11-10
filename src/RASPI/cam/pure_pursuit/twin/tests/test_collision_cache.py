"""Cached collision geometry must preserve causes and live Field edits."""
import copy
import random
from types import SimpleNamespace
import pytest
from pure_pursuit.twin import world as W
from pure_pursuit.wro_field import randomize
World = W.World
body_corners = W.body_corners
SIGN_MM = W.SIGN_MM
_point_outside_field = W._point_outside_field
_point_in_island = W._point_in_island
_bbox = W._bbox
_bbox_apart = W._bbox_apart
_inside_rect = W._inside_rect
_segments_cross = W._segments_cross
_hits_polygon = W._hits_polygon

def reference_collision(
    world: World,
    x: float,
    y: float,
    heading_deg: float,
    length_mm: float,
    width_mm: float,
    rear_overhang_mm: float,
    ignore: tuple[str, ...] = (),
) -> str | None:
    """Primera causa de contacto, saltando las de `ignore` (p. ej. "pared exterior")."""
    corners = body_corners(x, y, heading_deg, length_mm, width_mm, rear_overhang_mm)
    corner_list = [tuple(c) for c in corners]
    for cx, cy in corner_list:
        if _point_outside_field(cx, cy) and "pared exterior" not in ignore:
            return "pared exterior"
        if _point_in_island(cx, cy):
            return "isla"
    edges = list(zip(corner_list, corner_list[1:] + corner_list[:1]))
    car_box = _bbox(corner_list)
    track = world.field
    polys = (track.inner,) if "pared exterior" in ignore else (track.outer, track.inner)
    for poly in polys:
        wall =[(float(p[0]), float(p[1])) for p in poly]
        wall_box = _bbox(wall)
        # Carro lejos de la isla, o bien adentro del tapete: no hay cruce.
        if _bbox_apart(car_box, wall_box) or _inside_rect(car_box, wall):
            continue
        wall_edges = list(zip(wall, wall[1:] + wall[:1]))
        for a, b in edges:
            for c, d in wall_edges:
                if _segments_cross(a, b, c, d):
                    return "pared"
    half = SIGN_MM / 2.0
    for sign in track.signs:
        box = [
            (sign.x - half, sign.y - half),
            (sign.x + half, sign.y - half),
            (sign.x + half, sign.y + half),
            (sign.x - half, sign.y + half),
        ]
        if _bbox_apart(car_box, _bbox(box)):
            continue
        if _hits_polygon(corner_list, box):
            return f"señal {sign.color} {sign.section}/{sign.seat}"
    for box in track.barriers:
        if _bbox_apart(car_box, _bbox(box)):
            continue
        if _hits_polygon(corner_list, box):
            return "cajón magenta"
    return None


@pytest.mark.parametrize("seed", [1, 7, 19, 3170839])
@pytest.mark.parametrize("ignore", [(), ("pared exterior",)])
def test_collision_cache_matches_reference(seed, ignore):
    world = W.World(randomize(seed))
    rng = random.Random(seed)
    poses = [(rng.uniform(-1550,1550), rng.uniform(-1550,1550), rng.uniform(-360,720)) for _ in range(1000)]
    # Exact edges plus nearby offsets exercise strict point/segment comparisons.
    poses += [(x,y,h) for x in (-1500,-500,0,500,1500) for y in (-1500,-500,0,500,1500) for h in (0,45,90)]
    for pose in poses:
        assert W.collision(world,*pose,176,110,35,ignore) == reference_collision(world,*pose,176,110,35,ignore)


def test_collision_cache_tracks_live_mutations_and_new_field():
    world = W.World(randomize(19))
    def check():
        rng = random.Random(9)
        for _ in range(300):
            pose = (rng.uniform(-1500,1500),rng.uniform(-1500,1500),rng.uniform(0,360))
            assert W.collision(world,*pose,176,110,35) == reference_collision(world,*pose,176,110,35)
        for sign in world.field.signs:
            assert W.collision(world,sign.x,sign.y,0,176,110,35) == reference_collision(world,sign.x,sign.y,0,176,110,35)
    check()
    world.field.signs[0].x += 150
    world.field.signs[0].color = "Changed"
    check()
    world.field.signs.pop()
    check()
    world.field.barriers[0][0] = (1000,1200)
    check()
    world.field.signs.clear()
    world.field.barriers.clear()
    check()
    world.field = randomize(7)
    check()


def test_collision_world_new_and_custom_dynamic_walls():
    field = randomize(1)
    custom = SimpleNamespace(outer=field.outer,inner=field.inner,signs=copy.deepcopy(field.signs),barriers=copy.deepcopy(field.barriers))
    world = W.World.__new__(W.World)
    world.field = custom
    for inner in [custom.inner, [(x+750,y+750) for x,y in custom.inner]]:
        custom.inner = inner
        for pose in [(600,650,45),(1000,1000,0),(-1000,-1000,90)]:
            assert W.collision(world,*pose,176,110,35) == reference_collision(world,*pose,176,110,35)
    assert not hasattr(world,"_collision_cache")
