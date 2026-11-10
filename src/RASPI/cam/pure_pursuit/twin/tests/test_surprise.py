"""Altura desde imágenes y puntuación del lado original, sin verdad al detector."""
import math
import numpy as np
import pytest
from vision import Vision
from pure_pursuit import config as C
from pure_pursuit.surprise import SurpriseInterpreter, estimate_height
from pure_pursuit.wro_field import Sign, randomize, surprise_candidates, with_surprise, _heading
from pure_pursuit.twin.camera import CameraModel
from pure_pursuit.twin.params import TwinParams
from pure_pursuit.twin.surprise import SurpriseJudge


@pytest.mark.parametrize('direction', ['CW', 'CCW'])
@pytest.mark.parametrize('visible', ['Red', 'Green'])
@pytest.mark.parametrize('height', [50, 100, 150])
def test_rendered_height_and_driving_color(direction, visible, height):
    params = TwinParams(); cam = CameraModel(params.camera, params); bev, _ = cam.build_bev()
    field = randomize(1); sign = Sign(0, 1000, visible, 'N', 'X1', height_mm=height)
    field.signs = [sign]; field.barriers = []
    vis = Vision(open_cam=False); vis.min_contour_area = C.SURPRISE_MIN_CONTOUR_AREA
    interpreter = SurpriseInterpreter()
    heading = _heading('N', direction); a = math.radians(heading)
    for distance in range(1100, 499, -25):
        image = cam.render_camera(field, sign.x-distance*math.sin(a), sign.y-distance*math.cos(a), heading)
        _, positions = vis.detect_on(image)
        assert len(positions[visible]) == 1, (height, visible, distance, positions)
        for _ in range(C.SURPRISE_CONFIRM_FRAMES):
            logical = interpreter.interpret(positions, bev, image.shape)
        expected = visible if height == 100 else ('Green' if visible == 'Red' else 'Red')
        assert len(logical[expected]) == 1, interpreter.observations
        assert interpreter.observations[0]['visible'] == visible
        assert interpreter.observations[0]['confirmed_mm'] == height


@pytest.mark.parametrize('height', [50, 100, 150])
def test_metric_height_from_vertical_projection(height):
    params = TwinParams(); cam = CameraModel(params.camera, params); bev, _ = cam.build_bev()
    for distance in (900, 1200, 1500):
        points = np.array([[0, distance, -params.camera.height_mm], [0, distance, height-params.camera.height_mm]])
        bottom, top = cam._project(points @ cam._robot_to_cam.T)
        box = (bottom[0]-10, top[1], 21, bottom[1]-top[1]+1)
        z = estimate_height(bev, box, (480,640,3), camera_ground=(params.camera.right_mm, params.camera.forward_from_rear_axle_mm-params.vehicle.wheelbase_mm))
        assert z == pytest.approx(height, abs=0.01)


def test_clipped_unknown_is_not_published_as_normal():
    p = TwinParams(); cam=CameraModel(p.camera,p); bev,_=cam.build_bev(); interpreter=SurpriseInterpreter()
    for _ in range(5):
        out=interpreter.interpret({'Red':[(0,100,50,50)],'Green':[]},bev,(480,640,3))
        assert not out['Red'] and not out['Green']


@pytest.mark.parametrize('seed', range(1,25))
def test_randomized_special_is_eligible_and_preserves_layout(seed):
    base=randomize(seed, start='cajon'); eligible=surprise_candidates(base)
    assert eligible
    for height in (50,150):
        modified=with_surprise(base,eligible[-1],height)
        assert [(s.x,s.y,s.seat,s.section) for s in modified.signs] == [(s.x,s.y,s.seat,s.section) for s in base.signs]
        assert sum(s.original_color is not None for s in modified.signs)==1
        for a,b in zip(base.signs,modified.signs): assert b.driving_color==a.color
        assert base.signs[eligible[-1]].original_color is None
    first=next(i for i in range(len(base.signs)) if i not in eligible)
    with pytest.raises(ValueError): with_surprise(base,first,50)
    with pytest.raises(ValueError): with_surprise(base,eligible[0],100)


@pytest.mark.parametrize('direction',['CW','CCW'])
@pytest.mark.parametrize('original',['Red','Green'])
def test_judge_uses_original_color_retains_points_and_stops_scoring(direction,original):
    sign=Sign(0,0,'Green' if original=='Red' else 'Red','N','X1',50,original)
    judge=SurpriseJudge(sign,direction); h=math.radians(_heading('N',direction)); sh,ch=math.sin(h),math.cos(h)
    def position(along,lat): return along*sh+lat*ch,along*ch-lat*sh
    lat=100 if original=='Red' else -100
    for lap in range(3):
        judge.update(*position(-10,lat),lap*4,lap*10)
        event=judge.update(*position(10,lat),lap*4,lap*10+1)
        assert event['correct']
        judge.update(*position(-10,lat),lap*4,lap*10+2)
        assert judge.update(*position(10,lat),lap*4,lap*10+3) is None
    assert judge.report()['surprise_points']==15
    judge=SurpriseJudge(sign,direction)
    judge.update(*position(-10,lat),0,0); judge.update(*position(10,lat),0,1)
    judge.update(*position(-10,-lat),4,10); event=judge.update(*position(10,-lat),4,11)
    assert not event['correct'] and judge.failed and judge.report()['surprise_points']==5
    judge.update(*position(-10,lat),8,20)
    assert judge.update(*position(10,lat),8,21) is None


def test_seat_memory_survives_lap_and_go_clears_it():
    from types import SimpleNamespace
    from pure_pursuit.wro_field import section_to_world, SEATS
    p=TwinParams(); cam=CameraModel(p.camera,p); bev,_=cam.build_bev(); field=randomize(1)
    sx,sy=section_to_world('N',*SEATS['X1'])
    field.signs=[Sign(sx,sy,'Green','N','X1',50)]; field.barriers=[]
    vision=Vision(open_cam=False); vision.min_contour_area=200
    interpreter=SurpriseInterpreter()
    dm=SimpleNamespace(direction='CW',pose_xy=(sx-900,sy),_view_heading=90)
    image=cam.render_camera(field,*dm.pose_xy,90); _,positions=vision.detect_on(image)
    for _ in range(3):out=interpreter.interpret(positions,bev,image.shape,dm)
    assert out['Red'] and interpreter.seats[('N','X1')]==('Green',50)
    for _ in range(8):interpreter.interpret({'Red':[],'Green':[]},bev,image.shape,dm)
    # Nueva vuelta: la clasificación vuelve en el primer cuadro por asiento.
    assert interpreter.interpret(positions,bev,image.shape,dm)['Red']
    interpreter.reset()
    assert not interpreter.seats
    assert not interpreter.interpret(positions,bev,image.shape,dm)['Red']


def test_height_needs_calibration_and_runtime_rejects_color_only_roi(monkeypatch):
    from pure_pursuit.bev import BEVTransformer
    from pure_pursuit.runtime_nuevo import PPConfig,PPRuntime
    from pure_pursuit.twin.tests.test_runtime_step import FakeSerialLink
    bev=BEVTransformer(calib_path=__import__('pathlib').Path('/nonexistent-surprise.npz'))
    assert estimate_height(bev,(100,100,50,100),(480,640,3)) is None
    monkeypatch.setattr(C,'SURPRISE_RULE',True)
    with pytest.raises(ValueError,match='calibración'):
        PPRuntime(PPConfig(show_window=False),vision=Vision(open_cam=False),serial_link=FakeSerialLink(),bev=bev)
    p=TwinParams(); cam=CameraModel(p.camera,p); bev,_=cam.build_bev()
    monkeypatch.setattr(C,'ROUTE_SEAT_ROI',True)
    with pytest.raises(ValueError,match='ROUTE_SEAT_ROI'):
        PPRuntime(PPConfig(show_window=False),vision=Vision(open_cam=False),serial_link=FakeSerialLink(),bev=bev)


def test_judge_counter_boundary_does_not_drop_second_pass():
    sign=Sign(0,0,'Green','N','X1',50,'Red'); judge=SurpriseJudge(sign,'CW')
    for turns in (4,7,11):
        judge.update(-10,-100,turns,turns)
        assert judge.update(10,-100,turns,turns+0.1)['correct']
    assert judge.report()['surprise_points']==15


def test_empirical_normal_pixel_curve_and_no_extrapolation(monkeypatch):
    from pure_pursuit.calibrar_sorpresa import build_reference
    p=TwinParams(); cam=CameraModel(p.camera,p); bev,_=cam.build_bev(); field=randomize(1)
    field.signs=[Sign(0,1000,'Green','N','X1',100)]; field.barriers=[]
    frames=[cam.render_camera(field,-d,1000,90) for d in (500,700,900,1100,1300)]
    table=build_reference(frames,bev)['SURPRISE_NORMAL_HEIGHT_SAMPLES']
    monkeypatch.setattr(C,'SURPRISE_NORMAL_HEIGHT_SAMPLES',table)
    interpreter=SurpriseInterpreter()
    vis=Vision(open_cam=False); vis.min_contour_area=200
    for height in (50,100,150):
        field.signs=[Sign(0,1000,'Green','N','X1',height)]
        image=cam.render_camera(field,-1100,1000,90); _,pos=vis.detect_on(image)
        interpreter.reset()
        for _ in range(3):out=interpreter.interpret(pos,bev,image.shape,image=image)
        assert out['Green' if height==100 else 'Red']
    with pytest.raises(ValueError):
        monkeypatch.setattr(C,'SURPRISE_NORMAL_HEIGHT_SAMPLES',[(500,50),(300,70)])
        SurpriseInterpreter()


def test_occluded_special_needs_ground_support_and_preserves_known_class():
    p=TwinParams(); cam=CameraModel(p.camera,p); bev,_=cam.build_bev();field=randomize(1)
    field.signs=[Sign(0,1000,'Green','N','X1',50)];field.barriers=[]
    image=cam.render_camera(field,-900,1000,90);vis=Vision(open_cam=False);vis.min_contour_area=200
    _,pos=vis.detect_on(image);interpreter=SurpriseInterpreter()
    # La imagen sin piso no aporta prueba para una NUEVA clasificación.
    occluded=np.zeros_like(image)
    for _ in range(4):assert not interpreter.interpret(pos,bev,image.shape,image=occluded)['Red']
    for _ in range(3):out=interpreter.interpret(pos,bev,image.shape,image=image)
    assert out['Red']
    assert interpreter.interpret(pos,bev,image.shape,image=occluded)['Red']


def test_lost_foot_uses_existing_obstacle_memory():
    from pure_pursuit.obstacle_memory import ObstacleMemory
    p=TwinParams();cam=CameraModel(p.camera,p);bev,_=cam.build_bev();interpreter=SurpriseInterpreter()
    # Ya se sabe que la señal verde es especial: la memoria guarda ROJO de paso.
    memory=ObstacleMemory()
    memory.update([(220,220,'Red')],0,0)
    for _ in range(3):
        positions=interpreter.interpret({'Green':[(200,400,60,80)],'Red':[]},bev,(480,640,3))
        assert not positions['Red'] and not positions['Green']
        retained=memory.update([],0.025,0)
        assert any(o[2]=='Red' for o in retained)
