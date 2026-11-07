"""salida_cajon: lado de paso de la 1ª lata adelante del cajón (isal=)."""
import pytest

from pure_pursuit import config as C
from pure_pursuit.salida_cajon import SalidaCajon

ACK_I = "ACK:V2,ang=0.00,est=I,dL=8,dR=80,px=0,py=0,yaw=0.00,tc=0"


def _foot(fwd_mm: float, right_mm: float, color: str):
    # Pie en la hoja BEV (sin las correcciones eje/pie, la ventana es ancha).
    return (C.ROBOT_BEV_X + right_mm / C.MM_PER_PX, C.ROBOT_BEV_Y - fwd_mm / C.MM_PER_PX, color)


@pytest.mark.parametrize("right,color,side", [
    (450.0, "Red", 2),      # isla a la derecha (CW), rojo: cruzar adentro
    (450.0, "Green", 1),
    (-450.0, "Green", 2),   # isla a la izquierda (CCW), verde: adentro
    (-450.0, "Red", 1),
])
def test_lado(right, color, side):
    s = SalidaCajon()
    for _ in range(3):
        s.update(ACK_I, [_foot(300.0, right, color)])
    assert s.side == side


def test_necesita_votos_y_solo_en_inicio():
    s = SalidaCajon()
    s.update(ACK_I, [_foot(300.0, 450.0, "Red")])
    s.update(ACK_I, [_foot(300.0, 450.0, "Red")])
    assert s.side == 0
    s2 = SalidaCajon()
    for _ in range(5):
        s2.update(ACK_I.replace("est=I", "est=S"), [_foot(300.0, 450.0, "Red")])
    assert s2.side == 0


def test_lata_lejos_no_decide():
    # T1 en CW (~94 cm adelante): la cruza el PP, no la S.
    s = SalidaCajon()
    for _ in range(5):
        s.update(ACK_I, [_foot(800.0, 450.0, "Red")])
    assert s.side == 0
