"""Juez de la señal especial: cruce del eje trasero, como las métricas del twin."""
import math
from ..wro_field import _heading


class SurpriseJudge:
    def __init__(self, sign, direction):
        self.sign = sign
        h = math.radians(_heading(sign.section, direction))
        self.sh, self.ch = math.sin(h), math.cos(h)
        self.previous = None
        self.last_turns = None
        self.passages = []
        self.failed = False

    def update(self, x, y, turns, t):
        dx, dy = x-self.sign.x, y-self.sign.y
        along = dx*self.sh+dy*self.ch
        lateral = dx*self.ch-dy*self.sh
        previous, self.previous = self.previous, along
        if (self.failed or previous is None or not previous < 0 <= along
                or abs(lateral) >= 450 or len(self.passages) >= 3
                or (self.last_turns is not None and turns-self.last_turns < 3)):
            return None
        # Cerca del límite de vuelta el ACK puede ir un giro adelantado o
        # atrasado: no agrupar por tc//4 (perdería pasos válidos).
        self.last_turns = turns
        correct = lateral > 0 if self.sign.driving_color == "Red" else lateral < 0
        event = dict(t=t, passage=len(self.passages)+1, turns=turns, correct=correct, visible=self.sign.color,
                     original=self.sign.driving_color, lateral_mm=lateral)
        self.passages.append(event)
        self.failed = not correct
        return event

    def report(self):
        return {"surprise_passages": list(self.passages),
                "surprise_points": 5*sum(p["correct"] for p in self.passages),
                "surprise_wrong_side": self.failed}
