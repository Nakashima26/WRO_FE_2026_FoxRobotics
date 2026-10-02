"""
Odometría Pi: frame ODOM (x adelante al arranque, y izquierda, yaw CCW+ en el eje trasero).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from . import config as C


def parse_ack(ack: str) -> dict[str, str]:
    """Pares k=v tras 'ACK:V2,' (o desde el inicio si ya viene recortado)."""
    if not ack:
        return {}
    i = ack.find("ACK:V2,")
    tail = ack[i + len("ACK:V2,") :] if i >= 0 else ack
    if tail.startswith("V2,"):
        tail = tail[3:]
    out: dict[str, str] = {}
    for part in tail.split(","):
        part = part.strip()
        if not part or "=" not in part:
            continue
        k, v = part.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def _f(fields: dict[str, str], key: str) -> float | None:
    v = fields.get(key)
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def _i(fields: dict[str, str], key: str) -> int | None:
    v = fields.get(key)
    if v is None or v == "":
        return None
    try:
        return int(v)
    except ValueError:
        return None


def _norm_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def _unwrap_delta_deg(prev: float, cur: float) -> float:
    return _norm_deg(cur - prev)


@dataclass(frozen=True)
class OdomPose:
    x_mm: float
    y_mm: float
    yaw_deg: float
    t: float
    source: str
    quality: str


@dataclass
class _LegacyState:
    base_yaw: float = 0.0
    tc: int = 0
    turn_dir: str | None = None
    last_ang: float = 0.0
    in_giro: bool = False


class Odometry:
    """Integra ACK del ESP en poses ODOM continuas."""

    def __init__(self) -> None:
        self.pose: OdomPose | None = None
        self._last_od: float | None = None
        self._last_yaw: float | None = None
        self._legacy = _LegacyState()
        self._last_t: float | None = None

    @staticmethod
    def delta(prev: OdomPose, cur: OdomPose) -> tuple[float, float, float]:
        """(dx_adelante, dy_izquierda, dyaw) en el frame del robot previo."""
        dyaw = math.radians(_unwrap_delta_deg(prev.yaw_deg, cur.yaw_deg))
        dx_w = cur.x_mm - prev.x_mm
        dy_w = cur.y_mm - prev.y_mm
        c = math.cos(-math.radians(prev.yaw_deg))
        s = math.sin(-math.radians(prev.yaw_deg))
        dx_fwd = c * dx_w - s * dy_w
        dy_left = s * dx_w + c * dy_w
        return dx_fwd, dy_left, math.degrees(dyaw)

    def update(
        self,
        fields: dict[str, str],
        t: float,
        assumed_speed_mms: float,
    ) -> OdomPose:
        dt = 0.05
        if self._last_t is not None and t > self._last_t:
            dt = min(0.5, t - self._last_t)
        self._last_t = t

        px = _f(fields, "px")
        py = _f(fields, "py")
        yaw_cont = _f(fields, "yaw")
        ang = _f(fields, "ang")
        od = _f(fields, "od")

        if px is not None and py is not None and yaw_cont is not None:
            pose = OdomPose(px, py, yaw_cont, t, "esp_pose", "high")
            self._stash_od_yaw(od, yaw_cont)
            self.pose = pose
            return pose

        if od is not None and yaw_cont is not None:
            x, y = self._integrate_od(od, yaw_cont, dt, assumed_speed_mms)
            pose = OdomPose(x, y, yaw_cont, t, "od_yaw", "medium")
            self.pose = pose
            return pose

        if yaw_cont is not None:
            x, y = self._integrate_yaw_only(yaw_cont, dt, assumed_speed_mms)
            pose = OdomPose(x, y, yaw_cont, t, "yaw_assumed", "medium")
            self.pose = pose
            return pose

        pose = self._legacy_update(fields, t, dt, assumed_speed_mms)
        self.pose = pose
        return pose

    def _stash_od_yaw(self, od: float | None, yaw: float) -> None:
        if od is not None:
            self._last_od = od
        self._last_yaw = yaw

    def _integrate_od(
        self,
        od: float,
        yaw_deg: float,
        dt: float,
        assumed_speed_mms: float,
    ) -> tuple[float, float]:
        if self.pose is None:
            self._last_od = od
            return 0.0, 0.0
        if self._last_od is None:
            self._last_od = od
            ds = assumed_speed_mms * dt
        else:
            ds = od - self._last_od
            self._last_od = od
        mid = math.radians((self.pose.yaw_deg + yaw_deg) * 0.5)
        dx = ds * math.cos(mid)
        dy = ds * math.sin(mid)
        return self.pose.x_mm + dx, self.pose.y_mm + dy

    def _integrate_yaw_only(
        self,
        yaw_deg: float,
        dt: float,
        assumed_speed_mms: float,
    ) -> tuple[float, float]:
        if self.pose is None:
            return 0.0, 0.0
        ds = assumed_speed_mms * dt
        mid = math.radians((self.pose.yaw_deg + yaw_deg) * 0.5)
        dx = ds * math.cos(mid)
        dy = ds * math.sin(mid)
        return self.pose.x_mm + dx, self.pose.y_mm + dy

    def _legacy_update(
        self,
        fields: dict[str, str],
        t: float,
        dt: float,
        assumed_speed_mms: float,
    ) -> OdomPose:
        ang = _f(fields, "ang")
        est = fields.get("est")
        tc = _i(fields, "tc")
        direccion = fields.get("dir")

        if ang is None:
            ang = self._legacy.last_ang
        if tc is None:
            tc = self._legacy.tc

        if tc > self._legacy.tc:
            sign = self._turn_sign(direccion, ang, est)
            self._legacy.base_yaw += sign * 90.0
            self._legacy.tc = tc

        if direccion in ("L", "R"):
            self._legacy.turn_dir = direccion

        in_g = est == "G"
        if in_g and not self._legacy.in_giro:
            self._legacy.last_ang = ang
        self._legacy.in_giro = in_g

        yaw = self._legacy.base_yaw + ang
        if self.pose is None:
            x, y = 0.0, 0.0
        elif in_g:
            x, y = self.pose.x_mm, self.pose.y_mm
        else:
            ds = assumed_speed_mms * dt
            mid = math.radians((self.pose.yaw_deg + yaw) * 0.5)
            x = self.pose.x_mm + ds * math.cos(mid)
            y = self.pose.y_mm + ds * math.sin(mid)

        self._legacy.last_ang = ang
        return OdomPose(x, y, yaw, t, "legacy", "low")

    def _turn_sign(self, direccion: str | None, ang: float, est: str | None) -> float:
        if direccion == "L":
            return 1.0
        if direccion == "R":
            return -1.0
        if est == "G" and ang != 0.0:
            return 1.0 if ang > 0 else -1.0
        if self._legacy.turn_dir == "L":
            return 1.0
        if self._legacy.turn_dir == "R":
            return -1.0
        return 1.0


def fields_from_ack_line(ack: str) -> dict[str, str]:
    """Atajo para tests."""
    return parse_ack(ack)
