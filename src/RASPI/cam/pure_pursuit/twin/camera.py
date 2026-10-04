"""Render de cámara 640×480 y homografía BEV (copia de chassis_twin, parametrizada)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .. import config as C
from ..bev import BEVTransformer
from ..centerline import map_obstacle_to_bev
from .params import CameraParams, TwinParams
from ..wro_field import (
    INNER_HALF_MM,
    LINE_MM,
    OUTER_HALF_MM,
    SIGN_MM,
    blue_segments,
    orange_segments,
)
from .world import robot_to_world

# Paleta id: 0 cielo, 1 piso, 2 pared, 3 rojo, 4 verde, 5 magenta

# Largo de cada tramo de pared para el orden del pintor (ver render_camera).
_WALL_TILE_MM = 100.0


def _hsv_bgr(h, s, v):
    pix = np.uint8([[[h, s, v]]])
    b, g, r = (int(c) for c in cv2.cvtColor(pix, cv2.COLOR_HSV2BGR)[0, 0])
    return (b, g, r)


FLOOR_BGR = (175, 198, 213)
WALL_BGR = (0, 0, 0)
# La cinta real deja píxeles con S >= 140 (LINE_CORE_HSV, medido en orillas820);
# con S=115 la Pi rechazaba toda la naranja del twin por falta de núcleo.
ORANGE_BGR = _hsv_bgr(9, 150, 190)
BLUE_BGR = _hsv_bgr(122, 152, 180)
_RED = np.array([55, 39, 238], dtype=np.float32)
RED_BGR = tuple(int(c) for c in np.round(_RED * (230.0 / 238.0)))
GREEN_BGR = (44, 214, 68)
MAGENTA_BGR = (255, 0, 255)
SKY_BGR = (48, 48, 48)

_PALETTE = np.array(
    [SKY_BGR, FLOOR_BGR, WALL_BGR, RED_BGR, GREEN_BGR, MAGENTA_BGR],
    dtype=np.uint8,
)

# Calibración antigua 4 puntos (bev_calib.npz viejo)
OLD_CALIB_4_MM = np.float32(
    [
        [-60.0, 200.0],
        [60.0, 200.0],
        [-150.0, 380.0],
        [150.0, 380.0],
    ]
)


@dataclass
class CameraModel:
    params: CameraParams
    vehicle: TwinParams

    _dirs_robot: np.ndarray | None = None
    _cam_to_robot: np.ndarray | None = None
    _robot_to_cam: np.ndarray | None = None
    _fx: float = 0.0
    _fy: float = 0.0
    _cx: float = 0.0
    _cy: float = 0.0
    _uu: np.ndarray | None = None
    _vv: np.ndarray | None = None

    def __post_init__(self) -> None:
        self._build_intrinsics()

    def _build_intrinsics(self) -> None:
        p = self.params
        half = math.radians(p.hfov_deg) / 2.0
        # Pinhole: el borde del ancho es tan(hfov/2). Equidistante: r = f·θ,
        # el mismo borde es hfov/2 en radianes. A 120° y 45° de tilt el
        # horizonte cae en la fila 0 y el piso ocupa las 480.
        if p.equidistant:
            self._fx = (p.width / 2.0) / half
        else:
            self._fx = (p.width / 2.0) / math.tan(half)
        self._fy = self._fx
        self._cx = p.width / 2.0
        self._cy = p.height / 2.0
        if p.rotation_cam_to_robot is not None:
            self._cam_to_robot = np.asarray(p.rotation_cam_to_robot, dtype=np.float64).reshape(3, 3)
        else:
            tilt = math.radians(p.tilt_deg)
            st, ct = math.sin(tilt), math.cos(tilt)
            self._cam_to_robot = np.array(
                [
                    [1.0, 0.0, 0.0],
                    [0.0, -st, ct],
                    [0.0, -ct, -st],
                ],
                dtype=np.float64,
            )
        self._robot_to_cam = self._cam_to_robot.T
        uu, vv = np.meshgrid(
            np.arange(p.width, dtype=np.float64),
            np.arange(p.height, dtype=np.float64),
        )
        self._uu, self._vv = uu, vv
        dx = (uu - self._cx) / self._fx
        dy = (vv - self._cy) / self._fy
        if p.equidistant:
            theta = np.hypot(dx, dy)
            scale = np.ones_like(theta)
            nz = theta > 1e-8
            scale[nz] = np.sin(theta[nz]) / theta[nz]
            dirs_cam = np.stack([scale * dx, scale * dy, np.cos(theta)], axis=-1)
        else:
            dirs_cam = np.stack([dx, dy, np.ones_like(uu)], axis=-1)
        self._dirs_robot = dirs_cam @ self._cam_to_robot.T
        # En el frame de la cámara: la componente vertical de cada rayo y el
        # factor con el que la distancia al piso se reparte en right/adelante.
        # No dependen de la pose del carro, así que se calculan una sola vez.
        dz = self._dirs_robot[..., 2]
        self._floor_down = dz < -1e-4
        self._floor_scale = np.zeros(dz.shape, dtype=np.float32)
        self._floor_scale[self._floor_down] = (
            -1.0 / dz[self._floor_down]).astype(np.float32)
        self._ray_right = self._dirs_robot[..., 0].astype(np.float32)
        self._ray_fwd = self._dirs_robot[..., 1].astype(np.float32)

    def _project(self, cam_pts: np.ndarray) -> np.ndarray:
        """Puntos en el frame de la cámara (z adelante) → píxeles."""
        x = cam_pts[:, 0]
        y = cam_pts[:, 1]
        z = cam_pts[:, 2]
        if self.params.equidistant:
            theta = np.arctan2(np.hypot(x, y), z)
            radius = self._fx * theta
            rho = np.hypot(x, y)
            scale = np.ones_like(rho)
            nz = rho > 1e-8
            scale[nz] = radius[nz] / rho[nz]
            return np.column_stack([self._cx + scale * x, self._cy + scale * y])
        return np.column_stack(
            [self._cx + self._fx * x / z, self._cy + self._fy * y / z]
        )

    def _pixel_to_bev(self, u: float, v: float) -> tuple[float, float] | None:
        """Pie en la imagen → píxel BEV, con el mismo modelo que el warp."""
        dx = (u - self._cx) / self._fx
        dy = (v - self._cy) / self._fy
        if self.params.equidistant:
            theta = math.hypot(dx, dy)
            if theta < 1e-8:
                d_cam = np.array([0.0, 0.0, 1.0])
            else:
                s = math.sin(theta) / theta
                d_cam = np.array([s * dx, s * dy, math.cos(theta)])
        else:
            d_cam = np.array([dx, dy, 1.0])
        d_robot = d_cam @ self._cam_to_robot.T
        dz = float(d_robot[2])
        if dz >= -1e-4:
            return None
        t = -self.params.height_mm / dz
        lat = t * float(d_robot[0]) + self.params.right_mm
        fwd = t * float(d_robot[1]) - self.behind_front_mm
        return (
            C.ROBOT_BEV_X + lat / C.MM_PER_PX,
            C.ROBOT_BEV_Y - fwd / C.MM_PER_PX,
        )

    @property
    def behind_front_mm(self) -> float:
        return self.vehicle.vehicle.wheelbase_mm - self.params.forward_from_rear_axle_mm

    def render_camera(self, track, robot_x: float, robot_y: float, heading_deg: float) -> np.ndarray:
        p = self.params
        h = math.radians(heading_deg)
        sh, ch = math.sin(h), math.cos(h)
        ox, oy = robot_to_world(
            self.params.right_mm,
            self.params.forward_from_rear_axle_mm,
            robot_x,
            robot_y,
            heading_deg,
        )

        # Distancia al piso por rayo, y de ahí el punto del tapete. El giro del
        # heading se aplica al punto, no al rayo: una sola rotación por frame.
        t = self._floor_scale * np.float32(p.height_mm)
        finite = self._floor_down & (t >= np.float32(1e-3))
        rr = t * self._ray_right
        ff = t * self._ray_fwd
        gx = np.zeros(t.shape, dtype=np.float32)
        gy = np.zeros(t.shape, dtype=np.float32)
        gx[finite] = np.float32(ox) + ff[finite] * np.float32(sh) + rr[finite] * np.float32(ch)
        gy[finite] = np.float32(oy) + ff[finite] * np.float32(ch) - rr[finite] * np.float32(sh)
        in_mat = (np.abs(gx) <= OUTER_HALF_MM) & (np.abs(gy) <= OUTER_HALF_MM)
        in_island = (np.abs(gx) < INNER_HALF_MM) & (np.abs(gy) < INNER_HALF_MM)
        floor = finite & in_mat & ~in_island
        island = finite & in_island
        best_id = np.zeros(t.shape, dtype=np.uint8)
        best_id = np.where(floor, np.uint8(1), best_id)
        best_id = np.where(island, np.uint8(2), best_id)
        img = _PALETTE[best_id]

        oz = p.height_mm
        half_line = LINE_MM / 2.0
        for seg, color in (
            *((s, ORANGE_BGR) for s in orange_segments()),
            *((s, BLUE_BGR) for s in blue_segments()),
        ):
            self._fill_face(img, self._ribbon_to_cam(seg, half_line, ox, oy, oz, sh, ch), color)

        faces = []
        # Pared en tramos: con el quad entero (3 m) la profundidad media del
        # pintor quedaba por delante de la madera del cajón pegada a ella y la
        # tapaba (CCW: el rosa del INICIO caía de 0.29 a 0.22). Los cortes caen
        # también en los cantos de las maderas, si no el tramo que las cruza
        # sigue tapando una esquina.
        for axis, value, a0, a1, b0, b1 in self._wall_quads():
            n = max(1, int(math.ceil((a1 - a0) / _WALL_TILE_MM)))
            cuts = {a0 + k * (a1 - a0) / n for k in range(n + 1)}
            for box in track.barriers:
                for pt in box:
                    if a0 < pt[1 - axis] < a1:
                        cuts.add(pt[1 - axis])
            cuts = sorted(cuts)
            for c0, c1 in zip(cuts[:-1], cuts[1:]):
                corners = self._quad_corners(axis, value, c0, c1, b0, b1)
                cam = self._to_cam(corners, ox, oy, oz, sh, ch)
                if np.all(cam[:, 2] < 8.0):
                    continue
                faces.append((float(np.mean(cam[:, 2])), cam, WALL_BGR))
        half = SIGN_MM / 2.0
        for sign in track.signs:
            color = RED_BGR if sign.color == "Red" else GREEN_BGR
            for corners in self._box_quads(
                sign.x - half,
                sign.x + half,
                sign.y - half,
                sign.y + half,
                p.sign_height_mm,
            ):
                cam = self._to_cam(corners, ox, oy, oz, sh, ch)
                faces.append((float(np.mean(cam[:, 2])), cam, color))
        for box in track.barriers:
            xs = [pt[0] for pt in box]
            ys = [pt[1] for pt in box]
            for corners in self._box_quads(
                min(xs), max(xs), min(ys), max(ys), p.sign_height_mm
            ):
                cam = self._to_cam(corners, ox, oy, oz, sh, ch)
                faces.append((float(np.mean(cam[:, 2])), cam, MAGENTA_BGR))
        for _depth, cam, color in sorted(faces, key=lambda item: item[0], reverse=True):
            self._fill_face(img, cam, color)
        return img

    def build_bev(self, calib_real_mm: np.ndarray | None = None) -> tuple[BEVTransformer, float]:
        xy = np.asarray(
            calib_real_mm if calib_real_mm is not None else C.CALIB_REAL_MM,
            dtype=np.float64,
        )
        delta = np.column_stack(
            [
                xy[:, 0] - self.params.right_mm,
                xy[:, 1] + self.behind_front_mm,
                np.full(len(xy), -self.params.height_mm),
            ]
        )
        cam = delta @ self._robot_to_cam.T
        if np.any(cam[:, 2] <= 1e-3):
            raise RuntimeError("punto de calibración detrás de la cámara")
        src = self._project(cam).astype(np.float32)
        dst = BEVTransformer._real_mm_to_bev_px(xy.astype(np.float32))
        bev = BEVTransformer(calib_path=Path("/nonexistent-twin-calib.npz"))
        if self.params.equidistant:
            # Un fisheye de piso no es una homografía. El warp y el pie de la
            # lata salen del mismo rayo, así que el verde lejos cae donde está.
            h, w = int(C.BEV_H), int(C.BEV_W)
            gy, gx = np.mgrid[0:h, 0:w]
            lat = (gx.astype(np.float64) - C.ROBOT_BEV_X) * C.MM_PER_PX
            fwd = (C.ROBOT_BEV_Y - gy.astype(np.float64)) * C.MM_PER_PX
            ground = np.stack(
                [
                    lat - self.params.right_mm,
                    fwd + self.behind_front_mm,
                    np.full((h, w), -self.params.height_mm),
                ],
                axis=-1,
            )
            cam_grid = ground.reshape(-1, 3) @ self._robot_to_cam.T
            uv = self._project(cam_grid).reshape(h, w, 2)
            valid = cam_grid[:, 2].reshape(h, w) > 1e-3
            mapx = uv[:, :, 0].astype(np.float32)
            mapy = uv[:, :, 1].astype(np.float32)
            mapx[~valid] = -1.0
            mapy[~valid] = -1.0
            bev.H = np.eye(3, dtype=np.float64)
            bev.H_inv = np.eye(3, dtype=np.float64)

            def warp(frame, _mapx=mapx, _mapy=mapy):
                return cv2.remap(
                    frame, _mapx, _mapy, cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,
                )

            def cam_to_bev(cam_x, cam_y, model=self):
                return model._pixel_to_bev(float(cam_x), float(cam_y))

            bev.warp = warp
            bev.cam_to_bev = cam_to_bev
            back = []
            for pt in src:
                hit = self._pixel_to_bev(float(pt[0]), float(pt[1]))
                back.append(hit if hit is not None else (1e6, 1e6))
            err = float(np.linalg.norm(np.asarray(back) - dst, axis=1).mean())
            return bev, err
        H, _ = cv2.findHomography(src, dst, 0)
        if H is None:
            raise RuntimeError("findHomography falló")
        proj = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
        err = float(np.linalg.norm(proj - dst, axis=1).mean())
        bev.H = H
        bev.H_inv = np.linalg.inv(H)
        # Misma homografía, hoja más grande: ±2.3 m a los lados y 2.6 m
        # adelante. El verde de la foto caía en (576, -29), fuera de los
        # 400 px, y el warp no tenía píxel donde ponerlo.
        self._attach_wide_warp(bev, H)
        return bev, err

    def _attach_wide_warp(self, bev: BEVTransformer, H: np.ndarray) -> None:
        pad_x, pad_y = 950, 920
        wide = (400 + pad_x + 950, 400 + pad_y)
        T = np.array(
            [[1.0, 0.0, pad_x], [0.0, 1.0, pad_y], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        H_wide = T @ H.astype(np.float64)

        def warp_wide(frame, _H=H_wide, _size=wide):
            return cv2.warpPerspective(frame, _H, _size)

        bev.warp_wide = warp_wide
        bev.wide_shift = (pad_x, pad_y)

    def see(self, bev_tf: BEVTransformer, vision, track, robot_x, robot_y, heading_deg):
        cam = self.render_camera(track, robot_x, robot_y, heading_deg)
        annotated, positions = vision.detect_on(cam.copy())
        bev = bev_tf.warp(cam)
        obstacles = []
        for color_name in ("Red", "Green"):
            for obj in positions.get(color_name, []):
                mapped = map_obstacle_to_bev(bev_tf, *obj)
                if mapped is not None:
                    obstacles.append((mapped[0], mapped[1], color_name))
        return annotated, bev, obstacles

    @classmethod
    def from_calib_npz(
        cls,
        path: Path | str,
        twin: TwinParams | None = None,
        hfov_deg: float = 62.0,
        search_hfov: bool = False,
        calib_points_mm: np.ndarray | None = None,
    ) -> tuple[CameraModel, dict[str, Any]]:
        """Cámara pinhole que explica la homografía real del .npz.

        Los puntos de calibración (mm desde el eje DELANTERO) se llevan a la
        imagen con H⁻¹ y se resuelve PnP planar. Con search_hfov el FOV sale
        del mínimo error de reproyección de PnP: una homografía de piso solo
        es compatible con un fx (centro óptico en el centro del cuadro).
        El error que queda es lo que el pinhole no explica (distorsión de la
        lente, clics corridos). La Pi warpea con la H real, no con la de este
        modelo: el twin debe usar BEVTransformer(calib_path=path) para el BEV.
        """
        twin = twin or TwinParams()
        calib = np.asarray(
            calib_points_mm if calib_points_mm is not None else C.CALIB_REAL_MM,
            dtype=np.float64,
        )
        H = np.load(str(path))["H"].astype(np.float64)
        dst = BEVTransformer._real_mm_to_bev_px(calib.astype(np.float32))
        img_pts = cv2.perspectiveTransform(
            dst.reshape(-1, 1, 2).astype(np.float64), np.linalg.inv(H)
        ).reshape(-1, 2)
        # Piso: x derecha, y adelante del eje delantero, z arriba.
        obj_pts = np.column_stack([calib[:, 0], calib[:, 1], np.zeros(len(calib))])
        base = CameraParams()
        w, h = float(base.width), float(base.height)

        def solve(hfov: float):
            fx = (w / 2.0) / math.tan(math.radians(hfov) / 2.0)
            K = np.array([[fx, 0.0, w / 2.0], [0.0, fx, h / 2.0], [0.0, 0.0, 1.0]])
            best = None
            n, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                obj_pts, img_pts, K, None, flags=cv2.SOLVEPNP_IPPE)
            for i in range(n):
                R, _ = cv2.Rodrigues(rvecs[i])
                center = (-R.T @ tvecs[i]).reshape(3)
                if center[2] <= 0.0:
                    continue   # solución espejo, cámara bajo el piso
                proj, _ = cv2.projectPoints(obj_pts, rvecs[i], tvecs[i], K, None)
                err = float(np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1).mean())
                if best is None or err < best[0]:
                    best = (err, R, center)
            return best

        best_hfov = hfov_deg
        best = solve(hfov_deg)
        if search_hfov:
            for hf in np.arange(40.0, 170.5, 1.0):
                cand = solve(float(hf))
                if cand is not None and (best is None or cand[0] < best[0]):
                    best, best_hfov = cand, float(hf)
        if best is None:
            raise RuntimeError("PnP no encontró una cámara sobre el piso")
        err, R, center = best
        cam_to_robot = R.T
        axis = cam_to_robot @ np.array([0.0, 0.0, 1.0])
        x_img = cam_to_robot @ np.array([1.0, 0.0, 0.0])
        wb = twin.vehicle.wheelbase_mm
        report = {
            "hfov_deg": best_hfov,
            "reproj_err_px": err,
            "height_mm": float(center[2]),
            "tilt_deg": math.degrees(math.asin(float(np.clip(-axis[2], -1.0, 1.0)))),
            # + = mira a la derecha / borde derecho de la imagen hacia arriba
            "yaw_deg": math.degrees(math.atan2(float(axis[0]), float(axis[1]))),
            "roll_deg": math.degrees(math.asin(float(np.clip(x_img[2], -1.0, 1.0)))),
            "forward_from_rear_axle_mm": wb + float(center[1]),
            "right_from_rear_axle_mm": float(center[0]),
            "image_points": img_pts.tolist(),
        }
        cam_params = CameraParams(
            hfov_deg=best_hfov,
            tilt_deg=report["tilt_deg"],
            height_mm=report["height_mm"],
            forward_from_rear_axle_mm=report["forward_from_rear_axle_mm"],
            right_mm=report["right_from_rear_axle_mm"],
            rotation_cam_to_robot=tuple(map(tuple, cam_to_robot)),
            equidistant=False,
        )
        return cls(params=cam_params, vehicle=twin), report

    # ── geometría de escena (igual que chassis_twin) ────────────────────────

    def _wall_quads(self):
        o = OUTER_HALF_MM
        i = INNER_HALF_MM
        z = self.params.wall_height_mm
        return (
            (0, o, -o, o, 0.0, z),
            (0, -o, -o, o, 0.0, z),
            (1, o, -o, o, 0.0, z),
            (1, -o, -o, o, 0.0, z),
            (0, i, -i, i, 0.0, z),
            (0, -i, -i, i, 0.0, z),
            (1, i, -i, i, 0.0, z),
            (1, -i, -i, i, 0.0, z),
        )

    @staticmethod
    def _quad_corners(axis, value, a0, a1, b0, b1):
        if axis == 0:
            return (
                (value, a0, b0),
                (value, a1, b0),
                (value, a1, b1),
                (value, a0, b1),
            )
        if axis == 1:
            return (
                (a0, value, b0),
                (a1, value, b0),
                (a1, value, b1),
                (a0, value, b1),
            )
        return (
            (a0, b0, value),
            (a1, b0, value),
            (a1, b1, value),
            (a0, b1, value),
        )

    @staticmethod
    def _box_quads(x0, x1, y0, y1, z1):
        return (
            CameraModel._quad_corners(0, x0, y0, y1, 0.0, z1),
            CameraModel._quad_corners(0, x1, y0, y1, 0.0, z1),
            CameraModel._quad_corners(1, y0, x0, x1, 0.0, z1),
            CameraModel._quad_corners(1, y1, x0, x1, 0.0, z1),
            CameraModel._quad_corners(2, z1, x0, x1, y0, y1),
        )

    @staticmethod
    def _ribbon(a, b, half):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy) or 1.0
        px, py = -dy / length * half, dx / length * half
        z = 1.0
        return (
            (ax + px, ay + py, z),
            (bx + px, by + py, z),
            (bx - px, by - py, z),
            (ax - px, ay - py, z),
        )

    def _to_cam(self, pts, ox, oy, oz, sh, ch):
        pts = np.asarray(pts, dtype=np.float64)
        dx = pts[:, 0] - ox
        dy = pts[:, 1] - oy
        dz = pts[:, 2] - oz
        right = dx * ch - dy * sh
        fwd = dx * sh + dy * ch
        robot = np.column_stack([right, fwd, dz])
        return robot @ self._robot_to_cam.T

    def _ribbon_to_cam(self, seg, half, ox, oy, oz, sh, ch):
        return self._to_cam(self._ribbon(*seg, half), ox, oy, oz, sh, ch)

    def _clip_near(self, cam_pts, zmin=8.0):
        out = []
        n = len(cam_pts)
        for i in range(n):
            a = cam_pts[i]
            b = cam_pts[(i + 1) % n]
            a_in = a[2] >= zmin
            b_in = b[2] >= zmin
            if a_in and b_in:
                out.append(b)
            elif a_in != b_in:
                t = (zmin - a[2]) / (b[2] - a[2])
                hit = a + t * (b - a)
                out.append(hit)
                if b_in:
                    out.append(b)
        return out

    def _fill_face(self, img, cam_pts, color):
        poly = self._clip_near(cam_pts)
        if len(poly) < 3:
            return
        poly = np.asarray(poly, dtype=np.float64)
        uv = self._project(poly)
        if np.any(np.abs(uv) > 20000):
            return
        cv2.fillConvexPoly(img, np.round(uv).astype(np.int32), color)
