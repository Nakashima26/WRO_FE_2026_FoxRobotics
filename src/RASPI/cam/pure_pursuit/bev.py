"""
BEVTransformer — Inversión de Perspectiva (IPM) para vista Bird's Eye View.

Carga la homografía desde bev_calib.npz (generado con calibrate.py).
Si existe cam_intrinsics.npz Y la homografía se ajustó sobre imagen
undistorsionada (`undistort_used=1`), warp()/cam_to_bev() undistorsionan primero.

Si el archivo de homografía no existe, is_calibrated = False y el runtime
usa fallback PID.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from . import config as C


def _homography_method_ls():
    """Least-squares DLT. Con puntos clickeados a propósito (todos inliers)
    RANSAC puede tirar el punto lejano — el más importante — y empeorar el BEV."""
    return 0


class BEVTransformer:
    """
    Encapsula la matriz de homografía H para transformar el frame de la cámara
    (vista perspectiva 640×480) a una imagen BEV (top-down 400×400 px).

    Coordenadas BEV:
      - Origen (0,0) en esquina superior-izquierda
      - X crece hacia la derecha  (= derecha física del robot)
      - Y crece hacia abajo       (= atrás físico del robot)
      - Robot siempre en (ROBOT_BEV_X, ROBOT_BEV_Y) = (200, 380)
      - Dirección de marcha → Y decreciente (hacia arriba en imagen)
    """

    def __init__(self, calib_path: Path | None = None):
        self.H: np.ndarray | None = None
        self.H_inv: np.ndarray | None = None
        self.last_reproj_err_px: float | None = None
        self.last_point_errs: np.ndarray | None = None
        self.last_src_pts: np.ndarray | None = None
        self.undistort_used: bool = False
        self.K: np.ndarray | None = None
        self.dist: np.ndarray | None = None
        self.newK: np.ndarray | None = None
        self._mapx = None
        self._mapy = None
        self._map_size: tuple[int, int] | None = None
        self._calib_path = Path(calib_path) if calib_path else C.CALIB_FILE
        self._load_intrinsics()
        if self._calib_path.exists():
            self._load(self._calib_path)

    # ── Intrínsecos ───────────────────────────────────────────────────────────

    def _load_intrinsics(self, path: Path | None = None) -> None:
        ipath = Path(path) if path else getattr(C, "INTRINSICS_FILE", None)
        if ipath is None or not Path(ipath).exists():
            return
        data = np.load(str(ipath))
        if "camera_matrix" not in data or "dist_coeffs" not in data:
            print(f"[BEV] {ipath} no tiene camera_matrix/dist_coeffs — se ignora.",
                  flush=True)
            return
        self.K = data["camera_matrix"].astype(np.float64)
        self.dist = data["dist_coeffs"].astype(np.float64).reshape(-1)
        if "new_camera_matrix" in data:
            self.newK = data["new_camera_matrix"].astype(np.float64)
        w = int(data["image_w"]) if "image_w" in data else 0
        h = int(data["image_h"]) if "image_h" in data else 0
        rms = float(data["rms"]) if "rms" in data else float("nan")
        print(f"[BEV] Intrínsecos cargados desde {ipath} "
              f"(rms={rms:.3f}px, {w}x{h})", flush=True)

    def _can_undistort(self) -> bool:
        return self.K is not None and self.dist is not None

    def uses_undistort(self) -> bool:
        """True si H se ajustó sobre imagen undistorsionada y hay K."""
        return bool(self.undistort_used and self._can_undistort())

    def _ensure_undist_maps(self, w: int, h: int) -> bool:
        if not self._can_undistort():
            return False
        if self._mapx is not None and self._map_size == (w, h):
            return True
        newK = self.newK
        if newK is None:
            newK, _ = cv2.getOptimalNewCameraMatrix(
                self.K, self.dist, (w, h), 0.0, (w, h)
            )
            self.newK = newK
        self._mapx, self._mapy = cv2.initUndistortRectifyMap(
            self.K, self.dist, None, newK, (w, h), cv2.CV_16SC2
        )
        self._map_size = (w, h)
        return True

    def undistort_frame(self, frame: np.ndarray, *, force: bool = False) -> np.ndarray:
        """Imagen en el espacio donde se ajustó H. Sin intrínsecos, es no-op."""
        if frame is None or not (force or self.uses_undistort()):
            return frame
        h, w = frame.shape[:2]
        if not self._ensure_undist_maps(w, h):
            return frame
        return cv2.remap(frame, self._mapx, self._mapy, cv2.INTER_LINEAR)

    def undistort_points(self, pts: np.ndarray, *, force: bool = False) -> np.ndarray:
        """Píxeles del frame crudo → píxeles undistorsionados (newK)."""
        arr = np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)
        if not (force or self.uses_undistort()):
            return arr.reshape(-1, 2)
        P = self.newK if self.newK is not None else self.K
        out = cv2.undistortPoints(arr, self.K, self.dist, P=P)
        return out.reshape(-1, 2)

    def distort_points(self, pts_undist: np.ndarray) -> np.ndarray:
        """Píxeles undistorsionados (newK) → píxeles del frame crudo."""
        arr = np.asarray(pts_undist, dtype=np.float32).reshape(-1, 1, 2)
        if not self.uses_undistort():
            return arr.reshape(-1, 2)
        P = self.newK if self.newK is not None else self.K
        n = cv2.undistortPoints(arr, P, None)
        xyz = np.hstack([n.reshape(-1, 2), np.ones((n.shape[0], 1), np.float32)])
        img, _ = cv2.projectPoints(
            xyz, np.zeros(3), np.zeros(3), self.K, self.dist
        )
        return img.reshape(-1, 2)

    # ── Persistencia ──────────────────────────────────────────────────────────

    def _load(self, path: Path) -> None:
        data = np.load(str(path))
        self.H = data["H"].astype(np.float64)
        self.H_inv = data["H_inv"].astype(np.float64)
        if "reproj_err_px" in data:
            self.last_reproj_err_px = float(data["reproj_err_px"])
        if "src_pts" in data:
            self.last_src_pts = data["src_pts"].astype(np.float32)
        if "point_err_px" in data:
            self.last_point_errs = data["point_err_px"].astype(np.float64)
        self.undistort_used = bool(int(data["undistort_used"])) if "undistort_used" in data else False
        extra = ""
        if self.last_reproj_err_px is not None:
            extra = f" (error reproyección guardado: {self.last_reproj_err_px:.2f}px)"
        print(f"[BEV] Calibración cargada desde {path}{extra}", flush=True)
        if self.K is not None and not self.undistort_used:
            print("[BEV] Hay cam_intrinsics.npz pero esta homografía se ajustó "
                  "SIN undistort. No se aplica remap (rehace calibrate.py para usarlo).",
                  flush=True)
        elif self.undistort_used and self.K is None:
            print("[BEV] La homografía espera undistort pero no hay "
                  f"{getattr(C, 'INTRINSICS_FILE', 'cam_intrinsics.npz')}. "
                  "El BEV va a salir mal hasta que copies los intrínsecos.",
                  flush=True)

    def fit(self, src_pts: np.ndarray, undistort_used: bool | None = None) -> dict:
        """
        Ajusta H en memoria (NO escribe disco). src_pts en el mismo espacio
        en el que se va a hacer warp: undistorsionado si hay intrínsecos activos.

        Retorna dict con mean_err, max_err, worst_idx, errs, dst_pts.
        """
        dst_pts = self._real_mm_to_bev_px(C.CALIB_REAL_MM)
        src = np.asarray(src_pts, dtype=np.float32).reshape(-1, 2)
        n_pts = len(src)
        if n_pts < 4:
            raise RuntimeError(f"[BEV] Se necesitan al menos 4 puntos, llegaron {n_pts}.")
        if n_pts != len(dst_pts):
            raise RuntimeError(
                f"[BEV] {n_pts} clics vs {len(dst_pts)} puntos en CALIB_REAL_MM."
            )

        H, _mask = cv2.findHomography(src, dst_pts, _homography_method_ls())
        if H is None:
            raise RuntimeError("[BEV] findHomography falló — verifica los puntos fuente.")
        H_inv = np.linalg.inv(H)

        proj = cv2.perspectiveTransform(
            src.reshape(-1, 1, 2).astype(np.float32), H
        ).reshape(-1, 2)
        errs = np.linalg.norm(proj - dst_pts, axis=1)
        mean_err = float(errs.mean())
        max_err = float(errs.max())
        worst_idx = int(np.argmax(errs))

        print(f"[BEV] Error de reproyección: media={mean_err:.2f}px "
              f"(~{mean_err * C.MM_PER_PX:.1f}mm)  max={max_err:.2f}px "
              f"en punto #{worst_idx + 1}", flush=True)
        for i, e in enumerate(errs):
            print(f"      P{i + 1}: {e:.2f}px", flush=True)

        if mean_err > C.CALIB_MAX_MEAN_ERR_PX:
            print(f"[BEV] ⚠ Error alto (> {C.CALIB_MAX_MEAN_ERR_PX}px). "
                  f"Revisa el punto #{worst_idx + 1} — probablemente el clic "
                  f"quedó desalineado del marcador físico. Considera rehacer (R).",
                  flush=True)

        self.H = H
        self.H_inv = H_inv
        self.last_reproj_err_px = mean_err
        self.last_point_errs = errs
        self.last_src_pts = src.copy()
        if undistort_used is None:
            undistort_used = self.K is not None
        self.undistort_used = bool(undistort_used)
        return {
            "mean_err": mean_err,
            "max_err": max_err,
            "worst_idx": worst_idx,
            "errs": errs,
            "dst_pts": dst_pts,
        }

    def persist(self, calib_path: Path | None = None) -> Path:
        """Escribe bev_calib.npz con H actual. No recálcula."""
        if self.H is None or self.H_inv is None:
            raise RuntimeError("[BEV] persist() sin H — primero fit().")
        path = Path(calib_path) if calib_path else self._calib_path
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = dict(
            H=self.H,
            H_inv=self.H_inv,
            reproj_err_px=np.float64(self.last_reproj_err_px or 0.0),
            undistort_used=np.int32(1 if self.undistort_used else 0),
        )
        if self.last_src_pts is not None:
            payload["src_pts"] = self.last_src_pts
            payload["dst_pts"] = self._real_mm_to_bev_px(C.CALIB_REAL_MM)
        if self.last_point_errs is not None:
            payload["point_err_px"] = self.last_point_errs
        np.savez(str(path), **payload)
        print(f"[BEV] Calibración guardada en {path}", flush=True)
        return path

    def save(self, src_pts: np.ndarray, calib_path: Path | None = None) -> float:
        """Ajusta H y la escribe. Conservado para callers viejos."""
        stats = self.fit(src_pts)
        self.persist(calib_path)
        return stats["mean_err"]

    # ── Transformaciones ──────────────────────────────────────────────────────

    def warp(self, frame: np.ndarray) -> np.ndarray | None:
        """Retorna imagen BEV (BEV_W × BEV_H) o None si no calibrado."""
        if self.H is None:
            return None
        src = self.undistort_frame(frame)
        return cv2.warpPerspective(src, self.H, (C.BEV_W, C.BEV_H))

    def cam_to_bev(self, cam_x: float, cam_y: float) -> tuple[float, float] | None:
        """Proyecta un píxel (cam_x, cam_y) del frame crudo al BEV."""
        if self.H is None:
            return None
        pt = self.undistort_points([(cam_x, cam_y)]).reshape(-1, 1, 2)
        result = cv2.perspectiveTransform(pt, self.H)
        return float(result[0, 0, 0]), float(result[0, 0, 1])

    def bev_to_cam(self, bev_x: float, bev_y: float) -> tuple[float, float] | None:
        """Proyecta un píxel BEV de vuelta al frame crudo de cámara."""
        if self.H_inv is None:
            return None
        pt = np.array([[[bev_x, bev_y]]], dtype=np.float32)
        undist = cv2.perspectiveTransform(pt, self.H_inv).reshape(-1, 2)
        dist = self.distort_points(undist)
        return float(dist[0, 0]), float(dist[0, 1])

    def bev_in_bounds(self, bev_x: float, bev_y: float) -> bool:
        return 0.0 <= bev_x < C.BEV_W and 0.0 <= bev_y < C.BEV_H

    def draw_ground_grid(self, cam_bgr: np.ndarray, step_mm: float | None = None,
                         color: tuple[int, int, int] = (0, 220, 0)) -> np.ndarray:
        """
        Dibuja sobre la imagen de cámara (espacio donde vive H: undistorsionado
        si aplica) una grilla métrica del piso. Si las líneas no coinciden con
        cintas/juntas reales, la homografía está mal aunque el error de los
        9 clics salga bajo.
        """
        if self.H_inv is None:
            return cam_bgr
        out = cam_bgr.copy()
        h, w = out.shape[:2]
        step = float(step_mm if step_mm is not None else getattr(C, "CALIB_GRID_STEP_MM", 100.0))
        x_mm = np.arange(-400.0, 401.0, step)
        y_mm = np.arange(0.0, 801.0, step)

        def _to_cam(pts_mm: np.ndarray) -> np.ndarray:
            bev = self._real_mm_to_bev_px(pts_mm.astype(np.float32))
            cam = cv2.perspectiveTransform(
                bev.reshape(-1, 1, 2), self.H_inv
            ).reshape(-1, 2)
            return cam

        for x in x_mm:
            mm = np.stack([np.full_like(y_mm, x), y_mm], axis=1)
            p = _to_cam(mm)
            vis = [(int(round(u)), int(round(v))) for u, v in p
                   if -50 <= u < w + 50 and -50 <= v < h + 50]
            if len(vis) >= 2:
                cv2.polylines(out, [np.array(vis, np.int32)], False, color, 1, cv2.LINE_AA)
        for y in y_mm:
            mm = np.stack([x_mm, np.full_like(x_mm, y)], axis=1)
            p = _to_cam(mm)
            vis = [(int(round(u)), int(round(v))) for u, v in p
                   if -50 <= u < w + 50 and -50 <= v < h + 50]
            if len(vis) >= 2:
                cv2.polylines(out, [np.array(vis, np.int32)], False, color, 1, cv2.LINE_AA)
        return out

    @property
    def is_calibrated(self) -> bool:
        return self.H is not None

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _real_mm_to_bev_px(real_mm: np.ndarray) -> np.ndarray:
        """
        Convierte coordenadas reales (x_mm lateral, y_mm adelante) a píxeles BEV.
        Robot siempre en (ROBOT_BEV_X, ROBOT_BEV_Y).
        """
        dst = np.zeros_like(real_mm)
        dst[:, 0] = C.ROBOT_BEV_X + real_mm[:, 0] / C.MM_PER_PX   # lateral
        dst[:, 1] = C.ROBOT_BEV_Y - real_mm[:, 1] / C.MM_PER_PX   # adelante → arriba
        return dst.astype(np.float32)

    @staticmethod
    def expected_dst_pts() -> np.ndarray:
        """Retorna los N puntos destino BEV para mostrar en calibrate.py."""
        return BEVTransformer._real_mm_to_bev_px(C.CALIB_REAL_MM)
