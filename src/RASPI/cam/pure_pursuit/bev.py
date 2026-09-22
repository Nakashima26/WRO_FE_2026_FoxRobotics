"""
BEVTransformer — Inversión de Perspectiva (IPM) para vista Bird's Eye View.

Carga la homografía desde bev_calib.npz (generado con calibrate.py).
Si el archivo no existe, is_calibrated = False y el runtime usa fallback PID.
"""

import cv2
import numpy as np
from pathlib import Path

from . import config as C


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
        path = Path(calib_path) if calib_path else C.CALIB_FILE
        if path.exists():
            self._load(path)

    # ── Persistencia ──────────────────────────────────────────────────────────

    def _load(self, path: Path) -> None:
        data = np.load(str(path))
        self.H     = data["H"].astype(np.float64)
        self.H_inv = data["H_inv"].astype(np.float64)
        if "reproj_err_px" in data:
            self.last_reproj_err_px = float(data["reproj_err_px"])
            print(f"[BEV] Calibración cargada desde {path} "
                  f"(error reproyección guardado: {self.last_reproj_err_px:.2f}px)", flush=True)
        else:
            print(f"[BEV] Calibración cargada desde {path}", flush=True)

    def save(self, src_pts: np.ndarray, calib_path: Path | None = None) -> float:
        """
        Calcula y guarda la homografía a partir de N puntos fuente en imagen
        (N debe coincidir con len(C.CALIB_REAL_MM), mínimo 4).

        src_pts: np.float32 shape (N, 2) — píxeles en el frame de cámara,
                 en el mismo orden que C.CALIB_REAL_MM.

        Retorna el error medio de reproyección en píxeles BEV.  Un valor alto
        (> C.CALIB_MAX_MEAN_ERR_PX) sugiere que uno o más clics quedaron mal
        puestos y conviene rehacer la calibración.
        """
        dst_pts = self._real_mm_to_bev_px(C.CALIB_REAL_MM)

        n_pts = len(src_pts)
        if n_pts < 4:
            raise RuntimeError(f"[BEV] Se necesitan al menos 4 puntos, llegaron {n_pts}.")

        # Con 4 puntos el ajuste es exacto (no hay redundancia que filtrar);
        # con 5+ puntos RANSAC sí puede promediar/descartar clics ruidosos.
        method = cv2.RANSAC if n_pts > 4 else 0
        H, mask = cv2.findHomography(src_pts, dst_pts, method, 3.0)
        if H is None:
            raise RuntimeError("[BEV] findHomography falló — verifica los puntos fuente.")
        H_inv = np.linalg.inv(H)

        # ── Error de reproyección: qué tan bien H explica los clics dados ────
        proj = cv2.perspectiveTransform(
            src_pts.reshape(-1, 1, 2).astype(np.float32), H
        ).reshape(-1, 2)
        errs = np.linalg.norm(proj - dst_pts, axis=1)
        mean_err = float(errs.mean())
        max_err  = float(errs.max())
        worst_idx = int(np.argmax(errs))

        print(f"[BEV] Error de reproyección: media={mean_err:.2f}px "
              f"(~{mean_err * C.MM_PER_PX:.1f}mm)  max={max_err:.2f}px "
              f"en punto #{worst_idx + 1}", flush=True)

        if mean_err > C.CALIB_MAX_MEAN_ERR_PX:
            print(f"[BEV] ⚠ Error alto (> {C.CALIB_MAX_MEAN_ERR_PX}px). "
                  f"Revisa el punto #{worst_idx + 1} — probablemente el clic "
                  f"quedó desalineado del marcador físico. Considera rehacer (R).",
                  flush=True)

        path = Path(calib_path) if calib_path else C.CALIB_FILE
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(str(path), H=H, H_inv=H_inv, reproj_err_px=mean_err)
        self.H     = H
        self.H_inv = H_inv
        self.last_reproj_err_px = mean_err
        print(f"[BEV] Calibración guardada en {path}", flush=True)
        return mean_err

    # ── Transformaciones ──────────────────────────────────────────────────────

    def warp(self, frame: np.ndarray) -> np.ndarray | None:
        """Retorna imagen BEV (BEV_W × BEV_H) o None si no calibrado."""
        if self.H is None:
            return None
        return cv2.warpPerspective(frame, self.H, (C.BEV_W, C.BEV_H))

    def cam_to_bev(self, cam_x: float, cam_y: float) -> tuple[float, float] | None:
        """Proyecta un píxel (cam_x, cam_y) de la imagen perspectiva al BEV."""
        if self.H is None:
            return None
        pt = np.array([[[cam_x, cam_y]]], dtype=np.float32)
        result = cv2.perspectiveTransform(pt, self.H)
        return float(result[0, 0, 0]), float(result[0, 0, 1])

    def bev_to_cam(self, bev_x: float, bev_y: float) -> tuple[float, float] | None:
        """Proyecta un píxel BEV de vuelta a coordenadas de imagen perspectiva."""
        if self.H_inv is None:
            return None
        pt = np.array([[[bev_x, bev_y]]], dtype=np.float32)
        result = cv2.perspectiveTransform(pt, self.H_inv)
        return float(result[0, 0, 0]), float(result[0, 0, 1])

    def bev_in_bounds(self, bev_x: float, bev_y: float) -> bool:
        return 0.0 <= bev_x < C.BEV_W and 0.0 <= bev_y < C.BEV_H

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
