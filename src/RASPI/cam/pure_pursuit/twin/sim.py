"""Co-simulación firmware SIL + PPRuntime + física."""

from __future__ import annotations

import contextlib
import math
import os
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np

from pure_pursuit import config as C
from pure_pursuit.bev import BEVTransformer
from pure_pursuit.runtime_nuevo import PPConfig, PPRuntime
from pure_pursuit.twin.camera import CameraModel
from pure_pursuit.twin.firmware.fw import FirmwareSIL, FirmwareSetupError
from pure_pursuit.twin.metrics import compute_metrics, _parse_ack_field, _parse_est
from pure_pursuit.twin.params import TwinParams, max_wheel_deg, physical_motor, wheel_deg_from_servo
from pure_pursuit.twin.pi_link import SimSerialLink
from pure_pursuit.twin.sensors import Bno085Heading, Encoder, Gyro, Pose, ToF, Ultrasonic
from pure_pursuit.twin.vehicle import Vehicle
from pure_pursuit.twin.world import OUTER_HALF_MM, World, body_corners, collision
from pure_pursuit.wro_field import randomize
from vision import Vision

_DEFAULT_CALIB = (
    Path(__file__).resolve().parent / "calib" / "bev_calib_main_2026-08-19.npz"
)
_UNARMED_FRAMES = 15
_COLLISION_INTERVAL_S = 0.005
_STUCK_WINDOW_S = 8.0
_STUCK_PROGRESS_MM = 50.0
_STOPPED_SPEED_MM_S = 15.0
_STOPPED_HOLD_S = 0.5

PRESETS: dict[str, dict[str, Any]] = {
    "baseline": {
        "fw_source": "head",
        "fw_overrides": {"TURNS_PER_RACE": "12", "PARK_MODO": "PARK_NINGUNO"},
        "start": "centro",
    },
    "baseline_park": {
        "fw_source": "head",
        "fw_overrides": {"TURNS_PER_RACE": "12"},
        "start": "cajon",
    },
    "giro_rapido": {
        "fw_source": "worktree",
        "fw_overrides": {
            "TURNS_PER_RACE": "12",
            "GIRO_RAPIDO_MODO": "1",
            # La línea del mapa dobla sola (verde en la boca): el ESP la cuenta.
            "GIRO_PI_CUENTA_DEG": "70.0f",
            # A PWM~155 el hold del mapa suelta tarde: con ventana 55 (grMin=40)
            # se pierde el arco y cae a MANIOBRA en reversa (esquina del cajón).
            "GR_VENTANA_CM": "75",
        },
        "pi_overrides": {
            "CORNER_HINT_TO_ESP": True,
            # Verde en la boca + giro a la derecha: la memoria lo pasa por la
            # izquierda y el giro no sale hasta después.
            "CORNER_EXTERIOR_PASS_ENABLED": True,
            # El verde de la boca cae ~110px por delante de la naranja; con 70
            # se iba a "beyond", prio bajaba a 0 y el giro salía encima.
            "CORNER_EXT_PASS_BAND_PX": 160.0,
            # El bloqueo de 10 frames, con esta mira, come la ventana de
            # 40–95 cm: la lata ya salió y el giro nace pegado a la pared.
            "CORNER_EXT_PASS_TURN_BLOCK_FRAMES": 0,
            # El path abre a 130px y el lookahead es 100: el carro ya está
            # encima de la lata cuando empieza a abrirse, y el cuerpo (65 mm)
            # más la lata (25 mm) no caben en los 70 mm del inflado.
            "CENTERLINE_RAMP_PX": 220,
            "OBS_INFLATE_R": 56,
            # La línea se cortaba en BEV_H/3 (~49 cm) aunque la pared de la
            # esquina ya estaba dibujada arriba. Muestrea hasta ~70 cm.
            "CENTERLINE_TOP_Y": 30,
            # La línea la arma el mapa (lado de paso), no el BEV.
            "DIGITAL_MAP_STEER": True,
            # Outer: hold clásico. Inner/cerrado: más alto = suelta antes
            # (dF<=hold); con 80 la línea se quedaba recta y el carro esperaba
            # aunque el arco cerrado ya estaba dibujado.
            "DIGITAL_MAP_TURN_HOLD_CM": 65.0,
            "DIGITAL_MAP_TURN_HOLD_INNER_CM": 95.0,
            # Recta del cajón: no abrir tanto al hueco magenta (evita llegar
            # a la esquina SW pegado a la exterior → MANIOBRA en reversa).
            "DIGITAL_MAP_PARK_OUTER_MAX_MM": 120.0,
            # Verde en la boca: la esquina la dobla el PP (el ESP la cuenta).
            "DIGITAL_MAP_PP_TURN_CM": 75.0,
        },
        "start": "cajon",
    },
    "mapa": {
        "fw_source": "worktree",
        "fw_overrides": {
            "TURNS_PER_RACE": "12",
            "PARK_MODO": "PARK_NINGUNO",
            "GIRO_RAPIDO_MODO": "1",
        },
        "pi_overrides": {"TRACK_MAP_ENABLED": True, "CORNER_HINT_TO_ESP": True},
        "start": "centro",
    },
}
# Hardware siguiente: encoder de cuadratura en el motor + 4 ToF (L, R, atrás,
# frente). Igual que giro_rapido; el .ino real sigue con FOX_ENCODER/FOX_TOF = 0.
# La Pi no necesita overrides: px/py del ACK ya son la pose (Odometry
# "esp_pose", digital_map) y tL/tR/tB/tF entran al Localizer de track_map
# (TRACK_MAP_SHADOW=True por defecto).
PRESETS["hw_nuevo"] = {
    **PRESETS["giro_rapido"],
    "fw_overrides": dict(PRESETS["giro_rapido"]["fw_overrides"]),
    "fw_defines": {"FOX_ENCODER": "1", "FOX_TOF": "1"},
    # IMU nueva: BNO085 (yaw fusionado en el chip), no el MPU6050.
    "imu": "bno085",
    # Pi 5 con cámara a 40 fps (vision.open_camera); el pipeline completo a 40 fps
    # NO está medido: si el real no llega, bajar aquí y exportar FOX_PI_FPS en la Pi.
    # PI_FPS escala las ventanas en frames.
    "pi_period_s": 0.025,
    "pi_overrides": {
        **PRESETS["giro_rapido"]["pi_overrides"],
        # La pose del mapa derivaba 150–430 mm (encoder + gyro) sin latas
        # confirmadas que la corrigieran: sonares contra paredes conocidas.
        "DIGITAL_MAP_WALL_FIX": True,
        "PI_FPS": 1.0 / 0.025,
        # Compensación del TWIN, no del carro: con los voladizos del CAD v2 (cola a 5 mm
        # de la madera, wro_field.stall_start_xy) la pared magenta ocupa ~0.25 del cuadro
        # simulado y el umbral real (0.28, calibrado en el carro) no dispara el INICIO.
        # En el carro el umbral se calibra con pick_color.py; aquí no se toca config.py.
        "PARK_PINK_RATIO_MIN": 0.20,
    },
}

# Modo ruta (route_mode.py): hw_nuevo + la vuelta como curva en mm que el ESP
# sigue con su odometría; localización EKF propia (localizer.py), así que las
# correcciones viejas del mapa (pared/lata) van apagadas. Vuelta 1 sin mapa.
PRESETS["ruta"] = {
    **PRESETS["hw_nuevo"],
    "fw_overrides": {**PRESETS["hw_nuevo"]["fw_overrides"], "PARK_MODO": "PARK_NINGUNO", "MOTOR_MAX": "155"},
    "pi_overrides": {
        **PRESETS["hw_nuevo"]["pi_overrides"],
        "ROUTE_FOLLOW": True,
        "RACING_LINE": "",
        "DIGITAL_MAP_WALL_FIX": False,
        "DIGITAL_MAP_SIGN_PULL": False,
    },
}

# Modo ruta + perfil por curvatura: rectas hasta 255, curvas al tope de hoy (155).
PRESETS["ruta_perfil"] = {
    **PRESETS["ruta"],
    "fw_overrides": {**PRESETS["ruta"]["fw_overrides"], "RUTA_PWM_MAX": "255",
                     "TERMINANDO_MS": "0"},
    "pi_overrides": {**PRESETS["ruta"]["pi_overrides"], "ROUTE_PROFILE": True},
}


@dataclass
class RunResult:
    metrics: dict[str, Any]
    trace: list[dict[str, Any]]
    field: Any
    camera_report: dict[str, Any] | None = None
    wall_time_s: float = 0.0
    stop_reason: str = ""


@dataclass
class SimFrameView:
    t: float
    field: Any
    vehicle: Vehicle
    frame_result: Any
    fw_debug_tail: str
    est: str | None
    tc: int | None


def resolve_preset(
    name: str | None,
    *,
    fw_source: str | None = None,
    fw_overrides: dict[str, str] | None = None,
    fw_defines: dict[str, str] | None = None,
    pi_overrides: dict[str, Any] | None = None,
    start: str | None = None,
) -> dict[str, Any]:
    if name:
        if name not in PRESETS:
            raise ValueError(f"preset desconocido: {name}")
        base = dict(PRESETS[name])
    else:
        base = {"fw_source": "worktree", "start": start or "centro"}
    if fw_source is not None:
        base["fw_source"] = fw_source
    if fw_overrides:
        base["fw_overrides"] = {**base.get("fw_overrides", {}), **fw_overrides}
    if fw_defines:
        base["fw_defines"] = {**base.get("fw_defines", {}), **fw_defines}
    if pi_overrides:
        base["pi_overrides"] = {**base.get("pi_overrides", {}), **pi_overrides}
    if start is not None:
        base["start"] = start
    return base


# Verdad del campo que el twin le puede inyectar al mapa digital (el carro real
# no la tiene: el sentido sale del dir= del ACK y la recta del cajón es el
# marco del mapa, ver config.DIGITAL_MAP_*).
_MAP_TRUTH_KEYS = {
    "parking": ("DIGITAL_MAP_PARKING", "parking_section"),
    "direction": ("DIGITAL_MAP_DIRECTION", "direction"),
}
_OFF = ("", "0", "false", "no", "off", "none")
_ON = ("1", "true", "yes", "on", "all")


def map_truth_keys(spec: Any) -> tuple[str, ...]:
    """Qué verdad se inyecta: True/False, o "0"/"1"/"parking,direction"."""
    if spec is None or spec is True:
        return tuple(_MAP_TRUTH_KEYS)
    if spec is False:
        return ()
    s = str(spec).strip().lower()
    if s in _OFF:
        return ()
    if s in _ON:
        return tuple(_MAP_TRUTH_KEYS)
    keys = tuple(k.strip() for k in s.split(",") if k.strip())
    bad = [k for k in keys if k not in _MAP_TRUTH_KEYS]
    if bad:
        raise ValueError(f"DIGITAL_MAP_INJECT_TRUTH: claves desconocidas {bad}")
    return keys


def _sign_from_hit(field, hit: str):
    """Señal del campo que nombra world.collision ("señal Red S/T2"), o None."""
    if not hit.startswith("señal "):
        return None
    try:
        _w, color, where = hit.split(" ", 2)
        section, seat = where.split("/")
    except ValueError:
        return None
    for s in field.signs:
        if s.color == color and s.section == section and s.seat == seat:
            return s
    return None


def _pose_at(history: "deque[tuple[float, float, float, float]]", t: float) -> tuple[float, float, float]:
    if not history:
        return 0.0, 0.0, 0.0
    if t <= history[0][0]:
        return history[0][1], history[0][2], history[0][3]
    if t >= history[-1][0]:
        return history[-1][1], history[-1][2], history[-1][3]
    # t_cam = t_k - cam_latency_s cae a pocos ms del final de history (hasta
    # 20 s de ventana): recorrer desde el final encuentra el intervalo en un
    # puñado de pasos en vez de barrer miles de entradas viejas. Mismo
    # resultado bit a bit (ver nota de empate en el PR): intervalos
    # consecutivos comparten el punto en el límite, así que da igual desde
    # qué lado se llegue.
    t1, x1, y1, h1 = history[-1]
    for i in range(len(history) - 2, -1, -1):
        t0, x0, y0, h0 = history[i]
        if t0 <= t <= t1:
            u = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            return (
                x0 + u * (x1 - x0),
                y0 + u * (y1 - y0),
                h0 + u * (h1 - h0),
            )
        t1, x1, y1, h1 = t0, x0, y0, h0
    return history[-1][1], history[-1][2], history[-1][3]


class Sim:
    def __init__(
        self,
        seed: int,
        start: str | None = None,
        *,
        preset: str | None = None,
        fw_source: str | None = None,
        fw_overrides: dict[str, str] | None = None,
        fw_defines: dict[str, str] | None = None,
        pi_overrides: dict[str, Any] | None = None,
        calib_npz: Path | str | None = _DEFAULT_CALIB,
        params: TwinParams | None = None,
        max_time_s: float = 180.0,
        pi_period_s: float | None = None,
        cam_latency_s: float | None = None,
        log_dir: Path | str | None = None,
        knock_signs: bool = False,
        speed_scale: float = 1.0,
        field: Any = None,
        fw_params: dict[str, float] | None = None,
        ignore_collisions: tuple[str, ...] = (),
        inject_map_truth: Any = None,
        park_pos: Any = None,
    ) -> None:
        # Tocar una lata termina la corrida, igual que una pared. knock_signs
        # solo sirve para medir el resto de la vuelta con la lata ya derribada.
        self.knock_signs = knock_signs
        # Causas que no detienen la corrida (p. ej. "pared exterior" en el
        # harness de estacionamiento); se registran en metrics["ignored_contacts"].
        self.ignore_collisions = tuple(ignore_collisions)
        # Escenario armado a mano (p. ej. twin/prepark.py) en vez del sorteo, y
        # parámetros SIL de arranque (sil_set_param) para el firmware.
        self.field_override = field
        self.fw_params = dict(fw_params or {})
        cfg = resolve_preset(
            preset,
            fw_source=fw_source,
            fw_overrides=fw_overrides,
            fw_defines=fw_defines,
            pi_overrides=pi_overrides,
            start=start,
        )
        self.seed = seed
        self.start = cfg.get("start", start)
        self.fw_source = cfg.get("fw_source", "worktree")
        self.fw_overrides = dict(cfg.get("fw_overrides", {}))
        self.fw_defines = dict(cfg.get("fw_defines", {}))
        self.pi_overrides = dict(cfg.get("pi_overrides", {}))
        self.imu = cfg.get("imu", "mpu6050")
        # Verdad inyectada al mapa digital: argumento > env
        # DIGITAL_MAP_INJECT_TRUTH > preset "inject_map_truth" > todo (como antes).
        spec = inject_map_truth
        if spec is None:
            spec = os.environ.get("DIGITAL_MAP_INJECT_TRUTH")
        if spec is None:
            spec = cfg.get("inject_map_truth", True)
        self.inject_map_truth = map_truth_keys(spec)
        # Dónde va el cajón en la recta (wro_field.park_w_of): argumento > env
        # FOX_PARK_POS > preset "park_pos" > fijo (w=1000, el programa oficial).
        # La Pi NO lo recibe: lo tiene que descubrir (route_mode, arranque ciego).
        if park_pos is None:
            park_pos = os.environ.get("FOX_PARK_POS") or cfg.get("park_pos")
        self.park_pos = park_pos
        self.params = params or TwinParams()
        # Relación de engranes (estudio): TWIN_SPEED_SCALE escala mm/s por PWM y
        # TWIN_TAU_SCALE la constante de tiempo de arranque (menos torque = más lento).
        speed_scale = float(os.environ.get("TWIN_SPEED_SCALE", speed_scale))
        self.speed_scale = speed_scale
        if speed_scale != 1.0:
            self.params.motor.k_mm_s_per_pwm *= speed_scale
            self.params.motor.k_rev_mm_s_per_pwm *= speed_scale
        tau_scale = float(os.environ.get("TWIN_TAU_SCALE", 1.0))
        if tau_scale != 1.0:
            self.params.motor.tau_drive_s *= tau_scale
        # Motor de hoja de datos (estudio): TWIN_MOTOR=fisico reemplaza k/pwm0/tau y pone tope de aceleración.
        self.motor_fisico: dict[str, float] | None = None
        if os.environ.get("TWIN_MOTOR") == "fisico":
            env_kw = {"mass_kg": "TWIN_MASS_KG", "gear": "TWIN_GEAR", "mu": "TWIN_MU", "v_batt": "TWIN_VBATT"}
            kw = {k: float(os.environ[e]) for k, e in env_kw.items() if e in os.environ}
            self.motor_fisico = physical_motor(**kw)
            for key, val in self.motor_fisico.items():
                setattr(self.params.motor, key, val)
        self.max_time_s = max_time_s
        if pi_period_s is None:
            pi_period_s = cfg.get("pi_period_s", self.params.timing.pi_proc_s)
        self.pi_period_s = pi_period_s
        self.cam_latency_s = cam_latency_s if cam_latency_s is not None else self.params.timing.cam_latency_s
        self.log_dir = Path(log_dir) if log_dir else None
        self.calib_npz = None if calib_npz is None else Path(calib_npz)
        self.camera_report: dict[str, Any] | None = None

    def run(
        self,
        frame_callback: Callable[[SimFrameView], bool] | None = None,
    ) -> RunResult:
        t_wall0 = time.perf_counter()
        # Antes de armar la homografía: BEV_H y ROBOT_BEV_Y se leen ahí.
        saved_cfg: dict[str, Any] = {}
        for k, v in self.pi_overrides.items():
            if hasattr(C, k):
                saved_cfg[k] = getattr(C, k)
                setattr(C, k, v)
        # TWIN_NOISE_SEED: otra realización del ruido de sensores con la misma pista (sin la variable = igual que antes).
        _ns = os.environ.get("TWIN_NOISE_SEED")
        rng = np.random.default_rng(self.seed if _ns is None else [self.seed, int(_ns)])
        field = self.field_override if self.field_override is not None else randomize(self.seed, start=self.start, park_pos=self.park_pos)
        for key in self.inject_map_truth:
            k, attr = _MAP_TRUTH_KEYS[key]
            if k not in saved_cfg and hasattr(C, k):
                saved_cfg[k] = getattr(C, k)
            setattr(C, k, getattr(field, attr))
        if len(self.inject_map_truth) < len(_MAP_TRUTH_KEYS):
            print(f"[twin] mapa digital: verdad inyectada {self.inject_map_truth or 'ninguna'}"
                  f" (campo {field.parking_section}/{field.direction})", flush=True)
        world = World(field)
        veh = Vehicle(
            self.params,
            field.start_x,
            field.start_y,
            field.start_heading,
        )
        enc = Encoder(self.params.encoder, rng)
        gyro = Gyro(self.params.gyro, rng)
        bno = Bno085Heading(self.params.bno, rng) if self.imu == "bno085" else None
        sonar = Ultrasonic(self.params.ultrasonic, rng)
        tof = ToF(self.params.tof, rng)
        w_max = max_wheel_deg(self.params.steering)

        # Montaje de la foto, no el PnP del .npz (ese daba ~25° con un lente inventado).
        cam = CameraModel(self.params.camera, self.params)
        bev, bev_err = cam.build_bev()
        self.camera_report = {
            "tilt_deg": cam.params.tilt_deg,
            "hfov_deg": cam.params.hfov_deg,
            "height_mm": cam.params.height_mm,
            "forward_from_rear_axle_mm": cam.params.forward_from_rear_axle_mm,
            "bev_fit_err_px": bev_err,
        }
        print(
            f"[CAM] tilt={cam.params.tilt_deg:.1f}° "
            f"hfov={cam.params.hfov_deg:.1f}° "
            f"h={cam.params.height_mm:.0f}mm "
            f"adelante={cam.params.forward_from_rear_axle_mm:.0f}mm "
            f"bev_err={bev_err:.2f}px "
            f"lente={'equidistante' if cam.params.equidistant else 'pinhole'} "
            f"bev={C.BEV_W}x{C.BEV_H} robot_y={C.ROBOT_BEV_Y} "
            f"vel x{self.speed_scale:.2f}"
            + (f" motor fisico k={self.motor_fisico['k_mm_s_per_pwm']:.2f}"
               f" pwm0={self.motor_fisico['pwm0']:.0f}"
               f" tau={self.motor_fisico['tau_drive_s']:.3f}" if self.motor_fisico else ""),
            flush=True,
        )

        fw = FirmwareSIL(
            overrides=self.fw_overrides,
            defines=self.fw_defines,
            source=self.fw_source,
        )
        for k, v in self.fw_params.items():
            fw.set_param(k, float(v))
        link = SimSerialLink(fw)

        # deque(maxlen=...) descarta el extremo viejo en O(1); con list +
        # pop(0) cada descarte movía hasta 20000 punteros (memmove) y era el
        # mayor hotspot medido en profile (~31 s de 234 s con cProfile en una
        # carrera de 89 s simulados: 549729 llamadas a list.pop).
        history: "deque[tuple[float, float, float, float]]" = deque(maxlen=20000)
        last_yaw_rate = 0.0
        last_ds = 0.0
        last_coll_check = 0.0
        trace: list[dict[str, Any]] = []
        last_trace_t = -1.0
        collision_cause: str | None = None
        collision_t: float | None = None
        sign_contacts: list[dict[str, Any]] = []
        ignored_contacts: dict[str, dict[str, Any]] = {}
        stop_reason = "max_time"
        armed_t: float | None = None
        fw_debug = ""
        stopped_since: float | None = None
        progress_anchor_t = 0.0
        progress_anchor_xy = (veh.x, veh.y)
        terminado_seen = False

        def _t_s() -> float:
            return fw.now_us / 1e6

        def _log_trace(extra: dict[str, Any]) -> None:
            nonlocal last_trace_t
            t = _t_s()
            if t - last_trace_t < 0.04 and not extra.get("pi_frame"):
                return
            last_trace_t = t
            row = {
                "t": t,
                "x": veh.x,
                "y": veh.y,
                "heading": veh.heading_deg,
                "ds_mm": last_ds,
                "est": extra.get("est"),
                "ack": extra.get("ack", ""),
                "motor_pwm": fw.motor_pwm(),
                "motor_dir": fw.motor_dir(),
                "servo": fw.servo_angle(),
                "speed_mm_s": abs(veh.v),
            }
            row.update(extra)
            trace.append(row)

        # Locales para advance (~566k llamadas por corrida).
        veh_substep, enc_update = veh._integrate_substep, enc.update
        servo_angle, motor_pwm, motor_dir = fw.servo_angle, fw.motor_pwm, fw.motor_dir
        sil_now_us, hist_append = fw._lib.sil_now_us, history.append
        st_params = self.params.steering

        def advance(us: int) -> None:
            nonlocal last_yaw_rate, last_ds, last_coll_check, collision_cause, collision_t, stop_reason, world
            dt_total = us / 1e6
            remaining = dt_total
            while remaining > 1e-12:
                sub = min(remaining, 0.001)
                remaining -= sub
                # = veh.step(sub, ...) con sub <= 1 ms (un solo sub-paso), sin StepInfo.
                h0, v0 = veh.heading_deg, veh.v
                veh_substep(sub, servo_angle(), float(motor_pwm()), motor_dir())
                last_ds = 0.0 + veh.v * sub
                last_yaw_rate = -(veh.heading_deg - h0) / sub
                enc_update(last_ds, wheel_deg_from_servo(veh.servo_deg, st_params), (veh.v - v0) / sub, w_max)
                t = int(sil_now_us()) / 1e6
                hist_append((t, veh.x, veh.y, veh.heading_deg))
                if t - last_coll_check >= _COLLISION_INTERVAL_S:
                    last_coll_check = t
                    hit = collision(
                        world,
                        veh.x,
                        veh.y,
                        veh.heading_deg,
                        self.params.vehicle.length_mm,
                        self.params.vehicle.width_mm,
                        self.params.vehicle.rear_overhang_mm,
                        ignore=self.ignore_collisions,
                    )
                    if self.ignore_collisions and collision_cause is None:
                        for cause in self.ignore_collisions:
                            if cause not in ignored_contacts and collision(
                                    world, veh.x, veh.y, veh.heading_deg,
                                    self.params.vehicle.length_mm, self.params.vehicle.width_mm,
                                    self.params.vehicle.rear_overhang_mm,
                                    ignore=tuple(c for c in self.ignore_collisions if c != cause),
                            ) == cause:
                                ignored_contacts[cause] = {"t": round(t, 3), "cause": cause}
                        if "pared exterior" in ignored_contacts:
                            pen = max(max(abs(cx), abs(cy)) - OUTER_HALF_MM for cx, cy in body_corners(
                                veh.x, veh.y, veh.heading_deg, self.params.vehicle.length_mm,
                                self.params.vehicle.width_mm, self.params.vehicle.rear_overhang_mm))
                            c = ignored_contacts["pared exterior"]
                            c["max_mm"] = round(max(c.get("max_mm", 0.0), pen), 1)
                    if hit and collision_cause is None:
                        touched = _sign_from_hit(field, hit) if self.knock_signs else None
                        if touched is not None:
                            field.signs.remove(touched)
                            world = World(field)
                            sign_contacts.append({
                                "t": round(t, 3), "sign": hit,
                                "x": round(veh.x), "y": round(veh.y),
                                "heading": round(veh.heading_deg, 1),
                            })
                        else:
                            collision_cause = hit
                            collision_t = t
                            stop_reason = "collision"

        fw.set_callbacks(
            advance=advance,
            sonar=lambda sid: sonar.echo_us(
                sid, world, Pose(veh.x, veh.y, veh.heading_deg)
            ),
            gyro=(lambda: bno.read(veh.heading_deg, _t_s())) if bno is not None
            else (lambda: gyro.read(last_yaw_rate, _t_s())),
            encoder=lambda: enc.count,
            tof=lambda idx: tof.read_mm(
                idx, world, Pose(veh.x, veh.y, veh.heading_deg), _t_s()
            ),
        )

        stdout_cm = contextlib.nullcontext()
        log_file = None
        if self.log_dir:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            log_file = open(self.log_dir / "pi.log", "w", encoding="utf-8")
            stdout_cm = contextlib.redirect_stdout(log_file)

        runtime: PPRuntime | None = None
        try:
            with stdout_cm:
                runtime = PPRuntime(
                    PPConfig(show_window=False, calib_path=self.calib_npz),
                    vision=Vision(open_cam=False),
                    serial_link=link,
                    bev=bev,
                )
                if getattr(C, "DIGITAL_MAP_SEED_CONFIRMED", False):
                    for s in field.signs:
                        runtime.digital.confirmed[(s.section, s.seat)] = s.color
                    print(
                        f"[twin] mapa: {len(runtime.digital.confirmed)} asientos "
                        f"sembrados (camino conocido)",
                        flush=True,
                    )
                freeze_at_tc = int(getattr(C, "DIGITAL_MAP_FREEZE_AT_TC", 0) or 0)
                map_frozen = bool(getattr(C, "DIGITAL_MAP_SEED_CONFIRMED", False))
                link.open()
                fw.push_serial2("READY")
                fw.setup()

                pi_k = 0
                t_k = _t_s()
                armed = False
                unarmed_left = _UNARMED_FRAMES
                max_us = int(self.max_time_s * 1e6) + fw.now_us
                loops = 0
                # Timing breakdown
                render_s = 0.0
                pi_s = 0.0
                other_s = 0.0

                while fw.now_us < max_us and stop_reason == "max_time":
                    if collision_cause:
                        break
                    t_loop_start = time.perf_counter()
                    fw.loop()
                    link.drain_tx()
                    loops += 1
                    sim_t = _t_s()
                    t_loop_end = time.perf_counter()

                    if sim_t - progress_anchor_t >= _STUCK_WINDOW_S and armed:
                        dx = veh.x - progress_anchor_xy[0]
                        dy = veh.y - progress_anchor_xy[1]
                        if math.hypot(dx, dy) < _STUCK_PROGRESS_MM:
                            stop_reason = "stuck"
                            break
                        progress_anchor_t = sim_t
                        progress_anchor_xy = (veh.x, veh.y)

                    dbg_chunk = fw.pop_debug()
                    if dbg_chunk:
                        fw_debug = (fw_debug + dbg_chunk)[-8000:]
                        if "TERMINADO" in dbg_chunk:
                            terminado_seen = True

                    est = _parse_est(link._last_ack)
                    tc = None
                    tc_s = _parse_ack_field(link._last_ack, "tc")
                    if tc_s is not None:
                        try:
                            tc = int(tc_s)
                        except ValueError:
                            pass
                    tpr_s = _parse_ack_field(link._last_ack, "tpr")
                    tpr = int(tpr_s) if tpr_s else 12

                    if (
                        freeze_at_tc > 0
                        and not map_frozen
                        and tc is not None
                        and tc >= freeze_at_tc
                        and runtime is not None
                        # Esperar recta (no esquina): si se engancha la ruta
                        # a mitad del giro, el índice inicial queda mal.
                        and est in ("S", "C", "R")
                    ):
                        for s in field.signs:
                            runtime.digital.confirmed[(s.section, s.seat)] = s.color
                        if "DIGITAL_MAP_SEED_CONFIRMED" not in saved_cfg and hasattr(
                            C, "DIGITAL_MAP_SEED_CONFIRMED"
                        ):
                            saved_cfg["DIGITAL_MAP_SEED_CONFIRMED"] = getattr(
                                C, "DIGITAL_MAP_SEED_CONFIRMED"
                            )
                        C.DIGITAL_MAP_SEED_CONFIRMED = True
                        # Planificador de vuelta solo si RACING_LINE pide qp/joint.
                        line = str(getattr(C, "RACING_LINE", "") or "").lower()
                        if line in ("qp", "joint") and not getattr(C, "ROUTE_PLANNER", False):
                            if "ROUTE_PLANNER" not in saved_cfg and hasattr(C, "ROUTE_PLANNER"):
                                saved_cfg["ROUTE_PLANNER"] = C.ROUTE_PLANNER
                            C.ROUTE_PLANNER = True
                        runtime.digital._route_key = None
                        runtime.digital._frozen_build_key = None
                        map_frozen = True
                        print(
                            f"[twin] mapa congelado tc={tc} est={est}: "
                            f"{len(runtime.digital.confirmed)} asientos "
                            f"ROUTE={getattr(C, 'ROUTE_PLANNER', False)} "
                            f"line={line or 'build_line'}",
                            flush=True,
                        )

                    spd = abs(veh.v)
                    if spd < _STOPPED_SPEED_MM_S and fw.motor_pwm() < 30:
                        if stopped_since is None:
                            stopped_since = sim_t
                    else:
                        stopped_since = None

                    if armed and terminado_seen and stopped_since and sim_t - stopped_since >= _STOPPED_HOLD_S:
                        stop_reason = "race_finished"
                        break
                    if armed and est == "T" and stopped_since and sim_t - stopped_since >= _STOPPED_HOLD_S:
                        stop_reason = "terminado"
                        break
                    # Estacionando (est=E) el carro se detiene en los settles: no es el final; el final llega con TERMINADO.
                    if armed and tc is not None and tc >= tpr and est != "E" and stopped_since and sim_t - stopped_since >= _STOPPED_HOLD_S:
                        stop_reason = "race_finished"
                        break

                    while sim_t >= t_k + self.pi_period_s:
                        t_end = t_k + self.pi_period_s
                        link.set_now_us(int(t_end * 1e6))
                        t_cam = max(0.0, t_k - self.cam_latency_s)
                        px, py, ph = _pose_at(history, t_cam)
                        t_render0 = time.perf_counter()
                        frame = cam.render_camera(field, px, py, ph)
                        render_s += time.perf_counter() - t_render0
                        t_proc0 = time.perf_counter()
                        # El visor pinta imágenes cada N frames: el callback
                        # puede decir si este las usa (wants_images).
                        want_img = frame_callback is not None and getattr(
                            frame_callback, "wants_images", lambda: True)()
                        result = runtime.process_frame(
                            frame, now=t_end, armed=armed,
                            need_bev_frame=want_img,
                        )
                        pi_ms = (time.perf_counter() - t_proc0) * 1000.0
                        pi_s += pi_ms / 1000.0
                        ack = link.try_readline()
                        if ack:
                            result.serial_ack = ack

                        if not armed:
                            unarmed_left -= 1
                            if unarmed_left <= 0:
                                runtime.arm(sleep_fn=lambda _s: None)
                                # arm() crea un DigitalMap limpio: la siembra
                                # del mapa conocido se repite aquí (si no, se
                                # perdía y SEED_CONFIRMED no tenía efecto).
                                if getattr(C, "DIGITAL_MAP_SEED_CONFIRMED", False):
                                    for s in field.signs:
                                        runtime.digital.confirmed[(s.section, s.seat)] = s.color
                                armed = True
                                armed_t = t_end
                                progress_anchor_t = t_end
                                progress_anchor_xy = (veh.x, veh.y)
                        elif frame_callback:
                            view = SimFrameView(
                                t=t_end,
                                field=field,
                                vehicle=veh,
                                frame_result=result,
                                fw_debug_tail=fw_debug[-500:],
                                est=_parse_est(ack or link._last_ack),
                                tc=tc,
                            )
                            if not frame_callback(view):
                                stop_reason = "user_stop"
                                break

                        _log_trace(
                            {
                                "pi_frame": True,
                                "pi_frame_ms": pi_ms,
                                "est": _parse_est(ack or link._last_ack),
                                "ack": ack or link._last_ack,
                            }
                        )
                        t_k = t_end
                        pi_k += 1
                        sim_t = _t_s()
                        if stop_reason != "max_time":
                            break

                    _log_trace({"est": est, "ack": link._last_ack})

                if runtime._last_heading is None and armed:
                    pass

        except FirmwareSetupError:
            stop_reason = "fw_setup_failed"
            raise
        finally:
            for k, v in saved_cfg.items():
                setattr(C, k, v)
            if log_file:
                log_file.close()
            fw.close()

        metrics = compute_metrics(
            trace,
            field,
            self.params.vehicle,
            armed_t=armed_t,
            stop_reason=stop_reason,
            collision_cause=collision_cause,
            collision_t=collision_t,
        )
        metrics["sign_contacts"] = sign_contacts
        metrics["signs_touched"] = len(sign_contacts)
        metrics["ignored_contacts"] = list(ignored_contacts.values())
        metrics["sim_loops"] = loops
        wall = time.perf_counter() - t_wall0
        # Timing breakdown: accumulate over the whole simulation
        metrics["render_s"] = round(render_s, 3)
        metrics["pi_s"] = round(pi_s, 3)
        metrics["other_s"] = round(max(0.0, wall - render_s - pi_s), 3)
        metrics["wall_time_s"] = round(wall, 3)
        if armed_t is not None and metrics["total_time_s"] > 0:
            metrics["wall_per_sim_s"] = round(wall / metrics["total_time_s"], 3)
        return RunResult(
            metrics=metrics,
            trace=trace,
            field=field,
            camera_report=self.camera_report,
            wall_time_s=wall,
            stop_reason=stop_reason,
        )
