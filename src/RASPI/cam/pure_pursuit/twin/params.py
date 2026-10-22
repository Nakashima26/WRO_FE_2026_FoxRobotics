"""Parámetros del twin Fox — defaults y procedencia (medido vs supuesto)."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Literal

import numpy as np

# Fuente única de dimensiones del carro: config.py (la Pi no puede importar twin/).
from .. import config as C

Provenance = tuple[Literal["medido", "supuesto"], str]

_M = C.SENSOR_MOUNTS

# Campo -> (procedencia, nota breve en español)
PROVENANCE: dict[str, Provenance] = {}


def _prov(key: str, kind: Literal["medido", "supuesto"], note: str) -> None:
    PROVENANCE[key] = (kind, note)


@dataclass
class VehicleParams:
    length_mm: float = C.ROBOT_LENGTH_MM
    width_mm: float = C.ROBOT_WIDTH_MM
    wheelbase_mm: float = C.WHEELBASE_MM
    rear_overhang_mm: float = C.REAR_OVERHANG_MM
    front_overhang_mm: float = C.FRONT_OVERHANG_MM
    track_mm: float = C.TRACK_MM
    wheel_diameter_mm: float = C.WHEEL_DIAMETER_MM

    def __post_init__(self) -> None:
        _prov("vehicle.length_mm", "medido", "180 mm, confirmado por usuario (aprox) — config.py")
        _prov("vehicle.width_mm", "medido", "130 mm, confirmado por usuario (aprox) — config.py")
        _prov("vehicle.wheelbase_mm", "medido", "batalla 113 mm, confirmada por usuario — config.py")
        _prov("vehicle.rear_overhang_mm", "supuesto", "derivado (180-113)/2, pendiente medir")
        _prov("vehicle.front_overhang_mm", "supuesto", "derivado (180-113)/2, pendiente medir")
        _prov("vehicle.track_mm", "supuesto", "solo dibujo; no entra al modelo bicicleta")
        _prov("vehicle.wheel_diameter_mm", "medido", "README llanta 43 mm")


@dataclass
class SteeringParams:
    # Grados de rueda (equivalente bicicleta) por 90° de comando de servo. v2: la
    # cremallera da ~44.5° equivalente a ±71° de comando (README §2.4), igual a los
    # dos lados (mismo piñón). Topes del .ino = SERVO_MIN_DEG/SERVO_MAX_DEG = 19..161 (±71°).
    # Medido 2026-10-21 (p7_radios, calib_fit_steer): 0.474 izq / 0.600 der °rueda por °servo.
    gain_left: float = 42.62
    gain_right: float = 54.04
    # Medido 2026-10-21 (p7_centro/p7_extremos): centro 77, rango útil 2..150 (en 154 la dirección se traba).
    servo_center_deg: float = 77.0
    servo_min_deg: float = 2.0
    servo_max_deg: float = 150.0
    slew_deg_per_s: float = 600.0
    deadband_deg: float = 2.0   # calib 2026-10-21 p7_centro: ±2 de juego alrededor del centro

    def __post_init__(self) -> None:
        _prov("steering.gain_left/right", "medido", "calib 2026-10-21 radios encoder+BNO: rueda máx 41.2°, R mín 129 mm")
        _prov("steering.slew_deg_per_s", "supuesto", "SG90 datasheet 0.1 s/60°")
        _prov("steering.deadband_deg", "medido", "calib 2026-10-21: correcciones de 1-2 no mueven las llantas")


@dataclass
class MotorParams:
    # Medido 2026-10-21 (calib/medidas_20261021, calib_fit_motor, 12.5 V, engrane 1.25):
    # v = k·(PWM − pwm0) con R² 0.997 (80→152, 120→315, 255→856 mm/s).
    k_mm_s_per_pwm: float = 4.02
    pwm0: float = 42.1
    k_rev_mm_s_per_pwm: float = 4.07
    tau_drive_s: float = 0.173
    tau_coast_s: float = 0.091
    # p8_curva: PWM 160 a servo 15 (casi tope) −11%, a servo 40 −2%.
    scrub_coeff: float = 0.12
    # Tope de aceleración por agarre; None = sin tope. 0.8·0.55·g (no medido; las llantas patinan ~1 cm al arrancar).
    a_max_mm_s2: float | None = 4316.0
    # Tope lateral medido (p8_curva_rapida): arriba de esto la curva se abre (subviraje); None = sin tope.
    # Der (servo 5/20) ~2610-2860, izq (servo 134) ~2400.
    a_lat_right_mm_s2: float | None = 2750.0
    a_lat_left_mm_s2: float | None = 2400.0

    def __post_init__(self) -> None:
        _prov("motor.k/pwm0", "medido", "calib 2026-10-21 rampas 20-255 (12.5 V)")
        _prov("motor.tau_drive_s", "medido", "calib 2026-10-21 escalones 90/160/230")
        _prov("motor.tau_coast_s", "medido", "calib 2026-10-21 coast 120/230")
        _prov("motor.scrub_coeff", "medido", "calib 2026-10-21 p8_curva: −11% a servo 15, −2% a 40 (PWM 160)")
        _prov("motor.a_lat_left/right_mm_s2", "medido", "calib 2026-10-21 p8_curva_rapida: 2360-2860 mm/s² según servo")
        _prov("motor.a_max_mm_s2", "supuesto", "mu*fracción en eje trasero*g")


def physical_motor(mass_kg=0.70, gear=1.5, v_batt=12.6, mu=0.8, rear_frac=0.55,
                   eff=0.9, wheel_d_mm=43.0, j_rotor=2.5e-9) -> dict:
    """Motor desde la hoja de datos: N20 50:1 (460 rpm, 0.14 N·m bloqueo, 0.1/1.6 A a 12 V)
    + etapa LEGO `gear`. DC lineal: dv/dt = (v0*d - v)/tau, tau = m_ef*v0/F_bloqueo."""
    r = wheel_d_mm / 2000.0
    v0 = 460.0 * v_batt / 12.0 / gear / 60.0 * math.pi * wheel_d_mm   # mm/s sin carga
    f_s = 0.14 * v_batt / 12.0 * gear * eff / r                         # N en la rueda
    f_int = 0.1 / 1.6                                                   # fricción interna (corriente sin carga)
    f_roll = 0.015 * mass_kg * 9.81 / f_s
    m_ef = mass_kg + j_rotor * (50.0 * gear) ** 2 / r ** 2
    k = v0 / (255.0 * (1.0 - f_int))
    return dict(k_mm_s_per_pwm=k, k_rev_mm_s_per_pwm=k, pwm0=255.0 * (f_int + f_roll),
                tau_drive_s=m_ef * v0 / 1000.0 / f_s, a_max_mm_s2=mu * rear_frac * 9810.0)


@dataclass
class EncoderParams:
    counts_per_mm: float = 5.45   # medido 2026-10-19 (CalibFox x4, enc_runs)
    slip_bias_sigma: float = 0.01
    slip_c_acc: float = 2e-5
    slip_c_acc_cap: float = 0.05
    slip_c_scrub: float = 0.02

    def __post_init__(self) -> None:
        _prov("encoder.counts_per_mm", "medido", "calib 2026-10-19 enc_runs (CalibFox x4): 5.45 cuentas/mm")
        _prov("encoder.slip_*", "supuesto", "deriva multiplicativa por acel/scrub")


@dataclass
class GyroParams:
    bias_sigma_dps: float = 0.1
    bias_walk_dps_per_sqrt_s: float = 0.02
    white_noise_dps: float = 0.15
    scale_error_sigma: float = 0.01

    def __post_init__(self) -> None:
        _prov("gyro.noise", "supuesto", "MPU6050 post-calibración; sin deadzone |gz|<1 del .ino")


@dataclass
class BnoParams:
    """IMU del hardware nuevo: BNO085, Game Rotation Vector (sin magnetómetro).

    El yaw ya viene fusionado en el chip: casi sin error de escala y con una
    deriva lenta (~0.5°/min, caminata). La salida se refresca a update_hz.
    """
    scale_error_sigma: float = 0.001
    drift_walk_deg_per_sqrt_s: float = 0.065   # ~0.5° de desvío en 1 min
    noise_deg: float = 0.15                    # Gauss-Markov lento (no ruido blanco)
    noise_tau_s: float = 0.5
    update_hz: float = 200.0

    def __post_init__(self) -> None:
        _prov("imu.bno085", "supuesto", "datasheet BNO08x (Game Rotation Vector), medir en pista")


@dataclass
class UltrasonicMount:
    right_mm: float
    forward_mm: float
    direction_deg: float


@dataclass
class UltrasonicParams:
    max_range_mm: float = 2000.0
    cone_half_deg: float = 15.0
    ray_step_deg: float = 1.0
    max_incidence_deg: float = 20.0
    noise_sigma_mm: float = 3.0
    noise_rel: float = 0.005
    sound_mm_per_us: float = 0.343
    # Orden = id del sonar en el SIL (0=L 1=R 2=F). Fuente: config.SENSOR_MOUNTS.
    # A 15 mm del costado (chasis 130). Si el transductor queda a <1 cm de la
    # pared, leerDistancia trunca a 0 cm y lo convierte en 200.
    mounts: tuple[UltrasonicMount, ...] = tuple(
        UltrasonicMount(*_M[k]) for k in ("us_left", "us_right", "us_front")
    )

    def __post_init__(self) -> None:
        _prov("ultrasonic.mounts", "supuesto", "laterales a 15 mm del costado; frontal al morro")
        _prov("ultrasonic.cone", "supuesto", "HC-SR04 ~30° total; README guiñada >30°")


@dataclass
class ToFMount:
    right_mm: float
    forward_mm: float
    direction_deg: float


@dataclass
class ToFParams:
    half_fov_deg: float = 13.5
    ray_step_deg: float = 1.5
    # Pared negra visible a 80 cm en Short y a 150 cm en Long (calib 2026-10-21 tof); 250 era del VL53L0X.
    black_wall_max_mm: float = 1500.0
    # Lectura medida contra distancia real, pared negra, Short ROI 16 (lo que usa el firmware en L/R/atrás):
    # +19 mm cerca y desde ~60 cm el cono ve el piso (800 -> 610, se satura). Solo para los ids de abajo.
    short_map_true_mm: tuple[float, ...] = (100.0, 200.0, 300.0, 450.0, 600.0, 800.0)
    short_map_read_mm: tuple[float, ...] = (119.0, 217.0, 319.0, 462.0, 550.0, 610.0)
    short_map_ids: tuple[int, ...] = (0, 1, 2)
    max_range_mm: float = 4000.0
    noise_sigma_mm: float = 5.0
    noise_rel: float = 0.01
    sample_period_s: float = 1.0 / 30.0
    # Orden = índice de tofMm[] (0=L 1=R 2=atrás 3=frente). Firmware v2: L/R/atrás
    # en Short (50 Hz, ~1.3 m); frente en Long (30 Hz, 4 m). Sin medir en el carro.
    sample_period_s_by_id: tuple[float, ...] = (0.020, 0.020, 0.020, 1.0 / 30.0)
    max_range_mm_by_id: tuple[float, ...] = (1300.0, 1300.0, 1300.0, 4000.0)
    reflectivity: dict[str, float] = field(
        default_factory=lambda: {"wall": 0.04, "magenta": 0.6, "sign": 0.5}
    )
    # Orden = índice de tofMm[] en el firmware (0=L 1=R 2=atrás 3=frente).
    # Fuente: config.SENSOR_MOUNTS. El frontal es del hardware nuevo.
    mounts: tuple[ToFMount, ...] = tuple(
        ToFMount(*_M[k]) for k in ("tof_left", "tof_right", "tof_rear", "tof_front")
    )

    def __post_init__(self) -> None:
        _prov("tof.reflectivity", "supuesto", "VL53 en pared negra vs magenta vs señal")
        _prov("tof.black_wall_max_mm", "medido", "calib 2026-10-21 tof: negro visible a 800 (Short) y 1500 mm (Long)")
        _prov("tof.short_map", "medido", "calib 2026-10-21 lateral izq Short ROI 16 50 ms, todo status 0; der/atrás sin medir (misma curva)")
        _prov("tof.mounts", "supuesto", "L/R/atrás/frente; frente nuevo, espejo del trasero; pendiente medir")
        _prov("tof.rate_range", "supuesto", "firmware v2: L/R/atrás Short 20 ms (50 Hz, ~1.3 m), frente Long 33 ms (30 Hz, 4 m); ST: modo Short máx 50 Hz y ~1.3 m; sin medir en el carro")


@dataclass
class CameraParams:
    width: int = 640
    height: int = 480
    # Lente NoIR ancho del carro (README), no el pinhole que el PnP inventaba (~73°).
    hfov_deg: float = 120.0
    # Soporte fijo de la foto: el lente mira para adelante, ~45° bajo el horizonte.
    tilt_deg: float = C.CAMERA_TILT_DEG
    # Lente por encima del techo. 90 + 7 mm: la LiPo 3S (~24 mm acostada)
    # reemplaza a la Pi 4 (~17 mm) en el centro. No está medido con regla.
    height_mm: float = C.CAMERA_HEIGHT_MM
    # En el morro, a la altura del sonar frontal. El lente de la foto queda ahí.
    forward_from_rear_axle_mm: float = C.CAMERA_FWD_MM
    right_mm: float = C.CAMERA_RIGHT_MM
    # Rotación cámara -> robot (derecha, adelante, arriba) de una calibración
    # real. Si es None se arma solo con tilt_deg (sin roll ni yaw).
    rotation_cam_to_robot: tuple | None = None
    k1: float = 0.0
    k2: float = 0.0
    # 3x3 de la imagen YA enderezada (newK de lens_calib.npz). None = hfov.
    K: tuple | None = None
    # Equidistante (r = f·θ) pone el horizonte en el borde y el verde pasa
    # el corte de 1000 px hasta ~80 cm. En la carrera mueve el pie de la
    # lata y el primer rojo del oeste se come a los 10 s. Apagado: el BEV
    # largo ya muestra el carril y la config de 50° sigue terminando.
    equidistant: bool = False
    wall_height_mm: float = 100.0
    sign_height_mm: float = 100.0

    def __post_init__(self) -> None:
        _prov("camera.hfov", "medido", "NoIR ancho ~120° (README); el PnP del .npz no se usa")
        _prov("camera.tilt_deg", "medido", "45° hacia el piso, confirmado por usuario")
        _prov("camera.height_mm", "supuesto", "97 mm = 90 + 7 (LiPo vs Pi4), medir con regla")
        _prov(
            "camera.forward_from_rear_axle_mm",
            "supuesto",
            "140 mm, en el morro, a la altura del sonar frontal",
        )
        if self.K is None and os.environ.get("TWIN_LENS_CALIB"):
            lens = np.load(os.environ["TWIN_LENS_CALIB"])
            self.K = tuple(map(tuple, lens["newK"].tolist()))
            self.width, self.height = (int(v) for v in lens["size"])
            _prov("camera.K", "medido", "lens_calib.npz (tablero)")


@dataclass
class TimingParams:
    pi_proc_s: float = 0.07
    cam_latency_s: float = 0.04
    uart_baud: int = 115200

    def __post_init__(self) -> None:
        _prov("timing.pi_proc_s", "supuesto", "pipeline ~14 fps")
        _prov("timing.uart_baud", "medido", "PurePursuit.ino Serial2")


@dataclass
class TwinParams:
    vehicle: VehicleParams = field(default_factory=VehicleParams)
    steering: SteeringParams = field(default_factory=SteeringParams)
    motor: MotorParams = field(default_factory=MotorParams)
    encoder: EncoderParams = field(default_factory=EncoderParams)
    gyro: GyroParams = field(default_factory=GyroParams)
    bno: BnoParams = field(default_factory=BnoParams)
    ultrasonic: UltrasonicParams = field(default_factory=UltrasonicParams)
    tof: ToFParams = field(default_factory=ToFParams)
    camera: CameraParams = field(default_factory=CameraParams)
    timing: TimingParams = field(default_factory=TimingParams)

    def describe(self) -> list[str]:
        """Líneas para imprimir al arrancar el twin (estilo INVENTED)."""
        lines: list[str] = []
        for key, (kind, note) in sorted(PROVENANCE.items()):
            tag = "MEDIDO" if kind == "medido" else "SUPUESTO"
            lines.append(f"[{tag}] {key}: {note}")
        return lines


def wheel_deg_from_servo(servo_deg: float, steering: SteeringParams) -> float:
    """Rueda + = derecha; servo > 90 = izquierda (firmware)."""
    delta = steering.servo_center_deg - servo_deg
    if delta >= 0.0:
        return delta * steering.gain_right / 90.0
    return delta * steering.gain_left / 90.0


def max_wheel_deg(steering: SteeringParams) -> float:
    w_min = wheel_deg_from_servo(steering.servo_min_deg, steering)
    w_max = wheel_deg_from_servo(steering.servo_max_deg, steering)
    return max(abs(w_min), abs(w_max))


def steady_speed_mm_s(
    pwm: float,
    wheel_deg: float,
    motor: MotorParams,
    steering: SteeringParams,
) -> float:
    if pwm <= motor.pwm0:
        return 0.0
    v = motor.k_mm_s_per_pwm * (pwm - motor.pwm0)
    w_max = max_wheel_deg(steering)
    if w_max > 1e-6:
        v *= 1.0 - motor.scrub_coeff * (abs(wheel_deg) / w_max) ** 2
    return max(0.0, v)
