"""Parámetros del twin Fox — defaults y procedencia (medido vs supuesto)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

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
    # Grados de rueda por 90° de comando de servo. Default: el comando son las
    # unidades internas del firmware (topes 30 y 160), presets sin FOX_SERVO_180.
    # El carro nuevo (hw_nuevo) usa steering_servo_180(): valor de servo 0-180.
    # El CAD de la mangueta mide 46.32° en ese recorrido (antes el twin usaba 50°).
    gain_left: float = C.MAX_WHEEL_STEER_DEG * 90.0 / 70.0   # servo 160 (70° de comando) → 46.32°
    gain_right: float = C.MAX_WHEEL_STEER_DEG * 90.0 / 60.0  # servo 30 (60° de comando) → 46.32°
    servo_center_deg: float = 90.0
    servo_min_deg: float = 30.0
    servo_max_deg: float = 160.0
    slew_deg_per_s: float = 600.0
    deadband_deg: float = 0.5
    # Slew por lado (°/s de comando). None en ambos = slew_deg_per_s a los dos
    # lados (camino de siempre, bit a bit). El lado lo da la POSICIÓN del servo:
    # > servo_center_deg = izquierda; al cruzar el centro cambia la tasa.
    slew_left_deg_per_s: float | None = None
    slew_right_deg_per_s: float | None = None
    # Banda muerta por lado (° de comando), mismo criterio de lado. None en ambos
    # (y sin slew por lado) = deadband_deg en el camino de siempre.
    deadband_left_deg: float | None = None
    deadband_right_deg: float | None = None

    def __post_init__(self) -> None:
        _prov(
            "steering.gain_left/right",
            "supuesto",
            "46.32° es del CAD (usuario); que se alcance justo en servo 30/160 con centro 90 "
            "sale de los topes del firmware del carro viejo (fc2bc3e). El carro nuevo "
            "(hw_nuevo) va con steering_servo_180()",
        )
        _prov("steering.slew_deg_per_s", "supuesto", "SG90 datasheet 0.1 s/60°")
        _prov("steering.deadband_deg", "supuesto", "micro-juego mecánico")


# ── Carro nuevo: valor de servo 0-180 (FOX_SERVO_180=1 en el .ino) ────────────
# Las maniobras del firmware siguen en unidades internas 30..160 (centro 90) y
# escribirServo() las lleva a valor de servo 0-180 por tramos: 30 -> 0,
# 90 -> 90, 160 -> 180. Supuestos sin medir (usuario: el servo recorre 0-180):
# ruedas rectas en 90 y ±46.32° de rueda en 0/180, lineal y simétrico.
SERVO_INT_MIN = 30.0
SERVO_INT_CENTRO = 90.0
SERVO_INT_MAX = 160.0
# Slew de siempre en unidades internas (600 °/s). "proporcional" lo escala por
# lado al valor de servo 0-180 (misma tasa de rueda que con 30..160: ~397 °/s
# izq, ~463 °/s der); "plano600" son 600 °/s de valor de servo (~309 °/s de rueda).
SLEW_INTERNO_DEG_S = 600.0
SLEW_180_OPCIONES = ("proporcional", "plano600")


def servo_180_desde_interno(v: float) -> float:
    """Unidades internas (30..160) -> valor de servo 0-180, sin el redondeo del pulso."""
    v = min(SERVO_INT_MAX, max(SERVO_INT_MIN, v))
    if v >= SERVO_INT_CENTRO:
        return 90.0 + (v - SERVO_INT_CENTRO) * 90.0 / (SERVO_INT_MAX - SERVO_INT_CENTRO)
    return 90.0 - (SERVO_INT_CENTRO - v) * 90.0 / (SERVO_INT_CENTRO - SERVO_INT_MIN)


def servo_interno_desde_180(s: float) -> float:
    """Valor de servo 0-180 -> unidades internas (30..160). Inversa de la anterior."""
    if s >= 90.0:
        return SERVO_INT_CENTRO + (s - 90.0) * (SERVO_INT_MAX - SERVO_INT_CENTRO) / 90.0
    return SERVO_INT_CENTRO - (90.0 - s) * (SERVO_INT_CENTRO - SERVO_INT_MIN) / 90.0


def pulso_us_servo_180(v: int) -> int:
    """Pulso (µs) que escribe escribirServo() con FOX_SERVO_180=1 (misma aritmética entera)."""
    v = min(int(SERVO_INT_MAX), max(int(SERVO_INT_MIN), int(v)))
    d = v - int(SERVO_INT_CENTRO)
    return 1500 + (d * 1000 + 35) // 70 if d >= 0 else 1500 - (-d * 1000 + 30) // 60


def steering_servo_180(slew: str = "proporcional") -> SteeringParams:
    """SteeringParams del carro nuevo (hw_nuevo): el comando es el valor de servo 0-180."""
    if slew not in SLEW_180_OPCIONES:
        raise ValueError(f"slew desconocido: {slew} (opciones: {SLEW_180_OPCIONES})")
    lado_izq = 90.0 / (SERVO_INT_MAX - SERVO_INT_CENTRO)   # 90/70
    lado_der = 90.0 / (SERVO_INT_CENTRO - SERVO_INT_MIN)   # 90/60
    db = SteeringParams.deadband_deg   # 0.5 de unidades internas
    st = SteeringParams(
        gain_left=C.MAX_WHEEL_STEER_DEG,
        gain_right=C.MAX_WHEEL_STEER_DEG,
        servo_center_deg=90.0,
        servo_min_deg=0.0,
        servo_max_deg=180.0,
        slew_deg_per_s=SLEW_INTERNO_DEG_S,
        slew_left_deg_per_s=SLEW_INTERNO_DEG_S * lado_izq if slew == "proporcional" else None,
        slew_right_deg_per_s=SLEW_INTERNO_DEG_S * lado_der if slew == "proporcional" else None,
        # Banda muerta escalada igual que el recorrido (0.5·90/70 y 0.5·90/60 de
        # valor de servo): la misma en rueda que con 30..160 (0.331° izq, 0.386° der).
        # Con 0.5 plano la rueda paraba más cerca del objetivo (0.257°) y el twin
        # cambiaba de comportamiento sin que cambiara el carro.
        deadband_left_deg=db * lado_izq,
        deadband_right_deg=db * lado_der,
    )
    _prov(
        "steering(servo 0-180).gain",
        "supuesto",
        "46.32° (CAD, usuario) en valor de servo 0 y 180, centro 90, lineal y simétrico; "
        "el servo recorre 0-180 (usuario). Medir: valor con ruedas rectas, rueda en 0/180, "
        "radio a tope por lado",
    )
    if slew == "proporcional":
        _prov(
            "steering(servo 0-180).slew",
            "supuesto",
            "supuesto — escalado a proporción del rango viejo (decisión del usuario 2026-10-05); "
            "medir el servo. 600·90/70≈771.4 °/s izq, 600·90/60=900 °/s der de valor de servo "
            "(misma tasa de rueda que con 30..160)",
        )
    else:
        _prov(
            "steering(servo 0-180).slew",
            "supuesto",
            "600 °/s plano de valor de servo (sensibilidad; ~309 °/s de rueda); medir el servo",
        )
    _prov(
        "steering(servo 0-180).deadband",
        "supuesto",
        "micro-juego mecánico; 0.5 de unidades internas escalado como el recorrido "
        "(0.643 izq, 0.75 der de valor de servo, misma banda en rueda); medir el servo",
    )
    return st


@dataclass
class MotorParams:
    k_mm_s_per_pwm: float = 3.5
    pwm0: float = 25.0
    k_rev_mm_s_per_pwm: float = 3.5
    tau_drive_s: float = 0.25
    # 100:1 + etapa LEGO: en coast el carro se frena rápido. Con 0.35 s el twin
    # seguía rodando ~10 cm tras el retroceso de MANIOBRA y se comía la pared de
    # atrás en casi todas las esquinas; con 0.12 el baseline completa vueltas.
    tau_coast_s: float = 0.12
    scrub_coeff: float = 0.35

    def __post_init__(self) -> None:
        _prov(
            "motor.k/pwm0",
            "supuesto",
            "~245 mm/s a PWM 95 (firmware 21–31 cm/s); PWM 120 → ~333 mm/s",
        )
        _prov("motor.tau_drive_s", "supuesto", "arranque N20+reductora")
        _prov("motor.tau_coast_s", "supuesto",
              "coast TB6612 A1=A2=0; medir cuánto rueda tras cortar a PWM 100")
        _prov("motor.scrub_coeff", "supuesto", "pérdida por scrub al volante")


@dataclass
class EncoderParams:
    counts_per_mm: float = 20.7
    slip_bias_sigma: float = 0.01
    slip_c_acc: float = 2e-5
    slip_c_acc_cap: float = 0.05
    slip_c_scrub: float = 0.02

    def __post_init__(self) -> None:
        _prov(
            "encoder.counts_per_mm",
            "supuesto",
            "7 PPR×4 (cuadratura x4)×50 (N20)×2 (LEGO) / (π×43) = 20.73 cuentas/mm "
            "(0.0482 mm/cuenta) — encoder del hardware nuevo; recalibrar en pista",
        )
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
    black_wall_max_mm: float = 250.0
    max_range_mm: float = 4000.0
    noise_sigma_mm: float = 5.0
    noise_rel: float = 0.01
    sample_period_s: float = 1.0 / 30.0
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
        _prov("tof.black_wall_max_mm", "medido", "README VL53L0X fuera de rango ~300 mm negro")
        _prov("tof.mounts", "supuesto", "L/R/atrás/frente; frente nuevo, espejo del trasero; pendiente medir")


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
