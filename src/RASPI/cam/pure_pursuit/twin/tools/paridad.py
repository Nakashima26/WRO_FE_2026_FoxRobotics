"""Paridad twin <-> carro real: qué cambia un preset respecto a lo desplegado.

Lee el .ino (#define, const/PARK_AJ y globales con valor, nombres SIL_AJ),
config.py (Pi) y el preset del twin ya resuelto (sim.resolve_preset; hw_nuevo
hereda giro_rapido), más lo que el twin supone por su cuenta (IMU, periodo de
la Pi, verdad inyectada al mapa, servo de params.SteeringParams). Imprime
`nombre | fuente | .ino/config | preset | ¿difiere? | aceptada`.

Cada diferencia tiene que estar en ACEPTADAS con su motivo:
  exit 0 = todas aceptadas, 1 = alguna sin aceptar,
  2 = el preset nombra algo que no existe (el twin lo ignoraría o no compila).
No cambia ningún valor de despliegue ni escribe a disco.

Límites del parseo (regex, no un parser de C++): globals solo a nivel de
archivo (`int x = ...;` al inicio de línea); un #define cuenta como #ifndef-
guardado solo si el #ifndef está en la línea anterior; con nombres repetidos se
usa el primero; `servo.fuera_de_topes` solo ve `centroServo ± var` con
`var = constrain(...)` en las 12 líneas previas.

Uso (desde src/RASPI/cam):
  PYTHONPATH=. python pure_pursuit/twin/tools/paridad.py [preset] [--ino PATH]
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from pure_pursuit import config as C
from pure_pursuit.twin.firmware.build import _CONST_RE, _INO, _INO_GIT, _REPO, load_ino_text
from pure_pursuit.twin.params import TwinParams
from pure_pursuit.twin.sim import map_truth_keys, resolve_preset

_CONFIG_PY = Path(C.__file__)

# Diferencias aceptadas: nombre -> (tipo, motivo).
#   carro     = el carro real todavía no tiene ese hardware/ajuste; cambiarlo
#               en el .ino/config cuando se valide en la pista.
#   twin      = solo del twin, el carro no lo necesita.
#   pendiente = hueco conocido sin resolver (otro frente lo atiende).
ACEPTADAS: dict[str, tuple[str, str]] = {
    "TURNS_PER_RACE": (
        "carro", "el .ino está en TEST de 1 vuelta (4); la carrera son 12 esquinas: "
        "poner 12 antes de competir"),
    "GIRO_RAPIDO_MODO": (
        "carro", "el twin mide el giro rápido (1); el .ino real sigue con MANIOBRA "
        "por tramos (0)"),
    "GIRO_PI_CUENTA_DEG": (
        "carro", "la línea del mapa dobla sola (verde en la boca) y el ESP cuenta la "
        "esquina por gyro; 0 = off en el carro"),
    "PARK_MODO": ("twin", "presets baseline/mapa: arrancan del centro, sin estacionar"),
    "TRACK_MAP_ENABLED": ("carro", "preset mapa: localizador de track_map activo (sin validar en pista)"),
    "FOX_ENCODER": ("carro", "encoder en el motor: hardware nuevo, el .ino real compila sin él"),
    "FOX_TOF": ("carro", "4 ToF (L/R/atrás/frente): hardware nuevo, el .ino real compila sin ellos"),
    "FOX_SERVO_180": (
        "carro", "servo 0-180 del carro nuevo (frente t16servo); el .ino real "
        "(carro viejo) sin el define"),
    "imu": ("carro", "BNO085 en el twin; el .ino real usa MPU6050_tockn (driver BNO085 pendiente, T14)"),
    "pi_period_s": (
        "carro", "el twin fija el periodo de la Pi (hw_nuevo 0.035 = Pi 5 ~28 fps, "
        "supuesto; sin preset, TimingParams.pi_proc_s); el carro corre a lo que dé la "
        "Pi (PI_FPS=14 nominal). Medir fps reales"),
    "PI_FPS": ("carro", "ídem pi_period_s: escala las ventanas en frames"),
    "inject_map_truth": (
        "twin", "el twin le da al mapa digital la verdad del campo; el carro usa dir= "
        "del ACK y el cajón como marco. T16infra (Mac, hw_nuevo, SOLO_CAJON=1, 21 "
        "seeds): con DIGITAL_MAP_INJECT_TRUTH=0 trace.csv y fw_debug.log idénticos"),
    # pi_overrides de giro_rapido/hw_nuevo: ajustes medidos en el twin, sin
    # pasar a config.py hasta probarlos en la pista (motivos de sim.PRESETS).
    "CORNER_HINT_TO_ESP": ("carro", "giro_rapido: la Pi le manda al ESP el hint de esquina"),
    "CORNER_EXTERIOR_PASS_ENABLED": (
        "carro", "giro_rapido: verde en la boca + giro a la derecha, la memoria lo pasa "
        "por la izquierda"),
    "CORNER_EXT_PASS_BAND_PX": (
        "carro", "giro_rapido: el verde de la boca cae ~110 px delante de la naranja; "
        "con 70 se iba a beyond"),
    "CORNER_EXT_PASS_TURN_BLOCK_FRAMES": (
        "carro", "giro_rapido: el bloqueo de 10 frames comía la ventana de 40-95 cm"),
    "CENTERLINE_RAMP_PX": (
        "carro", "giro_rapido: con 130 px y lookahead 100 el path abría encima de la lata"),
    "OBS_INFLATE_R": ("carro", "giro_rapido: cuerpo 65 mm + lata 25 mm no caben en el inflado"),
    "CENTERLINE_TOP_Y": ("carro", "giro_rapido: la línea se cortaba en BEV_H/3 (~49 cm); muestrea hasta ~70 cm"),
    "DIGITAL_MAP_STEER": ("carro", "giro_rapido: la línea la arma el mapa (lado de paso), no el BEV"),
    "DIGITAL_MAP_TURN_HOLD_CM": (
        "carro", "giro_rapido: el giro rápido a 95 cm salía pegado a la isla (seeds 1 y 6)"),
    "DIGITAL_MAP_TURN_HOLD_INNER_CM": ("carro", "giro_rapido: ídem DIGITAL_MAP_TURN_HOLD_CM"),
    "DIGITAL_MAP_PP_TURN_CM": ("carro", "giro_rapido: verde en la boca, la esquina la dobla el PP"),
    "DIGITAL_MAP_WALL_FIX": (
        "carro", "hw_nuevo: la pose del mapa derivaba 150-430 mm; sonares contra paredes conocidas"),
    "servo.fisico": (
        "pendiente", "carro nuevo: servo físico 0-180; el twin modela 30..160 = ±46.32° "
        "de rueda. t16servo agrega FOX_SERVO_180 (30..160 -> 0..180)"),
    "servo.fuera_de_topes": (
        "pendiente", "las U del .ino (PARK/PUNTA) escriben centroServo - ticks con ticks "
        "hasta 70 = 20, fuera de 30..160; escribirServo solo acota a 0..180 y el twin "
        "acota a 30"),
    "servo.rueda_der": (
        "pendiente", "las U del .ino suponen 70 ticks = 46.32° a los dos lados; el twin "
        "pone 46.32° en servo 30 (60 ticks) a la derecha. Sin medir cuál es el real"),
}

# Hardware del carro nuevo que no está en el .ino ni en config (dato del usuario).
CARRO_NUEVO = {
    "servo.fisico": "0..180",
}


@dataclass
class Val:
    val: str
    line: int
    kind: str           # define | const | global
    guarded: bool = False


@dataclass
class InoInfo:
    defines: dict[str, list[Val]] = field(default_factory=dict)
    consts: dict[str, list[Val]] = field(default_factory=dict)
    sil_aj: set[str] = field(default_factory=set)
    includes: list[str] = field(default_factory=list)
    text: str = ""


@dataclass
class Row:
    name: str
    fuente: str
    real: str
    twin: str
    difiere: bool | None    # None = informativo (sin contraparte)
    nota: str = ""


_DEFINE_RE = re.compile(
    r"^[ \t]*#[ \t]*define[ \t]+(?P<name>[A-Za-z_]\w*)\b(?!\()(?P<val>[^\n]*)$", re.M)
_IFNDEF_RE = re.compile(r"^[ \t]*#[ \t]*ifndef[ \t]+(?P<name>[A-Za-z_]\w*)", re.M)
_GLOBAL_RE = re.compile(
    r"^(?:unsigned\s+|signed\s+)?(?:int|long|float|double|bool|char|u?int(?:8|16|32)_t)"
    r"\s+(?P<name>[A-Za-z_]\w*)\s*=\s*(?P<val>[^;]+);", re.M)
_SIL_AJ_RE = re.compile(r"\bSIL_AJ\(\s*(?P<name>[A-Za-z_]\w*)\s*\)\s*;")
_INCLUDE_RE = re.compile(r"^[ \t]*#[ \t]*include[ \t]*[<\"](?P<f>[^>\"]+)[>\"]", re.M)


def _line_of(text: str, pos: int) -> int:
    return text.count("\n", 0, pos) + 1


def _strip_comment(s: str) -> str:
    s = re.sub(r"/\*.*?\*/", "", s)
    return s.split("//", 1)[0].strip()


def parse_ino(text: str) -> InoInfo:
    info = InoInfo(text=text)
    guards = {(m.group("name"), _line_of(text, m.start())) for m in _IFNDEF_RE.finditer(text)}
    for m in _DEFINE_RE.finditer(text):
        name, line = m.group("name"), _line_of(text, m.start())
        g = (name, line - 1) in guards
        info.defines.setdefault(name, []).append(
            Val(_strip_comment(m.group("val")), line, "define", g))
    for m in _CONST_RE.finditer(text):
        info.consts.setdefault(m.group("name"), []).append(
            Val(_strip_comment(m.group("val")), _line_of(text, m.start("name")), "const"))
    for m in _GLOBAL_RE.finditer(text):
        info.consts.setdefault(m.group("name"), []).append(
            Val(_strip_comment(m.group("val")), _line_of(text, m.start("name")), "global"))
    info.sil_aj = {m.group("name") for m in _SIL_AJ_RE.finditer(text)}
    info.includes = [m.group("f") for m in _INCLUDE_RE.finditer(text)]
    return info


def _norm(v: Any) -> Any:
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return float(v)
    s = str(v).strip()
    if s in ("true", "True"):
        return True
    if s in ("false", "False"):
        return False
    t = s.rstrip("fFlLuU")
    try:
        return float(t)
    except ValueError:
        return s


def _same(a: Any, b: Any) -> bool:
    na, nb = _norm(a), _norm(b)
    if isinstance(na, bool) != isinstance(nb, bool):
        return False
    return na == nb


def _fmt(v: Any) -> str:
    if isinstance(v, float) and not v.is_integer():
        return f"{v:.4f}".rstrip("0")
    return repr(v)


def _config_line(name: str) -> int | None:
    m = re.search(rf"^{re.escape(name)}\s*=", _CONFIG_PY.read_text(encoding="utf-8"), re.M)
    return None if m is None else _line_of(m.string, m.start())


def _twin_steering(cfg: dict[str, Any], errors: list[str]) -> tuple[Any, bool]:
    """SteeringParams que usaría Sim con este preset (sigue a FOX_SERVO_180)."""
    from pure_pursuit.twin import params as P
    s180 = str(cfg.get("fw_defines", {}).get("FOX_SERVO_180", "0")) == "1"
    if s180:
        if hasattr(P, "steering_servo_180"):
            slew = os.environ.get("TWIN_SERVO_SLEW") or cfg.get("servo_slew", "proporcional")
            return P.steering_servo_180(slew), True
        errors.append("fw_defines FOX_SERVO_180=1 pero twin/params.py no tiene "
                      "steering_servo_180: el twin seguiría modelando 30..160")
    return TwinParams().steering, False


def _servo_rows(info: InoInfo, cfg: dict[str, Any], errors: list[str]) -> list[Row]:
    from pure_pursuit.twin import params as P
    st, s180 = _twin_steering(cfg, errors)
    lines = info.text.splitlines()
    m_esc = re.search(r"void\s+escribirServo\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}", info.text, re.S)
    esc = (range(_line_of(info.text, m_esc.start()), _line_of(info.text, m_esc.end()) + 1)
           if m_esc else range(0))
    clamps: dict[tuple[int, int], list[int]] = {}
    locks: dict[int, list[int]] = {}
    for i, ln in enumerate(lines, 1):
        code = ln.split("//", 1)[0]
        if "ervo" not in code or i in esc:    # escribirServo va en su propia fila
            continue
        if "constrain(" in code:
            for a, b in re.findall(r",\s*(\d+)\s*,\s*(\d+)\s*\)", code):
                clamps.setdefault((int(a), int(b)), []).append(i)
        for a, b in re.findall(r"\?\s*(\d+)\s*:\s*(\d+)", code):
            for v in (int(a), int(b)):
                locks.setdefault(v, []).append(i)
    vals = [v for p in clamps for v in p] + list(locks)
    lo, hi = (min(vals), max(vals)) if vals else (None, None)
    centro = info.consts.get("centroServo", [Val("?", 0, "global")])[0]
    c = float(_norm(centro.val)) if centro.val != "?" else 90.0
    # Rango interno que modela el twin: SteeringParams sin FOX_SERVO_180; con
    # él, el twin convierte 30..160 internos a valor de servo 0..180.
    int_lo = getattr(P, "SERVO_INT_MIN", 30.0) if s180 else st.servo_min_deg
    int_hi = getattr(P, "SERVO_INT_MAX", 160.0) if s180 else st.servo_max_deg
    int_c = getattr(P, "SERVO_INT_CENTRO", 90.0) if s180 else st.servo_center_deg
    nlock = len({ln for v in locks.values() for ln in v})
    detalle = ", ".join(f"{a}..{b} x{len(v)}" for (a, b), v in sorted(clamps.items()))
    twin_int = (f"{int_lo:g}..{int_hi:g} internos -> 0..180" if s180
                else f"{st.servo_min_deg:g}..{st.servo_max_deg:g} (SteeringParams)")

    rows = [
        Row("servo.topes", "ino",
            f"{lo}..{hi} (constrain {detalle}; {nlock} líneas a tope)", twin_int,
            not (len(clamps) <= 1 and _same(lo, int_lo) and _same(hi, int_hi))),
        Row("servo.centro", "ino", f"centroServo={centro.val} (ino:{centro.line})",
            f"{int_c:g}", not _same(centro.val, int_c)),
    ]

    # Comandos que salen de los topes: centroServo ± var con var acotada poco antes.
    fuera: dict[float, list[int]] = {}
    for i, ln in enumerate(lines, 1):
        code = ln.split("//", 1)[0]
        for var in set(re.findall(r"centroServo\s*[+-]\s*([A-Za-z_]\w*)\b", code)):
            for j in range(i - 1, max(0, i - 12) - 1, -1):
                m = re.search(rf"\b{var}\s*=\s*constrain\(.*,\s*(-?\d+)\s*,\s*(-?\d+)\s*\)\s*;",
                              lines[j].split("//", 1)[0])
                if m:
                    vmax = int(m.group(2))
                    for v in (c - vmax, c + vmax):
                        if v < int_lo or v > int_hi:
                            fuera.setdefault(v, []).append(i)
                    break
    rows.append(Row(
        "servo.fuera_de_topes", "ino",
        "; ".join(f"{v:g} en ino:{','.join(map(str, ls))}" for v, ls in sorted(fuera.items()))
        or "ninguno",
        f"acota a {int_lo:g}..{int_hi:g} ({'params' if s180 else 'vehicle.py'})",
        bool(fuera)))

    if m_esc:
        body = m_esc.group("body")
        cm = re.search(r"constrain\(\s*\w+\s*,\s*(\d+)\s*,\s*(\d+)\s*\)", body)
        mm = re.search(r"map\(\s*\w+\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\)", body)
        real = (f"acota {cm.group(1)}..{cm.group(2)}" if cm else "?") + (
            f", {mm.group(3)}-{mm.group(4)} us" if mm else "")
        rows.append(Row("servo.escribirServo", "ino", f"{real} (ino:{esc.start})", "-", None))

    # Rueda por tick de servo. El .ino lo supone en las U (PARK/PUNTA):
    # ticks = delta / 46.32 * 70 a los DOS lados. El twin: gain_* (° de rueda
    # por 90 de comando) con el comando en unidades internas, o en valor 0-180
    # con FOX_SERVO_180 (un tick interno = 90/70 izq, 90/60 der).
    fm = re.findall(r"constrain\(\s*\(int\)\s*\(\s*\w+\s*/\s*([\d.]+)f?\s*\*\s*([\d.]+)f?\s*\)"
                    r"\s*,\s*-?\d+\s*,\s*-?\d+\s*\)", info.text)
    fl = [_line_of(info.text, mm.start()) for mm in re.finditer(
        r"constrain\(\s*\(int\)\s*\(\s*\w+\s*/\s*[\d.]+f?\s*\*\s*[\d.]+f?\s*\)", info.text)]
    wmax = float(C.MAX_WHEEL_STEER_DEG)
    if len(set(fm)) == 1:
        deg_ino, ticks_ino = (float(x) for x in fm[0])
        where = ",".join(map(str, fl))
        rows.append(Row("servo.rueda_max", f"ino:{where}", f"{deg_ino:g}°",
                        f"MAX_WHEEL_STEER_DEG={wmax:g}° (config)", abs(deg_ino - wmax) > 1e-6))
        k_l = 90.0 / (int_hi - int_c) if s180 else 1.0
        k_r = 90.0 / (int_c - int_lo) if s180 else 1.0
        for lado, g, k in (("izq", st.gain_left, k_l), ("der", st.gain_right, k_r)):
            twin = g * k / 90.0 * ticks_ino
            rows.append(Row(
                f"servo.rueda_{lado}", f"ino:{where}",
                f"{deg_ino:g}° en {ticks_ino:g} ticks",
                f"{twin:.2f}° en {ticks_ino:g} ticks (gain_{'left' if lado == 'izq' else 'right'}"
                f"={g:.2f})", abs(twin - deg_ino) > 0.01))
    else:
        rows.append(Row("servo.rueda_max", "ino", f"fórmula de ticks no encontrada ({len(fm)})",
                        f"MAX_WHEEL_STEER_DEG={wmax:g}°", None))
    sl = getattr(st, "slew_left_deg_per_s", None)
    sr = getattr(st, "slew_right_deg_per_s", None)
    slew = (f"izq {sl:g} / der {sr:g} °/s" if sl is not None and sr is not None
            else f"{st.slew_deg_per_s:g} °/s ambos lados (supuesto SG90)")
    rows.append(Row("servo.slew", "twin", "sin medir", slew, None,
                    "" if s180 else "con FOX_SERVO_180 (t16servo): 900 °/s der, ~771 °/s izq"))
    # Solo el carro nuevo (preset con su hardware) tiene el servo 0-180.
    fd = cfg.get("fw_defines", {})
    if any(str(fd.get(k, "0")) == "1" for k in ("FOX_ENCODER", "FOX_TOF", "FOX_SERVO_180")):
        rango = f"{st.servo_min_deg:g}..{st.servo_max_deg:g}"
        rows.append(Row("servo.fisico", "carro", CARRO_NUEVO["servo.fisico"], rango,
                        CARRO_NUEVO["servo.fisico"] != rango))
    if "FOX_SERVO_180" not in info.defines and "FOX_SERVO_180" not in cfg.get("fw_defines", {}):
        rows.append(Row("FOX_SERVO_180", "ino-define", "(no existe)", "(no definido)", None,
                        "hueco previsto: lo agrega t16servo"))
    return rows


def compare(preset: str = "hw_nuevo", ino_text: str | None = None,
            cfg: dict[str, Any] | None = None) -> tuple[list[Row], list[str]]:
    """Filas de la tabla y errores (nombres que el twin ignoraría)."""
    cfg = resolve_preset(preset) if cfg is None else cfg
    if ino_text is None:
        src = cfg.get("fw_source", "worktree")
        if src == "head":
            # build.load_ino_text("head") decodifica con el locale (cp1252 en
            # Windows) y el .ino es UTF-8: aquí se pide UTF-8 explícito.
            ino_text = subprocess.run(["git", "show", f"HEAD:{_INO_GIT}"], cwd=_REPO,
                                      capture_output=True, check=True).stdout.decode("utf-8")
        else:
            ino_text = load_ino_text(src)
    info = parse_ino(ino_text)
    rows: list[Row] = []
    errors: list[str] = []

    for name, val in cfg.get("fw_overrides", {}).items():
        hits = info.consts.get(name, [])
        hits = [h for h in hits if h.kind == "const"]
        if len(hits) != 1:
            errors.append(f"fw_overrides {name}: {len(hits)} const/PARK_AJ en el .ino (build exige 1)")
            continue
        h = hits[0]
        rows.append(Row(name, f"ino:{h.line}", h.val, str(val), not _same(h.val, val)))

    for name, val in cfg.get("fw_defines", {}).items():
        hits = info.defines.get(name, [])
        if not hits:
            errors.append(f"fw_defines {name}: no hay #define en el .ino (el -D no hace nada)")
            continue
        h = hits[0]
        if not h.guarded:
            errors.append(f"fw_defines {name}: #define sin #ifndef en ino:{h.line} (el -D no gana)")
        rows.append(Row(name, f"ino:{h.line}", h.val, str(val), not _same(h.val, val)))

    for name, val in cfg.get("fw_params", {}).items():
        if name not in info.sil_aj:
            errors.append(f"fw_params {name}: no está en SIL_AJ (sil_param no lo lee)")
            continue
        h = info.consts.get(name, [Val("?", 0, "const")])[0]
        rows.append(Row(name, f"ino:{h.line} SIL_AJ", h.val, str(val), not _same(h.val, val)))

    for name, val in cfg.get("pi_overrides", {}).items():
        if not hasattr(C, name):
            errors.append(f"pi_overrides {name}: no existe en config.py (sim.run lo ignora)")
            continue
        real = getattr(C, name)
        ln = _config_line(name)
        rows.append(Row(name, f"config:{ln}" if ln else "config", _fmt(real), _fmt(val),
                        not _same(real, val)))

    imu_real = "mpu6050" if any("MPU6050" in f for f in info.includes) else (
        "bno085" if any("BNO08" in f for f in info.includes) else "?")
    imu_twin = str(cfg.get("imu", "mpu6050"))
    rows.append(Row("imu", "ino-include", imu_real, imu_twin, imu_real != imu_twin))

    period = float(cfg.get("pi_period_s", TwinParams().timing.pi_proc_s))
    real_p = 1.0 / float(C.PI_FPS)
    rows.append(Row("pi_period_s", f"config:{_config_line('PI_FPS')} 1/PI_FPS",
                    f"{real_p:.4f}", f"{period:.4f}", abs(real_p - period) > 1e-6))

    spec = os.environ.get("DIGITAL_MAP_INJECT_TRUTH")
    if spec is None:
        spec = cfg.get("inject_map_truth", True)
    keys = map_truth_keys(spec)
    rows.append(Row("inject_map_truth", "twin", "ninguna (dir= del ACK)",
                    ",".join(keys) or "ninguna", bool(keys)))

    rows.extend(_servo_rows(info, cfg, errors))
    return rows, errors


def render(rows: list[Row], errors: list[str], preset: str) -> tuple[str, int]:
    out = [f"Paridad twin <-> carro, preset {preset}", "",
           "| nombre | fuente | .ino/config | preset | ¿difiere? | aceptada |",
           "|---|---|---|---|---|---|"]
    unlisted: list[str] = []
    motivos: list[str] = []
    for r in rows:
        if r.difiere is None:
            dif, acc = "n/a", "-"
        elif not r.difiere:
            dif, acc = "no", "-"
        else:
            dif = "SÍ"
            a = ACEPTADAS.get(r.name)
            if a is None:
                acc = "NO"
                unlisted.append(r.name)
            else:
                acc = a[0]
                motivos.append(f"- {r.name} ({a[0]}): {a[1]}")
        nota = f" ({r.nota})" if r.nota else ""
        out.append(f"| {r.name} | {r.fuente} | {r.real} | {r.twin}{nota} | {dif} | {acc} |")
    if motivos:
        out += ["", "Motivos de las diferencias aceptadas:"] + motivos
    n = sum(1 for r in rows if r.difiere)
    out += ["", f"{n} diferencias: {n - len(unlisted)} aceptadas, {len(unlisted)} sin aceptar"
            + (f" ({', '.join(unlisted)})" if unlisted else "")]
    if errors:
        out += ["", "ERRORES (el preset nombra algo que no existe):"] + [f"- {e}" for e in errors]
    code = 2 if errors else (1 if unlisted else 0)
    return "\n".join(out), code


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("preset", nargs="?", default="hw_nuevo")
    ap.add_argument("--ino", type=Path, default=None, help=f"default: el del preset ({_INO.name})")
    a = ap.parse_args(argv)
    text = a.ino.read_text(encoding="utf-8") if a.ino else None
    rows, errors = compare(a.preset, text)
    report, code = render(rows, errors, a.preset)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    print(report)
    return code


if __name__ == "__main__":
    sys.exit(main())
