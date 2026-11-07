"""Utilidades compartidas: lectura de los CSV de CalibFox, segmentos, parches.

CSV de CalibFox: t_ms,pwm,enc_counts,v_mm_s,servo_deg,enc_err
  t_ms       ms desde el inicio de la corrida
  pwm        PWM firmado aplicado (negativo = reversa)
  enc_counts cuentas x4 con signo desde el inicio de la corrida
  v_mm_s     velocidad calculada en el ESP32 con el CPMM NOMINAL (no se usa al ajustar)
  servo_deg  ultimo angulo de servo comandado
  enc_err    saltos dobles del encoder acumulados (cuentas perdidas)
"""

from __future__ import annotations

import ast
import difflib
import json
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

CSV_COLS = ("t_ms", "pwm", "enc_counts", "v_mm_s", "servo_deg", "enc_err")


@dataclass
class Run:
    name: str
    t_s: np.ndarray
    pwm: np.ndarray
    counts: np.ndarray
    servo: np.ndarray
    enc_err: np.ndarray

    @property
    def n(self) -> int:
        return int(self.t_s.size)

    def dist_mm(self, counts_per_mm: float) -> np.ndarray:
        return self.counts / counts_per_mm


def parse_csv_text(text: str, name: str = "run") -> Run:
    rows = []
    for ln in text.splitlines():
        ln = ln.strip().lstrip("﻿")
        if not ln or ln.startswith("#") or ln.startswith("t_ms"):
            continue
        if ln.startswith(("OK", "ERR")):
            continue
        parts = ln.split(",")
        if len(parts) < 6:
            continue
        rows.append([float(x) for x in parts[:6]])
    if len(rows) < 3:
        raise ValueError(f"{name}: menos de 3 filas CSV")
    a = np.asarray(rows, dtype=float)
    return Run(
        name=name,
        t_s=a[:, 0] / 1000.0,
        pwm=a[:, 1],
        counts=a[:, 2],
        servo=a[:, 4],
        enc_err=a[:, 5],
    )


def load_run(path) -> Run:
    p = Path(path)
    return parse_csv_text(p.read_text(encoding="utf-8"), p.stem)


def run_kind(name: str) -> str:
    """'ramp_fwd_r01' -> 'ramp'. Los nombres los pone calib_capture.py."""
    return name.split("_", 1)[0]


def tag_int(name: str, prefix: str) -> int | None:
    """'step_p100_r01', 'p' -> 100;  'lock_p060_s030_r01', 's' -> 30."""
    m = re.search(rf"_{prefix}(\d+)", name)
    return int(m.group(1)) if m else None


def segments(run: Run, min_s: float = 0.25) -> list[tuple[int, int, int]]:
    """Tramos de PWM constante: (i0, i1_inclusivo, pwm). Descarta los < min_s."""
    out: list[tuple[int, int, int]] = []
    i0 = 0
    for i in range(1, run.n + 1):
        if i == run.n or run.pwm[i] != run.pwm[i0]:
            if run.t_s[i - 1] - run.t_s[i0] >= min_s:
                out.append((i0, i - 1, int(round(run.pwm[i0]))))
            i0 = i
    return out


def window_speed(run: Run, i0: int, i1: int, counts_per_mm: float, tail: float) -> tuple[float, bool]:
    """Velocidad (mm/s, con signo) en el ultimo `tail` del tramo [i0,i1] y si ya era estable.

    Se calcula con la diferencia de cuentas extremo a extremo (sin derivar muestra a
    muestra: la cuantizacion de 1 cuenta = 1/cpmm mm no se amplifica). 'Estable' =
    las velocidades de las dos mitades de la ventana difieren menos de 8 %.
    """
    t0, t1 = run.t_s[i0], run.t_s[i1]
    ts = t1 - tail * (t1 - t0)
    j0 = int(np.searchsorted(run.t_s[i0:i1 + 1], ts)) + i0
    j0 = min(j0, i1 - 1)
    jm = (j0 + i1) // 2
    dt = run.t_s[i1] - run.t_s[j0]
    v = (run.counts[i1] - run.counts[j0]) / counts_per_mm / dt if dt > 0 else 0.0
    d1 = run.t_s[jm] - run.t_s[j0]
    d2 = run.t_s[i1] - run.t_s[jm]
    if d1 > 0 and d2 > 0:
        va = (run.counts[jm] - run.counts[j0]) / counts_per_mm / d1
        vb = (run.counts[i1] - run.counts[jm]) / counts_per_mm / d2
        ref = max(abs(va), abs(vb), 1e-9)
        stable = abs(va - vb) / ref < 0.08 or max(abs(va), abs(vb)) < 3.0
    else:
        stable = False
    return float(v), bool(stable)


# ── Raiz del repo y rutas ─────────────────────────────────────────────────────
def repo_root(start: Path | None = None) -> Path:
    p = (start or Path(__file__)).resolve()
    for q in [p, *p.parents]:
        if (q / ".git").exists():
            return q
    raise RuntimeError("no encuentro la raiz del repo (.git)")


PARAMS_REL = "src/RASPI/cam/pure_pursuit/twin/params.py"
INO_REL = "src/ESP32/PurePursuit/PurePursuit.ino"


# ── Edicion de archivos fuente conservando bytes/EOL ──────────────────────────
@dataclass
class SrcFile:
    rel: str
    text: str
    eol: str
    bom: bool

    @classmethod
    def read(cls, root: Path, rel: str) -> "SrcFile":
        raw = (root / rel).read_bytes()
        bom = raw.startswith(b"\xef\xbb\xbf")
        txt = raw[3:].decode("utf-8") if bom else raw.decode("utf-8")
        eol = "\r\n" if "\r\n" in txt else "\n"
        return cls(rel, txt, eol, bom)


def fmt_num(x: float) -> str:
    s = f"{x:.4g}"
    if re.fullmatch(r"-?\d+", s):
        s += ".0"
    return s


def replace_field_default(text: str, field: str, value_src: str) -> str:
    """`    <field>: float = <lo que sea>` -> `... = value_src` (exactamente 1 coincidencia)."""
    pat = re.compile(rf"^(\s+{re.escape(field)}:\s*float\s*=\s*)([^\r\n#]*?)(\s*)(#[^\r\n]*)?$", re.M)
    ms = list(pat.finditer(text))
    if len(ms) != 1:
        raise ValueError(f"campo {field!r}: {len(ms)} coincidencias (se esperaba 1)")
    m = ms[0]
    tail = m.group(3) + (m.group(4) or "")
    return text[:m.start()] + m.group(1) + value_src + tail + text[m.end():]


def replace_prov(text: str, key: str, kind: str, note: str, eol: str) -> str:
    """Reemplaza la llamada `_prov(<key>, ...)` (de cualquier largo) por una de 1 linea."""
    tree = ast.parse(text)
    hit = None
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "_prov"
                and node.args and isinstance(node.args[0], ast.Constant) and node.args[0].value == key):
            hit = node
            break
    if hit is None:
        raise ValueError(f"no hay _prov({key!r}, ...)")
    lines = text.splitlines(keepends=True)
    first = lines[hit.lineno - 1]
    indent = first[:len(first) - len(first.lstrip())]
    new = f"{indent}_prov({json.dumps(key)}, {json.dumps(kind)}, {json.dumps(note, ensure_ascii=False)}){eol}"
    return "".join(lines[:hit.lineno - 1]) + new + "".join(lines[hit.end_lineno:])


def replace_ino_const(text: str, name: str, value_src: str) -> str:
    """`const <tipo> NAME   = <num>f;` -> conserva espaciado y comentario."""
    pat = re.compile(rf"^(\s*const\s+\w+\s+{re.escape(name)}\s*=\s*)([-+0-9.eE]+f?)(\s*;)", re.M)
    ms = list(pat.finditer(text))
    if len(ms) != 1:
        raise ValueError(f"constante {name!r}: {len(ms)} coincidencias (se esperaba 1)")
    m = ms[0]
    return text[:m.start()] + m.group(1) + value_src + m.group(3) + text[m.end():]


def unified_diff(rel: str, old: str, new: str) -> str:
    return "".join(difflib.unified_diff(
        old.splitlines(keepends=True), new.splitlines(keepends=True),
        fromfile=f"a/{rel}", tofile=f"b/{rel}", n=3))


def write_src(root: Path, f: SrcFile, new_text: str) -> None:
    data = new_text.encode("utf-8")
    if f.bom:
        data = b"\xef\xbb\xbf" + data
    (root / f.rel).write_bytes(data)
