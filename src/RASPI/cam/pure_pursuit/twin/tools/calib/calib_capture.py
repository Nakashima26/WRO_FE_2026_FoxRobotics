"""Captura por serie con CalibFox.ino y guarda los CSV para calib_fit_*.py.

Desde src/RASPI/cam (PYTHONUTF8=1 PYTHONPATH=.):
  python -m pure_pursuit.twin.tools.calib.calib_capture --port COM5 --out runs/calib/d1 --vbat 12.1 ramp
  ... step | coast | lock | deadband | enc | steer | tof | looptime | raw "PWM 80 1000"
  --port mock  = compila CalibFox contra el mock del host y lo usa (ensayo en seco).
Requiere pyserial (`pip install pyserial`) solo con puerto real.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

try:
    from . import calib_common as cc
except ImportError:  # ejecutado como script
    import calib_common as cc  # type: ignore


@dataclass
class Reply:
    status: str                   # "OK" | "ERR"
    text: str                     # linea final
    comments: list[str] = field(default_factory=list)
    rows: list[str] = field(default_factory=list)   # incluye la cabecera como 1a fila si hay tabla

    @property
    def ok(self) -> bool:
        return self.status == "OK"

    @property
    def csv_text(self) -> str:
        return "\n".join(self.rows) + "\n"


class CalibLink:
    def __init__(self, port, echo: Callable[[str], None] | None = None):
        self.port = port
        self.echo = echo or (lambda s: None)

    def _readline(self, deadline: float) -> str:
        while True:
            raw = self.port.readline()
            if raw:
                return raw.decode("ascii", errors="replace").strip()
            if time.time() > deadline:
                raise TimeoutError("CalibFox no responde (timeout)")
            if getattr(self.port, "proc", None) is not None:   # mock: EOF = proceso muerto
                raise ConnectionError("el mock termino")

    def command(self, cmd: str, timeout_s: float = 10.0) -> Reply:
        """Envia una linea y lee hasta OK/ERR. timeout_s = duracion esperada + holgura."""
        self.port.write((cmd + "\n").encode("ascii"))
        deadline = time.time() + timeout_s
        comments: list[str] = []
        rows: list[str] = []
        while True:
            ln = self._readline(deadline)
            if not ln:
                continue
            self.echo(ln)
            if ln.startswith(("OK", "ERR")):
                return Reply(ln.split()[0], ln, comments, rows)
            if ln.startswith("#"):
                comments.append(ln)
            else:
                rows.append(ln)

    def wait_ready(self, timeout_s: float = 6.0) -> None:
        """Tras abrir el puerto el ESP32 se reinicia (DTR): espera 'OK READY' o un PING valido."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                r = self.command("PING", 1.5)
                if r.ok:
                    return
            except TimeoutError:
                pass
        raise TimeoutError("CalibFox no contesta PING; revisa puerto, baudios 115200 y que cargaste CalibFox.ino")


def open_port(name: str, mock_dir: Path | None = None):
    if name == "mock":
        from . import mock_build
        exe = mock_build.build(mock_dir or Path("runs/calib/_mock"))
        port = mock_build.MockPort(exe)
        # consume el banner inicial del mock
        for _ in range(4):
            port.readline()
        return port
    try:
        import serial  # type: ignore
    except ImportError as e:
        raise SystemExit("falta pyserial: pip install pyserial") from e
    p = serial.Serial(name, 115200, timeout=0.5)
    time.sleep(2.0)
    p.reset_input_buffer()
    return p


# ── Construccion de comandos (puro, testeable) ────────────────────────────────
def cmd_ramp(a: int, b: int, step: int, hold_ms: int, servo: int | None = None) -> str:
    s = f"RAMP {a} {b} {step} {hold_ms}"
    return s + (f" {servo}" if servo is not None else "")


def cmd_pwm(v: int, ms: int, servo: int | None = None) -> str:
    return f"PWM {v} {ms}" + (f" {servo}" if servo is not None else "")


def ramp_duration_ms(a: int, b: int, step: int, hold_ms: int) -> int:
    return (abs(b - a) // abs(step) + 1) * hold_ms


# ── Sesion ────────────────────────────────────────────────────────────────────
class Session:
    def __init__(self, link: CalibLink, outdir: Path, vbat: float | None = None,
                 note: str = "", ask: Callable[[str], str] = input, say: Callable[[str], None] = print):
        self.link, self.out, self.ask, self.say = link, outdir, ask, say
        self.settle_s = 0.6      # pausa entre corridas (los tests la ponen en 0)
        self.out.mkdir(parents=True, exist_ok=True)
        self.meta_path = self.out / "session.json"
        self.meta = {"created": dt.datetime.now().isoformat(timespec="seconds"),
                     "vbat_V": vbat, "note": note, "files": []}
        self._flush()

    def _flush(self) -> None:
        self.meta_path.write_text(json.dumps(self.meta, indent=2, ensure_ascii=False), encoding="utf-8")

    def save(self, name: str, reply: Reply, cmd: str) -> Path:
        if not reply.ok:
            raise RuntimeError(f"{cmd!r} -> {reply.text}")
        p = self.out / f"{name}.csv"
        p.write_text(reply.csv_text, encoding="utf-8")
        self.meta["files"].append({"file": p.name, "cmd": cmd, "comments": reply.comments})
        self._flush()
        self.say(f"  guardado {p.name} ({len(reply.rows) - 1} filas)")
        return p

    def pause(self, msg: str, prompt: bool) -> None:
        if prompt:
            self.ask(f"{msg} y pulsa ENTER ... ")

    # (a) velocidad vs PWM -----------------------------------------------------
    def ramp(self, reps: int, a: int, b: int, step: int, hold: int, servo: int | None = None,
             prefix: str = "ramp", prompt: bool = True) -> list[Path]:
        out = []
        for r in range(1, reps + 1):
            for sign, tag in ((1, "fwd"), (-1, "rev")):      # adelante y atras: vuelve al punto
                cmd = cmd_ramp(sign * a, sign * b, sign * step, hold, servo)
                self.say(f"[{prefix} {tag} rep {r}/{reps}] {cmd}")
                rep = self.link.command(cmd, ramp_duration_ms(a, b, step, hold) / 1000 + 12)
                out.append(self.save(f"{prefix}_{tag}_r{r:02d}", rep, cmd))
                time.sleep(self.settle_s)
            self.pause("Revisa que el carro volvio cerca del inicio (o reposicionalo)", prompt)
        return out

    def step(self, pwms: list[int], reps: int, ms: int, prompt: bool = True) -> list[Path]:
        out = []
        for p in pwms:
            for r in range(1, reps + 1):
                for sign, tag in ((1, ""), (-1, "n")):       # + y luego - para volver al sitio
                    cmd = cmd_pwm(sign * p, ms)
                    self.say(f"[step {cmd} rep {r}/{reps}]")
                    rep = self.link.command(cmd, ms / 1000 + 8)
                    out.append(self.save(f"step{tag}_p{p:03d}_r{r:02d}", rep, cmd))
                    time.sleep(self.settle_s)
        return out

    def coast(self, pwms: list[int], reps: int, run_ms: int, log_ms: int, brake: bool,
              prompt: bool = True) -> list[Path]:
        out = []
        for p in pwms:
            for r in range(1, reps + 1):
                cmd = f"COAST {p} {run_ms} {log_ms}" + (" brake" if brake else "")
                self.say(f"[coast {cmd} rep {r}/{reps}]")
                self.pause("Pon el carro en el inicio de la pista", prompt)
                rep = self.link.command(cmd, (run_ms + log_ms) / 1000 + 8)
                kind = "brake" if brake else "coast"
                out.append(self.save(f"{kind}_p{p:03d}_r{r:02d}", rep, cmd))
        return out

    def lock(self, pwm: int, servos: list[int], reps: int, ms: int, prompt: bool = True) -> list[Path]:
        """Corridas con el volante fijo: velocidad bajo scrub (+ el radio con `steer`)."""
        out = []
        for s in servos:
            for r in range(1, reps + 1):
                cmd = cmd_pwm(pwm, ms, s)
                self.say(f"[lock {cmd} rep {r}/{reps}]")
                self.pause("Pon el carro con espacio para circular", prompt)
                rep = self.link.command(cmd, ms / 1000 + 8)
                out.append(self.save(f"lock_p{pwm:03d}_s{s:03d}_r{r:02d}", rep, cmd))
        self.link.command("SERVO 90", 3)
        return out

    # (b) encoder --------------------------------------------------------------
    def enc(self, dist_mm: float, reps: int, back: bool = True) -> Path:
        p = self.out / "enc_runs.csv"
        new = not p.exists()
        with p.open("a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["counts_fwd", "dist_mm", "counts_net", "enc_err"])
            for r in range(1, reps + 1):
                self.link.command("SERVOOFF", 3)
                self.link.command("ENCZERO", 3)
                self.ask(f"[enc {r}/{reps}] Empuja el carro a mano EXACTAMENTE {dist_mm:g} mm hacia adelante "
                         "(recto, sin deslizar) y pulsa ENTER ... ")
                fwd = self._enc_read()
                net = ""
                if back:
                    self.ask("Regresalo a la marca de salida y pulsa ENTER ... ")
                    net = self._enc_read()[0]
                w.writerow([fwd[0], dist_mm, net, fwd[1]])
                self.say(f"  cuentas={fwd[0]}  => {fwd[0] / dist_mm:.3f} cuentas/mm; neto al volver={net}")
        self.meta["files"].append({"file": p.name, "cmd": "ENC (manual)"})
        self._flush()
        return p

    def _enc_read(self) -> tuple[int, int]:
        r = self.link.command("ENC", 3)
        kv = dict(t.split("=") for t in r.text.split() if "=" in t)
        return int(kv["counts"]), int(kv["err"])

    # (c)(e) direccion ------------------------------------------------------------
    def steer(self, servos: list[int], reps: int, pwm: int, ms: int) -> Path:
        """Circula a servo fijo y pide el radio medido; escribe steer.csv para calib_fit_steer."""
        p = self.out / "steer.csv"
        new = not p.exists()
        with p.open("a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["servo_deg", "radius_mm", "rep", "note"])
            for s in servos:
                for r in range(1, reps + 1):
                    self.ask(f"[steer servo={s} rep {r}/{reps}] Pon el carro libre; marca el centro del eje trasero. ENTER ... ")
                    cmd = cmd_pwm(pwm, ms, s)
                    rep = self.link.command(cmd, ms / 1000 + 8)
                    self.save(f"lock_p{pwm:03d}_s{s:03d}_r{r:02d}", rep, cmd)
                    val = self.ask("  Radio del centro del eje trasero en mm (ENTER = saltar): ").strip()
                    if val:
                        w.writerow([s, float(val), r, ""])
                        f.flush()
        self.link.command("SERVO 90", 3)
        return p

    # (d) loop ---------------------------------------------------------------------
    def looptime(self, n: int) -> Path:
        rep = self.link.command(f"LOOPTIME {n}", 30 + n * 0.05)
        p = self.save("looptime", rep, f"LOOPTIME {n}")
        for ln in rep.rows:
            self.say("  " + ln)
        return p

    def tof(self, sensor: str, dists_mm: list[int], variants: list[tuple[str, str]], dur_ms: int) -> list[Path]:
        """variants = [(modo, presupuesto)] p. ej. [("SHORT","20"), ("LONG","DEF")]."""
        out = []
        for mode, bud in variants:
            for d in dists_mm:
                self.ask(f"[tof {sensor} {mode}/{bud}] Pon un blanco plano a {d} mm del sensor. ENTER ... ")
                cmd = f"TOF {sensor} {mode} {bud} {dur_ms}"
                rep = self.link.command(cmd, dur_ms / 1000 + 8)
                out.append(self.save(f"tof_{sensor}_{mode.lower()}_{bud.lower()}_d{d:04d}", rep, cmd))
                for c in rep.comments:
                    if c.startswith("# TOFSUM"):
                        self.say("  " + c)
        return out


def _ints(s: str) -> list[int]:
    return [int(x) for x in s.split(",") if x.strip()]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="COM5, /dev/ttyUSB0 o 'mock'")
    ap.add_argument("--out", default=None, help="carpeta de salida (def. runs/calib/<fecha>)")
    ap.add_argument("--vbat", type=float, default=None, help="tension de la bateria al empezar (V); queda en session.json")
    ap.add_argument("--note", default="")
    ap.add_argument("--no-prompt", action="store_true")
    sub = ap.add_subparsers(dest="what", required=True)

    s = sub.add_parser("ramp", help="v(PWM): escalera adelante y reversa")
    s.add_argument("--reps", type=int, default=3)
    s.add_argument("--from", dest="a", type=int, default=20)
    s.add_argument("--to", dest="b", type=int, default=130)
    s.add_argument("--step", type=int, default=10)
    s.add_argument("--hold", type=int, default=800)
    s = sub.add_parser("deadband", help="zona muerta: escalera fina de PWM bajo")
    s.add_argument("--reps", type=int, default=3)
    s = sub.add_parser("step", help="respuesta al escalon (tau_drive)")
    s.add_argument("--pwms", default="60,90,120")
    s.add_argument("--reps", type=int, default=3)
    s.add_argument("--ms", type=int, default=2500)
    s = sub.add_parser("coast", help="rodar tras cortar (tau_coast)")
    s.add_argument("--pwms", default="100")
    s.add_argument("--reps", type=int, default=3)
    s.add_argument("--run-ms", type=int, default=1500)
    s.add_argument("--log-ms", type=int, default=2000)
    s.add_argument("--brake", action="store_true")
    s = sub.add_parser("lock", help="velocidad con volante fijo (scrub)")
    s.add_argument("--pwm", type=int, default=100)
    s.add_argument("--servos", default="90,50,30")
    s.add_argument("--reps", type=int, default=3)
    s.add_argument("--ms", type=int, default=2500)
    s = sub.add_parser("enc", help="cuentas/mm empujando a mano")
    s.add_argument("--dist", type=float, default=1000.0)
    s.add_argument("--reps", type=int, default=5)
    s = sub.add_parser("steer", help="radio de giro a varios comandos de servo")
    s.add_argument("--servos", default="60,70,80,100,110,120,130,150")
    s.add_argument("--reps", type=int, default=2)
    s.add_argument("--pwm", type=int, default=70)
    s.add_argument("--ms", type=int, default=4000)
    s = sub.add_parser("looptime", help="duracion de la secuencia de sensado del loop()")
    s.add_argument("--n", type=int, default=500)
    s = sub.add_parser("tof", help="tasa/status/alcance del VL53L1X")
    s.add_argument("--sensor", default="L")
    s.add_argument("--dists", default="1000,1300,1500")
    s.add_argument("--variants", default="SHORT:20,SHORT:DEF,LONG:33,LONG:DEF")
    s.add_argument("--ms", type=int, default=4000)
    s = sub.add_parser("raw", help="manda una linea cruda y muestra la respuesta")
    s.add_argument("line")

    a = ap.parse_args(argv)
    out = Path(a.out) if a.out else Path("runs/calib") / dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    port = open_port(a.port, out / "_mock")
    link = CalibLink(port)
    link.wait_ready()
    sess = Session(link, out, a.vbat, a.note)
    prompt = not a.no_prompt
    try:
        if a.what == "ramp":
            sess.ramp(a.reps, a.a, a.b, a.step, a.hold, prompt=prompt)
        elif a.what == "deadband":
            sess.ramp(a.reps, 5, 50, 1, 250, prefix="dead", prompt=prompt)
        elif a.what == "step":
            sess.step(_ints(a.pwms), a.reps, a.ms, prompt)
        elif a.what == "coast":
            sess.coast(_ints(a.pwms), a.reps, a.run_ms, a.log_ms, a.brake, prompt)
        elif a.what == "lock":
            sess.lock(a.pwm, _ints(a.servos), a.reps, a.ms, prompt)
        elif a.what == "enc":
            sess.enc(a.dist, a.reps)
        elif a.what == "steer":
            sess.steer(_ints(a.servos), a.reps, a.pwm, a.ms)
        elif a.what == "looptime":
            sess.looptime(a.n)
        elif a.what == "tof":
            v = [tuple(x.split(":")) for x in a.variants.split(",")]
            sess.tof(a.sensor.upper(), _ints(a.dists), v, a.ms)
        elif a.what == "raw":
            r = link.command(a.line, 60)
            print("\n".join(r.comments + r.rows + [r.text]))
    finally:
        try:
            link.command("STOP", 3)
        except Exception:  # noqa: BLE001
            pass
        if hasattr(port, "close"):
            port.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
