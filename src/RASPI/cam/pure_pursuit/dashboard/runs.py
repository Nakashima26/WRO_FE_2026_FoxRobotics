"""Corridas grabadas para el replay del dashboard.

Cada corrida es videos_orillas/orillasN.avi y, desde 2026-11-24, orillasN_tele.jsonl (run_log.py):
el mismo estado que ve "En vivo", ~12 por segundo. Para las corridas de antes se reconstruye
un estado aproximado desde el journal del servicio (ACK del ESP + [LOCUPD] + [ROUTE]) y se
guarda como orillasN_tele.jsonl con "src": "journal" para no volver a leer el journal.
"""
from __future__ import annotations

import json
import math
import re
import subprocess
import threading
import time
from pathlib import Path

from ..telemetry import parse_ack

SERVICE = "wro-runtime"
EV_TAGS = ("[ROUTE]", "[PARK", "[INICIO]", "[REC]", "[FIXVOTE]", "[DMAP]", "[CAM]", "[carrera]",
           "[LOCUPD]", "[LOC]", "[TURNDIR]", "[LINEUP]", "[MEMREJ]", "[PROVBLOCK]", "[TELE]", "[GPIO]",
           "[CALIB]", "[INFO]")
EV_ANY = ("AVISO", "Traceback", "Error")


def tele_path(video: Path) -> Path:
    return video.with_name(video.stem + "_tele.jsonl")


def list_runs(videos: Path, n: int = 40) -> list[dict]:
    files = sorted(videos.glob("orillas*.avi"), key=lambda f: f.stat().st_mtime, reverse=True)[:n]
    out = []
    for f in files:
        st = f.stat()
        tp = tele_path(f)
        src = None
        if tp.exists():
            try:
                with open(tp, encoding="utf-8") as fh:
                    src = json.loads(fh.readline()).get("src", "runtime")
            except Exception:
                src = "?"
        out.append({"name": f.name, "mb": round(st.st_size / 1e6, 1),
                    "t": time.strftime("%m-%d %H:%M", time.localtime(st.st_mtime)), "tele": src})
    return out


# ── Lectura ──────────────────────────────────────────────────────────────────
def load(videos: Path, name: str) -> dict:
    video = videos / Path(name).name
    if not video.exists():
        raise FileNotFoundError(name)
    tp = tele_path(video)
    if not tp.exists():
        from_journal(video, tp)
    hdr, states, events = {}, [], []
    route = None
    with open(tp, encoding="utf-8") as fh:
        for line in fh:
            try:
                r = json.loads(line)
            except ValueError:
                continue   # última línea a medias si mataron el servicio
            if r.get("hdr"):
                hdr = r
                continue
            s = r.get("s") or {}
            if "route" in s:
                route = s["route"]
            elif route is not None:
                s["route"] = route
            states.append(s)
            for t, l in r.get("ev", []):
                events.append([t, l, "pi"])
            for t, l in r.get("esp", []):
                events.append([t, l, "esp"])
    events.sort(key=lambda e: e[0])
    return {"name": video.name, "hdr": hdr, "states": states, "events": events,
            "video": video_info(video), "summary": summary(hdr, states, events)}


def summary(hdr: dict, states: list, events: list) -> dict:
    acks = [(s.get("t"), s.get("ack") or {}) for s in states if s.get("ack")]
    armed = [s.get("t") for s in states if s.get("armed")]
    out = {"src": hdr.get("src", "runtime"), "n": len(states)}
    if states:
        out["t0"], out["t1"] = states[0].get("t"), states[-1].get("t")
    if armed:
        out["armed_s"] = round(armed[-1] - armed[0], 1)
    if acks:
        a = [x for _t, x in acks]
        out["fw"] = next((x["fw"] for x in a if x.get("fw")), "obs")
        out["dir"] = next((x["dir"] for x in reversed(a) if x.get("dir") not in (None, "?", "-")), None)
        tcs = [x.get("tc") for x in a if isinstance(x.get("tc"), (int, float))]
        out["tc_max"] = max(tcs) if tcs else None
        out["tpr"] = next((x.get("tpr") for x in reversed(a) if x.get("tpr")), None)
        out["est_end"] = a[-1].get("est")
        ests = {}
        prev = None
        for x in a:
            e = x.get("est")
            if e != prev and e is not None:
                ests[e] = ests.get(e, 0) + 1
            prev = e
        out["est_count"] = ests
        for k in ("tL", "tR", "tF", "tB"):
            v = [x.get(k) for x in a if isinstance(x.get(k), (int, float))]
            if v:
                ok = [z for z in v if z > 0]
                out[k] = {"min": min(ok) if ok else None, "lost": round(100.0 * (len(v) - len(ok)) / len(v), 1)}
        for k in ("ue", "uf"):
            v = [x.get(k) for x in a if isinstance(x.get(k), (int, float))]
            if v:
                out[k] = max(v) - min(v)
        od = [x.get("od") for x in a if isinstance(x.get("od"), (int, float))]
        if od:
            out["dist_m"] = round((max(od) - min(od)) / 1000.0, 2)
    park = [e[1] for e in events if "[PARK" in e[1] or "PARK" in e[1][:12]]
    out["park"] = park[-3:]
    out["n_ev"] = len(events)
    out["errors"] = [e[1] for e in events if any(k in e[1] for k in EV_ANY)][:10]
    return out


# ── Video ────────────────────────────────────────────────────────────────────
_CAPS: dict = {}
_CAP_LOCK = threading.Lock()


def video_info(video: Path) -> dict:
    try:
        import cv2
        c = cv2.VideoCapture(str(video))
        n = int(c.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        fps = float(c.get(cv2.CAP_PROP_FPS) or 0)
        c.release()
        return {"frames": n, "fps": fps}
    except Exception:
        return {"frames": 0, "fps": 0}


def frame_jpeg(videos: Path, name: str, i: int, width: int = 640) -> bytes | None:
    import cv2
    video = videos / Path(name).name
    with _CAP_LOCK:
        ent = _CAPS.get(video.name)
        if ent is None:
            for k in list(_CAPS):
                _CAPS.pop(k)[0].release()
            ent = _CAPS[video.name] = [cv2.VideoCapture(str(video)), -1]
        cap = ent[0]
        if i != ent[1] + 1:
            cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, i))
        ok, img = cap.read()
        ent[1] = i if ok else -1
    if not ok:
        return None
    if img.shape[1] > width:
        img = cv2.resize(img, (width, int(img.shape[0] * width / img.shape[1])))
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return buf.tobytes() if ok else None


# ── Corridas viejas: desde el journal ────────────────────────────────────────
_RX_ACK = re.compile(r"RX: (ACK:V2[^ |]*)")
_RX_OTHER = re.compile(r"\[SERIAL\] RX: (?!ACK|PRE )(.*)")
_LOCUPD = re.compile(r"\[LOCUPD\].*pose=\((-?\d+),(-?\d+),(-?\d+)\)")
_ROUTE_PARK = re.compile(r"park=([NESW])@(\d+)")
_ROUTE_SIGNS = re.compile(r"señales=\[([^\]]*)\]")


def _journal(since: float, until: float | None = None, grep: str | None = None) -> list[tuple[float, str]]:
    cmd = ["journalctl", "-u", SERVICE, "-o", "short-unix", "--no-pager", "--since", f"@{since:.3f}"]
    if until is not None:
        cmd += ["--until", f"@{until:.3f}"]
    if grep:
        cmd += ["--grep", grep]
    txt = subprocess.run(cmd, capture_output=True, text=True, timeout=60).stdout
    out = []
    for line in txt.splitlines():
        ts, _sp, rest = line.partition(" ")
        try:
            t = float(ts)
        except ValueError:
            continue
        _host, _sp, rest = rest.partition(" ")
        _proc, _sp, msg = rest.partition(": ")
        out.append((t, msg))
    return out


def from_journal(video: Path, out: Path) -> None:
    """Reconstruye <video>_tele.jsonl desde el journal (pose aproximada: LOCUPD + odometría del ACK)."""
    mt = video.stat().st_mtime
    hits = _journal(mt - 6 * 3600, mt + 5, grep=f"Grabando en .*{re.escape(video.name)}")
    if not hits:
        raise RuntimeError("el journal ya no tiene esta corrida")
    t0 = hits[-1][0] - 3.0
    lines = _journal(t0, mt + 8)
    end = mt + 8
    for t, msg in lines:
        if t > t0 + 4 and ("Grabación cortada" in msg or ("Grabando en" in msg and video.name not in msg)):
            end = t
            break
    lines = [(t, m) for t, m in lines if t <= end]
    states, pending_ev, pending_esp = [], [], []
    xf = None            # (tx, ty, th): pose = transform(px, py, yaw) como localizer.pose()
    last_ack, last_raw = None, None
    parking, park_w, confirmed, direction = None, None, [], None
    for t, msg in lines:
        m = _RX_ACK.search(msg)
        if m:
            if m.group(1) == last_raw:   # el mismo ACK sale repetido en las líneas TX | RX
                continue
            last_raw = m.group(1)
            a = parse_ack(m.group(1))
            last_ack = a
            s = {"t": t, "armed": True, "ack": a, "ack_age": 0}
            if a.get("dir") in ("L", "R"):
                direction = "CCW" if a["dir"] == "L" else "CW"
            px, py, yaw = a.get("px"), a.get("py"), a.get("yaw")
            if xf is not None and None not in (px, py, yaw):
                tx, ty, th = xf
                r = math.radians(th)
                s["loc"] = {"pose": [round(tx + px * math.sin(r) - py * math.cos(r), 1),
                                     round(ty + px * math.cos(r) + py * math.sin(r), 1),
                                     round((th - yaw) % 360.0, 2)]}
            s["map"] = {"direction": direction, "parking": parking, "park_w": park_w or 0.0,
                        "confirmed": confirmed}
            rec = {"t": round(t, 3), "s": s}
            if pending_ev:
                rec["ev"], pending_ev = pending_ev, []
            if pending_esp:
                rec["esp"], pending_esp = pending_esp, []
            states.append(rec)
            continue
        m = _RX_OTHER.search(msg)
        if m:
            pending_esp.append([round(t, 3), m.group(1)[:300]])
            continue
        if msg.startswith(EV_TAGS) or any(k in msg for k in EV_ANY):
            pending_ev.append([round(t, 3), msg[:500]])
        m = _LOCUPD.search(msg)
        if m and last_ack is not None and None not in (last_ack.get("px"), last_ack.get("py"), last_ack.get("yaw")):
            x, y, h = (float(v) for v in m.groups())
            px, py, yaw = last_ack["px"], last_ack["py"], last_ack["yaw"]
            th = h + yaw
            r = math.radians(th)
            xf = (x - px * math.sin(r) + py * math.cos(r), y - px * math.cos(r) - py * math.sin(r), th)
        if msg.startswith("[ROUTE]"):
            m = _ROUTE_PARK.search(msg)
            if m:
                parking, park_w = m.group(1), float(m.group(2))
            m = _ROUTE_SIGNS.search(msg)
            if m:
                confirmed = []
                for tok in m.group(1).split():
                    sec, _sl, rest = tok.partition("/")
                    if len(rest) >= 3:
                        confirmed.append([sec, rest[:2], "Red" if rest[2] == "R" else "Green"])
    if not states:
        raise RuntimeError("no hay ACK del ESP en el journal para esta corrida")
    if pending_ev or pending_esp:
        states[-1].setdefault("ev", []).extend(pending_ev)
        states[-1].setdefault("esp", []).extend(pending_esp)
    tmp = out.with_suffix(".tmp")
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write(json.dumps({"hdr": 1, "t": round(t0 + 3.0, 3), "video": video.name, "src": "journal"}) + "\n")
        for r in states:
            fh.write(json.dumps(r, separators=(",", ":")) + "\n")
    tmp.replace(out)


# ── Consultas compactas para análisis (texto plano, t relativo al [INICIO]) ──
ACK_CAMPOS = "est,tc,pkf,pns,pal,ang,dL,dR,dF,pw,tL,tR,tF,tB"


def _ventana(videos: Path, name: str) -> tuple[float, float, float]:
    """(t_inicio, desde, hasta) en epoch: el cero es el [INICIO] de la corrida (o el inicio del video)."""
    d = load(videos, name)
    s = d["summary"]
    t0 = next((e[0] for e in d["events"] if "[INICIO]" in e[1]), s.get("t0") or d["hdr"].get("t", 0.0))
    return t0, (s.get("t0") or t0) - 1.0, (s.get("t1") or t0 + 600) + 1.0


def acks_tsv(videos: Path, name: str, campos: str = ACK_CAMPOS, desde: float = -1e9, hasta: float = 1e9,
             paso: float = 0.0, cambio: str = "") -> str:
    """ACK del ESP a tasa completa desde el journal. paso: s mínimos entre filas; cambio: campos cuya
    variación fuerza fila (p. ej. "pkf,pns")."""
    t0, a, b = _ventana(videos, name)
    keys = [k for k in campos.split(",") if k]
    ver = [k for k in cambio.split(",") if k]
    out, last_raw, last_t, last_v = ["t\t" + "\t".join(keys)], None, -1e9, None
    for t, msg in _journal(a, b, grep="ACK:V2"):
        m = _RX_ACK.search(msg)
        if not m or m.group(1) == last_raw:
            continue
        last_raw = m.group(1)
        rt = t - t0
        if rt < desde or rt > hasta:
            continue
        d = dict(p.split("=", 1) for p in m.group(1)[7:].split(",") if "=" in p)
        v = tuple(d.get(k, "") for k in ver)
        if (ver and v != last_v) or (paso > 0 and rt - last_t >= paso) or (not ver and not paso):
            out.append(f"{rt:.2f}\t" + "\t".join(d.get(k, "").split("*")[0] for k in keys))
            last_t, last_v = rt, v
    return "\n".join(out) + "\n"


def log_grep(videos: Path, name: str, patron: str, desde: float = -1e9, hasta: float = 1e9,
             maxc: int = 240, n: int = 400) -> str:
    """Líneas del journal de la corrida que cumplen la regex, recortadas, con t relativo al [INICIO]."""
    t0, a, b = _ventana(videos, name)
    rx = re.compile(patron)
    out = []
    for t, msg in _journal(a, b):
        rt = t - t0
        if desde <= rt <= hasta:
            m = rx.search(msg)
            if m:
                out.append(f"{rt:.2f}\t{msg[m.start():m.start() + maxc]}")
                if len(out) >= n:
                    break
    return "\n".join(out) + "\n"
