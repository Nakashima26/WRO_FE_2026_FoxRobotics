"""Scratch: corre Sim con log_dir y guarda trace.csv + fw_debug.log + field."""
import csv, json, sys
from pathlib import Path
from pure_pursuit.twin import sim as S
from pure_pursuit.twin.firmware import fw as F

seed = int(sys.argv[1]); out = Path(sys.argv[2]); preset = sys.argv[3] if len(sys.argv) > 3 else "giro_rapido"
extra = (json.loads(sys.argv[4]) if len(sys.argv) > 4 else None) or {}
out.mkdir(parents=True, exist_ok=True)
dbg = open(out / "fw_debug.log", "w", encoding="utf-8")
_orig = F.FirmwareSIL.pop_debug
def pop(self):
    s = _orig(self)
    if s:
        dbg.write(f"@{self.now_us/1e6:.3f} " + s.replace("\n", f"\n@{self.now_us/1e6:.3f} ") if False else s)
    return s
# prefijo de tiempo por línea
_buf = {"t": ""}
def pop2(self):
    s = _orig(self)
    if s:
        t = self.now_us / 1e6
        txt = _buf["t"] + s
        lines = txt.split("\n")
        _buf["t"] = lines.pop()
        for ln in lines:
            dbg.write(f"{t:8.3f} {ln}\n")
    return s
F.FirmwareSIL.pop_debug = pop2
from pure_pursuit import digital_map as DM
_bt = DM.DigitalMap.blocks_turn
def bt(self):
    r = _bt(self)
    ga = self._green_ahead()
    print(f"[DMDBG] sec={self.section} tc={self._tc} along={self.along_mm:.0f} lat={self.lat_mm:.0f} "
          f"pose=({self.pose_xy[0]:.0f},{self.pose_xy[1]:.0f}) h={self.heading:.0f} aligned={self._aligned} "
          f"green_ahead={None if ga is None else (round(ga[0]), ga[1])} "
          f"outside_release={self.along_mm > 1900.0 and self.lat_mm < -80.0} blocks_turn={r} "
          f"needs_line={self.needs_line()} conf={sorted(self.confirmed.items())}", flush=True)
    return r
DM.DigitalMap.blocks_turn = bt
import math
from pure_pursuit import config as _C
_up = DM.DigitalMap.update
def up(self, ack, feet, orange=None):
    _up(self, ack, feet, orange)
    self._dbg_n = getattr(self, "_dbg_n", 0) + 1
    px, py = self.pose_xy; h = math.radians(self._view_heading)
    out = []
    for bx, by, color in feet or []:
        if color not in ("Red", "Green"): continue
        lat = (bx - _C.ROBOT_BEV_X) * _C.MM_PER_PX; fwd = (_C.ROBOT_BEV_Y - by) * _C.MM_PER_PX
        if fwd < 80.0 or fwd > 2200.0 or abs(lat) > 1400.0: continue
        fwd += DM._BEV_AHEAD_MM
        ray = math.hypot(fwd - DM._CAM_AHEAD_MM, lat)
        fwd += DM._FOOT_TO_CENTER_MM * (fwd - DM._CAM_AHEAD_MM) / ray; lat += DM._FOOT_TO_CENTER_MM * lat / ray
        out.append(f"{color[0]}({px + fwd*math.sin(h) + lat*math.cos(h):.0f},{py + fwd*math.cos(h) - lat*math.sin(h):.0f})@{fwd:.0f},{lat:.0f}")
    if out:
        print(f"[DMFEET] n={self._dbg_n} " + " ".join(out), flush=True)
DM.DigitalMap.update = up
_pp = DM.DigitalMap._pull_pose
def pp(self, wx, wy, sec, sid):
    f0 = self._pose_fix
    _pp(self, wx, wy, sec, sid)
    print(f"[DMPULL] {sec}/{sid} w=({wx:.0f},{wy:.0f}) h={self._view_heading:.0f} fix {f0[0]:.0f},{f0[1]:.0f} -> {self._pose_fix[0]:.0f},{self._pose_fix[1]:.0f}", flush=True)
DM.DigitalMap._pull_pose = pp
_LAST = {}
_init = DM.DigitalMap.__init__
def init(self):
    _init(self); _LAST["dm"] = self
DM.DigitalMap.__init__ = init
import os
if os.environ.get("NO_OUTSIDE_RELEASE"):
    def _bt_nr(self):
        ahead = self._green_ahead()
        if ahead is None:
            return False
        return ahead[0] > (480.0 if ahead[1] else 260.0)
    _bt = _bt_nr
sim = S.Sim(seed, preset=preset, log_dir=out, pi_overrides=extra.get("pi"), fw_overrides=extra.get("fw"), fw_defines=extra.get("def"), pi_period_s=extra.get("period"))
r = sim.run()
dbg.close()
if "dm" in _LAST:
    dm = _LAST["dm"]
    print("VOTES", {f"{k[0]}/{k[1]}": (v, round(dm._best.get(k, -1))) for k, v in dm._votes.items()})
    print("CONF", dm.confirmed)
keys = sorted({k for row in r.trace for k in row})
with open(out / "trace.csv", "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(r.trace)
fld = r.field
(out / "field.txt").write_text(
    f"dir={fld.direction} parking={fld.parking_section} start=({fld.start_x:.0f},{fld.start_y:.0f},{fld.start_heading})\n"
    + "\n".join(f"{s.color} {s.section}/{s.seat} ({s.x:.0f},{s.y:.0f})" for s in fld.signs), encoding="utf-8")
m = dict(r.metrics); m.pop("corners", None)
print("STOP", r.stop_reason, json.dumps(m, ensure_ascii=False, default=str))
