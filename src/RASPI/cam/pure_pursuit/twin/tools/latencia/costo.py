"""drive.py con el costo de cómputo de la Pi dentro del tiempo simulado (bucle síncrono, ROUTE_PLAN_ASYNC=False).
Cada comando al ESP sale en t_cuadro + (tiempo de pared hasta ese send) escalado; el siguiente cuadro empieza
cuando termina el cálculo (o a los 25 ms si es más rápido, tope de la cámara a 40 fps).
Escala: TWIN_COST_FRAME (cuadro sin DP) y TWIN_COST_DP (dentro de ring_route.plan), Mac -> Pi.
Uso (cwd src/RASPI/cam): costo.py <seed> <out_dir>   (resto igual que banco/wk.sh)"""
import os, sys, time, runpy, types, atexit, json
sys.path.insert(0, ".")
os.environ.setdefault("FOX_CV_THREADS", "1")
FF = float(os.environ.get("TWIN_COST_FRAME", "1")); FIRST = float(os.environ.get("TWIN_COST_FIRST", "0") or 0) or None; FD = float(os.environ.get("TWIN_COST_DP", "1"))
from pure_pursuit import ring_route as RR
from pure_pursuit.twin import pi_link as PL
st = types.SimpleNamespace(t0=None, tk=0.0, tend=0.0, dp=0.0, costs=[], dps=[])
def costo():  # segundos de Pi transcurridos en este cuadro
    e = time.thread_time() - st.t0
    return (e - st.dp) * FF + st.dp * FD
_plan = RR.plan
st.depth = 0
def plan(*a, **k):
    # los reintentos de plan() se llaman a sí mismos: solo se cobra la llamada exterior
    if st.depth: return _plan(*a, **k)
    st.depth = 1
    g0 = len(RR._GEOM_CACHE)
    try:
        t = time.thread_time(); r = _plan(*a, **k); d = time.thread_time() - t
    finally:
        st.depth = 0
    if FIRST and FD and len(RR._GEOM_CACHE) > g0: d = FIRST / FD   # plan que construye la caché: en la Pi 5 real 1.95 s
    st.dp += d; st.dps.append(d * FD)
    sys.stdout.write(f"PLAN t={st.tk:.3f} costo_ms={d*FD*1000:.0f} conf={len(a[1]) if len(a)>1 else '?'}\n")
    return r
RR.plan = plan
_send = PL.SimSerialLink.send_line
def send_line(self, line):
    if st.t0 is None: return _send(self, line)
    keep = self._pi_now_us
    self._pi_now_us = int(max(st.tend, st.tk + costo()) * 1e6)
    try: return _send(self, line)
    finally: self._pi_now_us = keep
PL.SimSerialLink.send_line = send_line
# ROUTE_PLAN_ASYNC: la DP va en otro hilo. Hilo falso: corre ya (tiempo de pared medido), pero
# is_alive() sigue True hasta t_inicio + costo escalado en tiempo simulado; su costo no cuenta en el cuadro.
# TWIN_ASYNC_PEN: factor de cuadro mientras corre la DP (GIL/cores compartidos con la visión).
import threading as _th
PEN = float(os.environ.get("TWIN_ASYNC_PEN", "1"))
st.job_end = None
_RealThread = _th.Thread
class _FakeJob:
    def __init__(self, target=None, args=(), kwargs=None, daemon=None, **k):
        self.target, self.args, self.kwargs = target, args, kwargs or {}
    def start(self):
        dp0 = st.dp; t = time.thread_time()
        self.target(*self.args, **self.kwargs)
        w = time.thread_time() - t; d = st.dp - dp0
        st.dp = dp0
        if st.t0 is not None: st.t0 += w
        now = st.tk + (costo() if st.t0 is not None else 0.0)
        st.job_end = max(now, st.tend) + d * FD + (w - d) * FF
    def is_alive(self):
        return st.job_end is not None and st.tend < st.job_end
def _thread(*a, **k):
    tgt = k.get("target")
    if getattr(tgt, "__name__", "") == "_run_plan": return _FakeJob(*a, **k)
    return _RealThread(*a, **k)
_th.Thread = _thread
_costo0 = costo
def costo():
    c = _costo0()
    return c * PEN if (st.job_end is not None and st.tk < st.job_end) else c
class COST:
    @staticmethod
    def begin(tk, tend): st.t0 = time.thread_time(); st.tk = tk; st.tend = tend; st.dp = 0.0
    @staticmethod
    def next_tk(tk, tend, period):
        c = costo(); st.t0 = None; st.costs.append(c)
        return tk + max(period, c)
# TWIN_WARM=1: llena las cachés de la DP al crear RouteMode (antes de armar; su costo cae en un cuadro desarmado)
if os.environ.get("TWIN_WARM") == "1":
    from pure_pursuit import route_mode as RM
    _init = RM.RouteMode.__init__
    def _winit(self, *a, **k):
        if not getattr(RM, "_warm_done", False):
            for d in ("CW", "CCW"): RR.plan(d, {}, None, prm=RM._route_params())
            RM._warm_done = True
        _init(self, *a, **k)
    RM.RouteMode.__init__ = _winit
# sim.py parchado en memoria (el repo no cambia)
import pure_pursuit.twin as PT
src_path = os.path.join(os.path.dirname(PT.__file__), "sim.py")
src = open(src_path).read()
a = "                        t_proc0 = time.perf_counter()\n"
b = "                        t_k = t_end\n                        pi_k += 1\n"
assert src.count(a) == 1 and src.count(b) == 1, "sim.py cambió: revisar parche"
src = src.replace(a, a + "                        _COST.begin(t_k, t_end)\n")
src = src.replace(b, "                        t_k = _COST.next_tk(t_k, t_end, self.pi_period_s)\n                        pi_k += 1\n")
mod = types.ModuleType("pure_pursuit.twin.sim"); mod.__file__ = src_path; mod.__package__ = "pure_pursuit.twin"
mod._COST = COST
sys.modules["pure_pursuit.twin.sim"] = mod; PT.sim = mod
exec(compile(src, src_path, "exec"), mod.__dict__)
def report():
    c = sorted(st.costs) or [0]
    n = len(c); q = lambda p: c[min(n - 1, int(p * n))] * 1000
    sys.stdout.write("COSTO " + json.dumps({"FF": FF, "FD": FD, "cuadros": n, "media_ms": sum(c) / n * 1000,
        "p50_ms": q(.5), "p95_ms": q(.95), "max_ms": c[-1] * 1000, "pct_mas_25ms": sum(x > .025 for x in c) / n * 100,
        "dp_ms": [round(d * 1000) for d in st.dps]}) + "\n"); sys.stdout.flush()
atexit.register(report)
seed, out = sys.argv[1], sys.argv[2]
_cfg = json.loads(open("pure_pursuit/twin/tools/carrera_bench.json").read())
_extra = json.loads(os.environ.get("TWIN_PI_EXTRA", "{}"))  # p. ej. {"ROUTE_PLAN_ASYNC": true}
from pure_pursuit import config as _PC
# sim.py ignora en silencio claves sin default en config.py: el banco mediría un no-op.
_bad = [k for k in _extra if not hasattr(_PC, k)]
if _bad: sys.exit(f"TWIN_PI_EXTRA: claves sin default en config.py (serían no-op): {_bad}")
_cfg.setdefault("pi", {}).update(_extra)
_cfg.setdefault("fw", {}).update(json.loads(os.environ.get("TWIN_FW_OVER", "{}")))  # p. ej. {"PARK_PL_W_DET": "145"}
sys.argv = ["drive.py", seed, out, "ruta_perfil", json.dumps(_cfg), "--solo-cajon"]
runpy.run_path("pure_pursuit/twin/tools/drive.py", run_name="__main__")
