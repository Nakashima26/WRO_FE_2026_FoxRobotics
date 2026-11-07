"""RondaAbierta (rama parking, src/ESP32/RondaAbierta) en el gemelo: ronda abierta sin señales ni cajón.
Sirve para comparar el gemelo con corridas reales de RondaAbierta (log "TERMINADO: 12 giros en X s").

Desde src/RASPI/cam (PYTHONPATH=.):
  python -m pure_pursuit.twin.tools.ronda_abierta.run_ra <semilla> [salida.csv] [--ino RUTA]
Sin --ino usa `git show origin/parking:src/ESP32/RondaAbierta/RondaAbierta.ino`.
RA_VIEJO=1: versión de la corrida real 133246 (29.9 s): sin trim de rumbo y con el frontal sin calibrar.
TWIN_MOTOR=fisico: motor de hoja de datos en vez del medido."""
import ctypes, os, re, subprocess, sys, tempfile
from pathlib import Path
import numpy as np
from pure_pursuit.wro_field import randomize
from pure_pursuit.twin.firmware import build as B
from pure_pursuit.twin.firmware.fw import FirmwareSIL
from pure_pursuit.twin.params import TwinParams, physical_motor
from pure_pursuit.twin.sensors import ToF, Bno085Heading, Pose
from pure_pursuit.twin.vehicle import Vehicle
from pure_pursuit.twin.world import World, collision

HERE = Path(__file__).resolve().parent
OUT = Path(tempfile.gettempdir()) / "fox_ronda_abierta"


def patch(t: str) -> str:
    reps = [("Serial.write(c);", "Serial.writeChar((char)c);"),
            ("Serial.write(buf, n);", "Serial.writeRaw((const char*)buf, n);"),
            ("Serial.read()", "-1"),
            ("ARRANQUE_POR_COMANDO = true", "ARRANQUE_POR_COMANDO = false"),
            # en el SIL el tiempo solo avanza con delay(): sin esto la espera del BNO no termina
            ("millis() - t0 < 3000) leerBno();", "millis() - t0 < 3000) { leerBno(); delay(1); }")]
    if os.environ.get("RA_VIEJO"):
        reps += [("float KiWall = 0.08;", "float KiWall = 0;"),
                 ("CORR_FRONTAL_MM[3] = { 30, 200,  30 }", "CORR_FRONTAL_MM[3] = { 0, 0, 0 }")]
    for a, b in reps:
        assert a in t, f"RondaAbierta cambió, revisa el parche: {a}"
        t = t.replace(a, b)
    return t


def build(ino_text: str) -> Path:
    OUT.mkdir(exist_ok=True)
    cpp = OUT / f"ra_{os.getpid()}.cpp"
    cpp.write_text('#include "Arduino.h"\nbool piReadyReceived = true;\n' + patch(ino_text))
    lib = OUT / f"libra_{os.getpid()}.dylib"
    subprocess.run([*B._cxx(), "-std=c++17", "-O1", "-fPIC", "-shared", "-Wno-everything", f"-I{HERE / 'inc'}",
                    f"-I{B._SIL}", "-DFOX_SIL=1", *(["-DRA_FRONT_INFLA=1"] if os.environ.get("RA_VIEJO") else []),
                    str(B._SIL / "sil_core.cpp"), str(cpp), "-o", str(lib)], check=True)
    cpp.unlink()
    return lib


def main():
    args = sys.argv[1:]
    ino = None
    if "--ino" in args:
        i = args.index("--ino"); ino = Path(args[i + 1]).read_text(); del args[i:i + 2]
    if ino is None:
        ino = subprocess.run(["git", "show", "origin/parking:src/ESP32/RondaAbierta/RondaAbierta.ino"],
                             check=True, capture_output=True, text=True).stdout
    seed = int(args[0])
    rng = np.random.default_rng(seed)
    p = TwinParams()
    if os.environ.get("TWIN_MOTOR") == "fisico":
        for k, v in physical_motor(mass_kg=0.564, gear=float(os.environ.get("TWIN_GEAR", 1.25)),
                                   v_batt=float(os.environ.get("TWIN_VBATT", 12.6))).items():
            setattr(p.motor, k, v)
    # Servo real (RondaAbierta escribe grados de servo): centro 77 y topes 2..150 medidos 2026-10-21.
    st = p.steering
    st.servo_center_deg, st.servo_min_deg, st.servo_max_deg = 77.0, 2.0, 150.0
    # VL53L1X en Long a 50 ms; la pared negra se ve a 80 cm+ (calib/medidas_20261021/tof), no los 250 mm del VL53L0X.
    p.tof.sample_period_s_by_id = (0.05,) * 4
    p.tof.max_range_mm_by_id = (4000.0,) * 4
    p.tof.black_wall_max_mm = float(os.environ.get("RA_TOF_NEGRO_MM", 2000))
    f = randomize(seed)
    f.signs, f.barriers = [], []
    world = World(f)
    veh = Vehicle(p, f.start_x, f.start_y, f.start_heading)
    tof, bno = ToF(p.tof, rng), Bno085Heading(p.bno, rng)
    lib_path = build(ino)
    fw = FirmwareSIL.__new__(FirmwareSIL)
    fw._lib = ctypes.CDLL(str(lib_path))
    fw._tmp = None
    fw._advance_cb = fw._sonar_cb = fw._gyro_cb = fw._encoder_cb = fw._tof_cb = None
    fw._bind()
    t_s = lambda: fw._lib.sil_now_us() / 1e6
    hit, rows, last = [None], [], [-1.0]
    L, W, R = p.vehicle.length_mm, p.vehicle.width_mm, p.vehicle.rear_overhang_mm

    def advance(us):
        rem = us / 1e6
        while rem > 1e-12:
            sub = min(rem, 0.001); rem -= sub
            veh._integrate_substep(sub, fw.servo_angle(), float(fw.motor_pwm()), fw.motor_dir())
        t = t_s()
        if t - last[0] >= 0.02:
            last[0] = t
            rows.append((t, veh.x, veh.y, veh.heading_deg, abs(veh.v), fw.motor_pwm(), fw.servo_angle()))
            if hit[0] is None:
                h = collision(world, veh.x, veh.y, veh.heading_deg, L, W, R)
                if h:
                    hit[0] = (round(t, 2), h)

    def tof_cb(idx):
        if idx == 99:
            bno.read(veh.heading_deg, t_s())
            y = -(bno._sample or 0.0)          # CCW -> yaw del BNO (crece a la derecha)
            return int(round(((y + 180.0) % 360.0 - 180.0) * 100))
        return tof.read_mm(idx, world, Pose(veh.x, veh.y, veh.heading_deg), t_s())

    fw.set_callbacks(advance=advance, sonar=lambda _i: 0, gyro=lambda: 0.0, encoder=lambda: 0, tof=tof_cb)
    fw.setup()
    log = ""
    while t_s() < 90 and hit[0] is None:
        fw.loop()
        log += fw.pop_debug() or ""
        if "TERMINADO" in log or "NO ARRANCA" in log:
            break
    lib_path.unlink(missing_ok=True)
    m = re.search(r"TERMINADO: (\d+) giros en ([\d.]+) s", log)
    print(f"seed={seed} dir={f.direction} res={'%s giros %s s' % m.groups() if m else 'NO'} choque={hit[0]}")
    if len(args) > 1:
        out = Path(args[1])
        out.write_text("t,x,y,h,v,pwm,servo\n" + "\n".join(",".join(f"{c:.3f}" for c in r) for r in rows))
        out.with_suffix(".log").write_text(log)


if __name__ == "__main__":
    main()
