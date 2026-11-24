"""RondaAbierta (src/ESP32/RondaAbierta) en el gemelo: ronda abierta sin señales ni cajón.
Sirve para comparar el gemelo con corridas reales de RondaAbierta (log "TERMINADO: 12 giros en X s").

Desde src/RASPI/cam (PYTHONPATH=.):
  python -m pure_pursuit.twin.tools.ronda_abierta.run_ra <semilla> [salida.csv] [--ino RUTA]
Sin --ino usa el RondaAbierta.ino del checkout.
RA_VIEJO=1: versión de la corrida real 133246 (29.9 s): sin trim de rumbo y con el frontal sin calibrar.
TWIN_MOTOR=fisico: motor de hoja de datos en vez del medido.
OPEN_MAP=1: elige un rectángulo Open por semilla, observa la vuelta 1 y
compara geometry() con el layout real del gemelo."""
import ctypes, json, math, os, re, subprocess, sys, tempfile
from pathlib import Path
import numpy as np
from pure_pursuit.open_map import OpenWallMap, field_inner_box_from_corridors, open_layout_from_seed
from pure_pursuit.track_map import field_heading_from_twin
from pure_pursuit.wro_field import OUTER_HALF_MM, randomize, section_to_world
from pure_pursuit.twin.firmware import build as B
from pure_pursuit.twin.firmware.fw import FirmwareSIL
from pure_pursuit.twin.params import TwinParams, physical_motor
from pure_pursuit.twin.sensors import ToF, Bno085Heading, Pose
from pure_pursuit.twin.vehicle import Vehicle
from pure_pursuit.twin.world import World, collision

HERE = Path(__file__).resolve().parent
OUT = Path(tempfile.gettempdir()) / "fox_ronda_abierta"


def patch(t: str) -> str:
    if "class LogAsync" in t:
        # El gemelo no ejecuta tareas FreeRTOS. Envía el mismo log sin cola y
        # arranca la UART de la Pi en el hilo del firmware simulado.
        t = re.sub(r"static portMUX_TYPE logMux\b.*?LogAsync Log;",
                   """class LogAsync : public Print {
public:
  size_t write(uint8_t c) override { return Serial.writeChar((char)c); }
  size_t write(const uint8_t* buf, size_t n) override {
    return Serial.writeRaw((const char*)buf, n);
  }
};
LogAsync Log;""", t, count=1, flags=re.S)
        t = re.sub(r"void logTarea\(void\*\) \{.*?^\}", "", t, count=1, flags=re.S | re.M)
        t = re.sub(r'^  xTaskCreatePinnedToCore\(logTarea,.*\n', "", t, count=1, flags=re.M)
        t = re.sub(r"    static volatile bool listo = false;.*?    while \(!listo\) delay\(1\);",
                   "    Serial2.begin(PI_BAUD, SERIAL_8N1, 17, 16);", t, count=1, flags=re.S)
    else:
        t = t.replace("Serial.write(c);", "Serial.writeChar((char)c);")
        t = t.replace("Serial.write(buf, n);", "Serial.writeRaw((const char*)buf, n);")
    reps = [("ARRANQUE_POR_COMANDO = true", "ARRANQUE_POR_COMANDO = false"),
            # El BNO simulado emite paquetes cuando avanza el reloj del SIL.
            ("millis() - t0 < 3000) leerBno();", "millis() - t0 < 3000) { leerBno(); delay(1); }")]
    if os.environ.get("RA_VIEJO"):
        reps += [("float KiWall = 0.08;", "float KiWall = 0;"),
                 ("CORR_FRONTAL_MM[3] = { 30, 200,  30 }", "CORR_FRONTAL_MM[3] = { 0, 0, 0 }")]
    t = t.replace("esp_reset_reason()", "1")  # el SIL no trae esp_system.h
    for a, b in reps:
        assert a in t, f"RondaAbierta cambió, revisa el parche: {a}"
        t = t.replace(a, b)
    return t


def build(ino_text: str) -> Path:
    OUT.mkdir(exist_ok=True)
    cpp = OUT / f"ra_{os.getpid()}.cpp"
    cpp.write_text('#include "Arduino.h"\n#include "BluetoothSerial.h"\nbool piReadyReceived = true;\n' + patch(ino_text))
    lib = OUT / f"libra_{os.getpid()}.dylib"
    subprocess.run([*B._cxx(), "-std=c++17", "-O1", "-fPIC", "-shared", "-Wno-everything", f"-I{HERE / 'inc'}",
                    f"-I{B._SIL}", "-DFOX_SIL=1", *(["-DRA_FRONT_INFLA=1"] if os.environ.get("RA_VIEJO") else []),
                    str(B._SIL / "sil_core.cpp"), str(B._SIL / "sil_vehicle.cpp"), str(cpp), "-o", str(lib)], check=True)
    cpp.unlink()
    return lib


def main():
    args = sys.argv[1:]
    ino = None
    if "--ino" in args:
        i = args.index("--ino"); ino = Path(args[i + 1]).read_text(); del args[i:i + 2]
    if ino is None:
        ino = (B._REPO / "src/ESP32/RondaAbierta/RondaAbierta.ino").read_text(encoding="utf-8")
    seed = int(args[0])
    rng = np.random.default_rng(seed)
    p = TwinParams()
    if os.environ.get("TWIN_MOTOR") == "fisico":
        for k, v in physical_motor(mass_kg=0.564, gear=float(os.environ.get("TWIN_GEAR", 1.25)),
                                   v_batt=float(os.environ.get("TWIN_VBATT", 12.6))).items():
            setattr(p.motor, k, v)
    # Servo real (RondaAbierta escribe grados de servo): centro 77 y topes 2..150 medidos 2026-10-21.
    st = p.steering
    center = re.search(r"\bcentroServo\s*=\s*(\d+)\s*;", ino)
    st.servo_center_deg = float(center[1]) if center else 77.0
    st.servo_min_deg, st.servo_max_deg = 2.0, 150.0
    # VL53L1X en Long a 50 ms; la pared negra se ve a 80 cm+ (calib/medidas_20261021/tof), no los 250 mm del VL53L0X.
    p.tof.sample_period_s_by_id = (0.05,) * 4
    p.tof.max_range_mm_by_id = (4000.0,) * 4
    p.tof.black_wall_max_mm = float(os.environ.get("RA_TOF_NEGRO_MM", 2000))
    f = randomize(seed)
    f.signs, f.barriers = [], []
    truth = open_layout_from_seed(seed) if os.environ.get("OPEN_MAP") == "1" else None
    if truth is not None:
        print("OPEN_MAP truth:", json.dumps(truth))
        f.start_x, f.start_y = section_to_world(f.parking_section, truth[f.parking_section] / 2, OUTER_HALF_MM)
    world = World(f, inner_box_mat=field_inner_box_from_corridors(truth) if truth is not None else None)
    veh = Vehicle(p, f.start_x, f.start_y, f.start_heading)
    tof, bno = ToF(p.tof, rng), Bno085Heading(p.bno, rng)
    lib_path = build(ino)
    fw = FirmwareSIL.__new__(FirmwareSIL)
    fw._lib = ctypes.CDLL(str(lib_path))
    fw._tmp = None
    fw._native_sync = None
    fw._debug_buf = ctypes.create_string_buffer(65536)
    fw._advance_cb = fw._sonar_cb = fw._gyro_cb = fw._encoder_cb = fw._tof_cb = None
    fw._bind()
    t_s = lambda: fw._lib.sil_now_us() / 1e6
    hit, rows, last = [None], [], [-1.0]
    L, W, R = p.vehicle.length_mm, p.vehicle.width_mm, p.vehicle.rear_overhang_mm
    wall_map = None
    if truth is not None:
        wall_map = OpenWallMap("L" if f.direction == "CCW" else "R", start_section=f.parking_section)
    lateral_tof = {}
    mapping_straight = False
    mapping_done = False
    odom_mm = 0.0

    def advance(us):
        nonlocal odom_mm
        x, y = veh.x, veh.y
        rem = us / 1e6
        while rem > 1e-12:
            sub = min(rem, 0.001); rem -= sub
            veh._integrate_substep(sub, fw.servo_angle(), float(fw.motor_pwm()), fw.motor_dir())
        if wall_map is not None and not mapping_done:
            odom_mm += math.hypot(veh.x - x, veh.y - y)
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
        t = t_s()
        mm = tof.read_mm(idx, world, Pose(veh.x, veh.y, veh.heading_deg), t)
        if wall_map is not None and mapping_straight and not mapping_done and idx in (0, 1):
            lateral_tof[idx] = mm
            if len(lateral_tof) == 2:
                pose = (veh.x, veh.y, field_heading_from_twin(veh.heading_deg))
                wall_map.update_straight(t * 1000.0, pose, lateral_tof[0], lateral_tof[1], None, odom_mm)
                lateral_tof.clear()
        return mm

    fw.set_callbacks(advance=advance, sonar=lambda _i: 0, gyro=lambda: 0.0, encoder=lambda: 0, tof=tof_cb)
    fw.setup()
    log = pending_log = ""
    while t_s() < 90 and hit[0] is None:
        fw.loop()
        chunk = fw.pop_debug()
        log += chunk
        pending_log += chunk
        lines, newline, pending_log = pending_log.rpartition("\n")
        if wall_map is not None and not mapping_done and newline:
            for line in lines.splitlines():
                if line.startswith("-> ARRANCA"):
                    mapping_straight = True
                elif re.match(r"Giro \d+ (izquierda|derecha)", line):
                    mapping_straight = False
                    lateral_tof.clear()
                elif match := re.match(r"Giro completado (\d+)/", line):
                    turn = int(match[1])
                    wall_map.on_corner_complete(turn)
                    mapping_straight = True
                    if turn == 4:
                        mapping_done = True
                        summary = "[OPEN_MAP] " + json.dumps(wall_map.geometry())
                        print(summary)
                        log += summary + "\n"
                        break
        if "TERMINADO" in log or "NO ARRANCA" in log:
            break
    lib_path.unlink(missing_ok=True)
    m = re.search(r"TERMINADO: (\d+) giros en ([\d.]+) s", log)
    print(f"seed={seed} dir={f.direction} res={'%s giros %s s' % m.groups() if m else 'NO'} choque={hit[0]}")
    if wall_map is not None:
        geo = wall_map.geometry()
        mapped = {sec: data["corridor_mm"] if data["frozen"] else None for sec, data in geo["sections"].items()}
        missing = [sec for sec, mm in mapped.items() if mm is None]
        mismatch = [sec for sec, mm in mapped.items() if mm is not None and mm != truth[sec]]
        status = "OK" if not (missing or mismatch) else "WARN"
        print(f"OPEN_MAP {status} mapped={json.dumps(mapped)} missing={missing} mismatch={mismatch}")
    if len(args) > 1:
        out = Path(args[1])
        out.write_text("t,x,y,h,v,pwm,servo\n" + "\n".join(",".join(f"{c:.3f}" for c in r) for r in rows))
        out.with_suffix(".log").write_text(log)


if __name__ == "__main__":
    main()
