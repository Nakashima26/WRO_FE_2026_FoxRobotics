"""Le da al Vehicle del gemelo los mismos comandos (servo, pwm) que dio el carro en una
vuelta real de SeguirRuta y compara, por ventanas, el giro (BNO) y lo recorrido (encoder).
En cada ventana el gemelo arranca con la velocidad real, así el error no se acumula.

  python twin/tools/calib/replay_seguir_ruta.py ../WRO2026/logs_tof/ruta_vuelta_20261024_125707.txt
(logs de SeguirRuta, rama parking; "viejo" = parámetros antes del 2026-10-24)
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")))
from pure_pursuit.twin.params import TwinParams  # noqa: E402
from pure_pursuit.twin.vehicle import Vehicle  # noqa: E402

VENTANA_S = 0.5


def viejos():
    p = TwinParams()
    st, m = p.steering, p.motor
    st.servo_center_deg, st.gain_left, st.gain_right = 78.0, 52.94, 49.71
    st.table_left = st.table_right = ()
    st.backlash_deg, st.deadband_deg, st.lag_tau_s = 0.0, 2.0, 0.0
    m.k_mm_s_per_pwm, m.pwm0, m.coast_decel_mm_s2 = 4.02, 42.1, None
    return p


def leer(f):
    P = []
    for lin in open(f, encoding="utf-8"):
        if lin.startswith("P,"):
            c = lin.strip().split(",")
            P.append([float(z) for z in c[1:10]])
    return np.array(P)   # t, x, y, th, i, lat, servo, pwm, giro


def replay(P, params):
    t = P[:, 0] / 1000
    d = np.concatenate([[0], np.cumsum(np.hypot(np.diff(P[:, 1]), np.diff(P[:, 2])))])
    th = np.unwrap(np.radians(P[:, 3]))
    v_real = np.gradient(d, t)
    errs_th, errs_d, rows = [], [], []
    i = 0
    veh = Vehicle(params, 0.0, 0.0, 0.0, servo_deg=P[0, 6])
    while True:
        j = np.searchsorted(t, t[i] + VENTANA_S)
        if j >= len(t) - 1:
            break
        # Estado del gemelo: velocidad real; servo y juego los arrastra de la ventana anterior.
        veh.v = veh.v_wheel = v_real[i]
        veh.sliding = False
        veh.heading_deg, veh.x, veh.y = 0.0, 0.0, 0.0
        s = 0.0
        for k in range(i, j):
            dt = t[k + 1] - t[k]
            pwm = P[k, 7]
            info = veh.step(dt, P[k, 6], pwm, 1 if pwm > 0 else 0)
            s += info.ds_true_mm
        dth_twin = -veh.heading_deg                    # gemelo: + = horario
        dth_real = np.degrees(th[j] - th[i])
        errs_th.append(dth_twin - dth_real)
        errs_d.append(s - (d[j] - d[i]))
        rows.append((t[i], np.mean(P[i:j, 6]), dth_real, dth_twin, d[j] - d[i], s))
        i = j
    return np.array(errs_th), np.array(errs_d), np.array(rows)


def main():
    for f in sys.argv[1:]:
        P = leer(f)
        P = P[P[:, 7] > 0]                             # solo mientras el motor empuja
        print(f.split("\\")[-1].split("/")[-1], f"{len(P)} muestras, {P[-1, 0] / 1000:.1f} s")
        for nombre, params in (("viejo", viejos()), ("nuevo", TwinParams())):
            et, ed, rows = replay(P, params)
            curva = np.abs(rows[:, 2]) > 20
            print(f"  {nombre}: giro por {VENTANA_S} s  sesgo {et.mean():+.2f}°  rms {np.sqrt(np.mean(et ** 2)):.2f}°"
                  f"  (curvas rms {np.sqrt(np.mean(et[curva] ** 2)):.2f}°, rectas rms {np.sqrt(np.mean(et[~curva] ** 2)):.2f}°)"
                  f" | avance sesgo {ed.mean():+.1f} mm rms {np.sqrt(np.mean(ed ** 2)):.1f} mm"
                  f" | total gemelo {rows[:, 5].sum() / 1000:.2f} m vs real {rows[:, 4].sum() / 1000:.2f} m,"
                  f" giro total gemelo {rows[:, 3].sum():.0f}° vs real {rows[:, 2].sum():.0f}°")


if __name__ == "__main__":
    main()
