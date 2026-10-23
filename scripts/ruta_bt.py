"""
Rutas de prueba para SeguirRuta.ino: genera una ruta, la simula con el mismo
pure pursuit del ESP32, la sube por Bluetooth (o USB), arranca el carro, graba
el log y grafica la ruta pedida contra la que hizo el carro (y la simulada).

Uso:
  python scripts/ruta_bt.py recta                         2 m derecho
  python scripts/ruta_bt.py recta --largo 3000 --pwm 180
  python scripts/ruta_bt.py carril --desp 300             cambio de carril de 30 cm (+ = izquierda)
  python scripts/ruta_bt.py s --radio 400 --angulo 50     slalom de arcos
  python scripts/ruta_bt.py vuelta --sentido izq --radio 300 --vueltas 3
  python scripts/ruta_bt.py archivo mi_ruta.csv           puntos "x,y" en mm, uno por renglón
  python scripts/ruta_bt.py ver logs_tof/ruta_recta_FECHA.txt   vuelve a graficar un log

Opciones de todas: --sim (solo simula y grafica, no se conecta), --puerto COM9
(COM9 = cable USB; sin puerto busca el Bluetooth del ESP32), --pwm, --pwm-curva,
--lookahead, --paso, --auto (arranca sin esperar Enter), --no-mostrar,
--pista 2975 (lado real de la pista), --pared N (corrección con paredes, abajo).

PAREDES: el carro registra los 3 ToF (renglones T del log). Con eso la gráfica
pone, en cada recta, dónde dicen las paredes que estaba el carro (puntos L/R)
junto a donde cree él (línea naranja): la diferencia es lo que se le fue la
odometría. --pared además lo corrige en el carro (bits, se suman):
  1 posición con los laterales   2 rumbo con los laterales   4 avance con el frontal
  8 usar también el cuadro interior (sin él, solo las paredes exteriores)
  --pared 1 = solo posición con las exteriores   --pared 0 = solo registrar (default)

PARÁMETROS SIN SUBIR EL SKETCH: --set clave=valor (se repite) cambia cualquier
parámetro del ESP32 por nombre (la lista sale con PAR; queda en el log como
"# par ..."). Por ejemplo:
  --set encCurva=0.08 --set salidaPared=1 --set frenoAntes=25 --set paredKPos=0.2
Al final se imprime una línea RESUMEN con lo que dicen las paredes (para comparar
corridas).

COORDENADAS (las de VueltaCalibracion): mm, origen en la esquina EXTERIOR de la
pista detrás del carro, th en grados (0 = +x, crece a la izquierda). El punto
del carro es el centro del EJE TRASERO. Antes de arrancar el script dice dónde
poner el carro; si pruebas en el piso, solo respeta la distancia a una pared o
marca de cinta "de atrás" (x) y "de la derecha" (y).

OJO: el log es la posición que el carro CREE tener (encoder + BNO). Si dice que
siguió perfecto pero en el piso se fue chueco, el error es de odometría, no del
seguidor: al terminar mide dónde quedó de verdad el eje trasero y compáralo con
la posición final que imprime el script.

El log queda en logs_tof/ruta_<tipo>_FECHA.txt y la gráfica junto a él (.png).
"""
import argparse
import datetime
import math
import os
import sys
import threading
import time

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ── Pista (mm) ────────────────────────────────────────────────────────────────
PISTA_MM = 3000.0          # por omisión; --pista la cambia (la lona del 2026-10-23 mide 2975)


def mapa_pista(pista=PISTA_MM, interior=None):
    """Cuadro exterior e interior (x0, y0, x1, y1), mm, origen en la esquina exterior.
    Sin `interior`, todo a escala de la pista: el interior mide un tercio y queda centrado
    (lona de 2975 -> interior de 991.7, pasillos de 991.7)."""
    interior = pista / 3 if interior is None else interior
    c, m = pista / 2, interior / 2
    return (0.0, 0.0, pista, pista), (c - m, c - m, c + m, c + m)


# ToF como en SeguirRuta.ino: (adelante, izquierda, ángulo °) de la cara que mide desde
# el centro del eje trasero, y las mismas reglas para decidir si una lectura sirve.
TOFS = {"F": (141.4, 0.0, 0.0), "L": (113.0, 35.2, 90.0), "R": (113.0, -35.2, -90.0)}
PARED_GATE_MM = 120.0
PARED_INC_MAX_DEG = 20.0
PARED_RECTA_DEG = 8.0
PARED_RUEDA_MAX = 12.0
CONO_MEDIO_DEG = 7.5

# ── Igual que SeguirRuta.ino ──────────────────────────────────────────────────
BATALLA_MM = 113.0
CENTRO_SERVO = 75   # el default de SeguirRuta.ino; --centro lo cambia
SERVO_MIN, SERVO_MAX = 1, 156
SERVO_POR_RUEDA_IZQ = 1.75
SERVO_POR_RUEDA_DER = 1.78
RUEDA_MAX_IZQ = 40.0
RUEDA_MAX_DER = 42.5
VENTANA_MM = 600.0
RUEDA_CURVA_MIN, RUEDA_CURVA_MAX = 10.0, 30.0
PWM_ARRANQUE, RAMPA_MS = 90, 500
LAT_MAX_MM = 400.0
RUTA_MAX = 1500

# Radio más cerrado medido en el tapete (eje trasero): der 123, izq 134 mm.
# Una ruta más cerrada que esto el carro no la puede seguir.
RADIO_MIN_MM = 134.0
RADIO_AVISO_MM = 180.0   # entre esto y RADIO_MIN_MM va a tope: se puede, pero sin margen

# ── Simulación ────────────────────────────────────────────────────────────────
# Velocidad en recta: PWM 180 ≈ 540 mm/s (vuelta_20261019_125759), PWM 90 ≈ 145 mm/s
# (CalibFox 2026-10-22) -> recta por esos dos puntos.
def vel_por_pwm(pwm):
    return max(0.0, 4.39 * (pwm - 57))

# Retraso del servo (primer orden). 0.12 s reproduce el rango del servo y el error
# de las vueltas reales a PWM 210, 240 y 255 (2026-10-23); con 0.08 la simulación
# no oscilaba al salir de las esquinas y el carro sí.
TAU_RUEDA_S = 0.12
VEL_MS = 20          # ventana con la que el ESP32 mide velocidad y giro (kd)
DT_SIM = 0.01

COLOR_RUTA = "#8a8a85"
COLOR_SIM = "#2a78d6"
COLOR_REAL = "#eb6834"
COLOR_PARED = {"L": "#1baf7a", "R": "#eda100"}


# ═══════════════════════════════════════════════════════════════════════════════
# Rutas
# ═══════════════════════════════════════════════════════════════════════════════

class Tortuga:
    """Dibuja con rectas y arcos tangentes (sin quiebres de rumbo)."""

    def __init__(self, x, y, th_deg):
        self.x, self.y, self.th = x, y, math.radians(th_deg)
        self.pts = [(x, y)]

    def recta(self, largo):
        n = max(1, int(abs(largo) / 5))
        for k in range(1, n + 1):
            d = largo * k / n
            self.pts.append((self.x + d * math.cos(self.th), self.y + d * math.sin(self.th)))
        self.x, self.y = self.pts[-1]

    def arco(self, radio, ang_deg):
        """ang > 0 = a la izquierda."""
        lado = 1 if ang_deg > 0 else -1
        cx = self.x - lado * radio * math.sin(self.th)
        cy = self.y + lado * radio * math.cos(self.th)
        a0 = self.th - lado * math.pi / 2
        ang = math.radians(ang_deg)
        n = max(2, int(abs(ang) * radio / 5))
        for k in range(1, n + 1):
            a = a0 + ang * k / n
            self.pts.append((cx + radio * math.cos(a), cy + radio * math.sin(a)))
        self.th += ang
        self.x, self.y = self.pts[-1]


def remuestrear(pts, paso, cerrada):
    """Puntos separados exactamente `paso` mm (o lo más cerca que dé el largo)."""
    pts = np.asarray(pts, float)
    if cerrada and np.hypot(*(pts[0] - pts[-1])) > 1e-6:
        pts = np.vstack([pts, pts[:1]])
    d = np.hypot(*np.diff(pts, axis=0).T)
    pts = pts[np.concatenate([[True], d > 1e-9])]
    d = d[d > 1e-9]
    s = np.concatenate([[0.0], np.cumsum(d)])
    largo = s[-1]
    if cerrada:
        n = max(10, round(largo / paso))
        paso_r = largo / n
        ss = np.arange(n) * paso_r
    else:
        n = max(10, round(largo / paso) + 1)
        paso_r = largo / (n - 1)
        ss = np.linspace(0, largo, n)
    return np.column_stack([np.interp(ss, s, pts[:, 0]), np.interp(ss, s, pts[:, 1])]), paso_r


def ruta_recta(a):
    t = Tortuga(a.x0, a.y0, 0)
    t.recta(a.largo)
    return t.pts, False, f"recta {a.largo:.0f} mm"


def ruta_carril(a):
    """Recta, cambio de carril suave (coseno) y recta."""
    pts = [(a.x0, a.y0)]
    x = a.x0 + a.antes
    pts.append((x, a.y0))
    for k in range(1, 201):
        u = k / 200
        pts.append((x + u * a.cambio, a.y0 + a.desp * (1 - math.cos(math.pi * u)) / 2))
    pts.append((x + a.cambio + a.despues, a.y0 + a.desp))
    return pts, False, f"carril {a.desp:+.0f} mm en {a.cambio:.0f} mm"


def ruta_s(a):
    """Slalom de arcos tangentes: +ang, -2ang, +2ang, ..., y vuelve al rumbo 0."""
    t = Tortuga(a.x0, a.y0, 0)
    t.recta(a.antes)
    giros = [a.angulo] + [(-1) ** (k + 1) * 2 * a.angulo for k in range(a.curvas)]
    giros.append(-sum(giros))
    for g in giros:
        t.arco(a.radio, g)
    t.recta(a.despues)
    return t.pts, False, f"s radio {a.radio:.0f} ángulo {a.angulo:.0f}° curvas {a.curvas}"


def ruta_vuelta(a):
    """Una vuelta a la pista por el centro de los pasillos (a `carril` mm de la
    pared exterior), esquinas en arco de `radio`. Sentido izq = antihorario."""
    d = a.pista / 6 if a.carril is None else a.carril   # sin --carril: centro del pasillo, a escala
    lado = a.pista - 2 * d
    inicio = lado / 2 if a.inicio is None else a.inicio
    if not (a.radio <= inicio <= lado - a.radio):
        sys.exit(f"--inicio tiene que quedar en la recta: entre {a.radio:.0f} y {lado - a.radio:.0f}")
    t = Tortuga(d + inicio, d, 0)
    t.recta(lado - a.radio - inicio)
    for k in range(4):
        t.arco(a.radio, 90)
        t.recta(lado - 2 * a.radio if k < 3 else inicio - a.radio)
    pts = t.pts
    if a.sentido == "der":
        # Espejo sobre x = y: la exterior queda a la izquierda y gira a la derecha
        # (como VueltaCalibracion DER: sale viendo hacia +y).
        pts = [(py, px) for px, py in pts]
    return pts, True, f"vuelta {a.sentido} radio {a.radio:.0f} carril {d:.0f} x{a.vueltas}"


def ruta_archivo(a):
    pts = []
    with open(a.csv, encoding="utf-8") as f:
        for linea in f:
            c = linea.replace(";", ",").split(",")
            try:
                pts.append((float(c[0]), float(c[1])))
            except (ValueError, IndexError):
                continue   # encabezado o renglón vacío
    if len(pts) < 2:
        sys.exit(f"{a.csv}: no encontré puntos x,y")
    return pts, a.cerrada, f"archivo {os.path.basename(a.csv)}"


def radios(ruta, cerrada):
    """Radio de curvatura en cada punto (círculo por el anterior, él y el siguiente)."""
    p = ruta
    if cerrada:
        a, b, c = np.roll(p, 1, axis=0), p, np.roll(p, -1, axis=0)
    else:
        a, b, c = p[:-2], p[1:-1], p[2:]
    ab, bc, ca = np.hypot(*(b - a).T), np.hypot(*(c - b).T), np.hypot(*(a - c).T)
    cruz = np.abs((b - a)[:, 0] * (c - a)[:, 1] - (b - a)[:, 1] * (c - a)[:, 0])
    with np.errstate(divide="ignore", invalid="ignore"):
        r = ab * bc * ca / (2 * cruz)
    return np.where(cruz < 1e-9, np.inf, r)


# ═══════════════════════════════════════════════════════════════════════════════
# Seguidor: copia de SeguirRuta.ino (si cambias uno, cambia el otro)
# ═══════════════════════════════════════════════════════════════════════════════

class Seguidor:
    def __init__(self, ruta, paso, cerrada, lookahead, vueltas, freno_antes=0.0):
        self.X, self.Y = ruta[:, 0], ruta[:, 1]
        self.n, self.paso, self.cerrada = len(ruta), paso, cerrada
        self.lookahead, self.vueltas = lookahead, vueltas
        self.antes = int(freno_antes / paso)   # frenoAntes del sketch: frena antes de la meta
        self.i, self.progreso = -1, 0

    def idx(self, i):
        return i % self.n if self.cerrada else min(max(i, 0), self.n - 1)

    def punto(self, i):
        n = self.n
        if not self.cerrada and i > n - 1:
            dx, dy = self.X[n - 1] - self.X[n - 2], self.Y[n - 1] - self.Y[n - 2]
            d = max(math.hypot(dx, dy), 1.0)
            extra = (i - (n - 1)) * self.paso
            return self.X[n - 1] + dx / d * extra, self.Y[n - 1] + dy / d * extra
        k = self.idx(i)
        return self.X[k], self.Y[k]

    def cercano(self, x, y, th):
        if self.i < 0:
            c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
            best = 1e18
            for i in range(self.n):
                a, b = i, i + 1
                if not self.cerrada and b >= self.n:
                    a, b = self.n - 2, self.n - 1
                tx, ty = self.X[self.idx(b)] - self.X[a], self.Y[self.idx(b)] - self.Y[a]
                if tx * c + ty * s < 0.3 * self.paso:
                    continue
                d2 = (self.X[i] - x) ** 2 + (self.Y[i] - y) ** 2
                if d2 < best:
                    best, self.i = d2, i
            return self.i >= 0
        best_o, best = 0, 1e18
        for o in range(-5, int(VENTANA_MM / self.paso) + 2):
            i = self.i + o
            if not self.cerrada and (i < 0 or i >= self.n):
                continue
            k = self.idx(i)
            d2 = (self.X[k] - x) ** 2 + (self.Y[k] - y) ** 2
            if d2 < best:
                best, best_o = d2, o
        self.progreso += best_o
        self.i = self.idx(self.i + best_o)
        return True

    def rueda(self, x, y, th):
        px, py = self.punto(self.i + int(self.lookahead / self.paso + 0.5))
        c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
        dx, dy = px - x, py - y
        adelante, izquierda = dx * c + dy * s, -dx * s + dy * c
        ld2 = max(adelante ** 2 + izquierda ** 2, 1.0)
        return math.degrees(math.atan(2 * BATALLA_MM * izquierda / ld2))

    def lateral(self, x, y):
        a, b = self.i, self.i + 1
        if not self.cerrada and b >= self.n:
            a, b = self.n - 2, self.n - 1
        ax, ay = self.X[a], self.Y[a]
        tx, ty = self.X[self.idx(b)] - ax, self.Y[self.idx(b)] - ay
        d = max(math.hypot(tx, ty), 1.0)
        return (tx * (y - ay) - ty * (x - ax)) / d

    def terminada(self):
        return self.falta() <= 0

    def falta(self):
        """mm de ruta que faltan para frenar (como faltaMm() del sketch)."""
        if self.cerrada:
            return (self.vueltas * self.n - self.antes - self.progreso) * self.paso
        return (self.n - 1 - self.antes - self.i) * self.paso


def servo_para_rueda(rueda):
    rueda = min(max(rueda, -RUEDA_MAX_DER), RUEDA_MAX_IZQ)
    s = rueda * (SERVO_POR_RUEDA_IZQ if rueda >= 0 else SERVO_POR_RUEDA_DER)
    return min(max(CENTRO_SERVO + int(round(s)), SERVO_MIN), SERVO_MAX)


def rueda_para_servo(servo):
    """Lo que de verdad gira la rueda con ese servo (inverso del mapeo, con topes)."""
    d = servo - CENTRO_SERVO
    r = d / SERVO_POR_RUEDA_IZQ if d >= 0 else d / SERVO_POR_RUEDA_DER
    return min(max(r, -RUEDA_MAX_DER), RUEDA_MAX_IZQ)


def pwm_para(rueda, t_ms, pwm, pwm_curva):
    f = min(max((abs(rueda) - RUEDA_CURVA_MIN) / (RUEDA_CURVA_MAX - RUEDA_CURVA_MIN), 0.0), 1.0)
    p = round(pwm + (pwm_curva - pwm) * f)
    if t_ms < RAMPA_MS:
        p = min(p, PWM_ARRANQUE + (pwm - PWM_ARRANQUE) * t_ms // RAMPA_MS)
    return p


def simular(ruta, paso, cerrada, salida, a):
    """Bicicleta cinemática en el eje trasero, con el retraso del servo, los
    topes medidos y servo en grados enteros. Devuelve (filas, motivo)."""
    sets = sets_de(a)
    seg = Seguidor(ruta, paso, cerrada, a.lookahead, a.vueltas, float(sets.get("frenoAntes", 0.0)))
    rampa_fin, pwm_fin = float(sets.get("rampaFin", 650.0)), int(float(sets.get("pwmFin", 150)))
    escalon = int(float(sets.get("finEscalon", 1)))
    x, y, th = salida
    if not seg.cercano(x, y, th):
        return np.zeros((0, 8)), "la salida no va en el sentido de la ruta"
    kd = getattr(a, "kd", None) or 0.0
    rueda_real, t, filas = 0.0, 0.0, []
    dist, vel_med, giro_med, prev = 0.0, 0.0, 0.0, None   # prev = (t, dist, th) de la última medición
    while True:
        if t > 120:
            return np.array(filas), "timeout"
        seg.cercano(x, y, th)
        if seg.terminada():
            return np.array(filas), "ruta completa"
        lat = seg.lateral(x, y)
        if abs(lat) > LAT_MAX_MM:
            return np.array(filas), "se salió de la ruta"
        if prev is None or t - prev[0] >= VEL_MS / 1000 - 1e-9:
            if prev is not None:
                vel_med = (dist - prev[1]) / (t - prev[0])
                giro_med = (th - prev[2]) / (t - prev[0])
            prev = (t, dist, th)
        ant = getattr(a, "anticipo", 0.0) or 0.0
        if ant:
            thm = math.radians(th + giro_med * ant * 0.5)
            rueda_ruta = seg.rueda(x + vel_med * ant * math.cos(thm), y + vel_med * ant * math.sin(thm),
                                   th + giro_med * ant)
        else:
            rueda_ruta = seg.rueda(x, y, th)
        rueda = rueda_ruta
        if kd > 0 and vel_med > 100:
            giro_pedido = math.degrees(vel_med * math.tan(math.radians(rueda_ruta)) / BATALLA_MM)
            rueda += kd * (giro_pedido - giro_med)
        servo = servo_para_rueda(rueda)
        pwm = pwm_para(rueda_ruta, t * 1000, a.pwm, a.pwm_curva)
        falta = seg.falta()
        if rampa_fin > 0 and falta < rampa_fin:   # llegada suave, como el sketch
            pwm = min(pwm, pwm_fin if escalon else round(pwm_fin + (a.pwm - pwm_fin) * max(falta, 0.0) / rampa_fin))
        filas.append((t * 1000, x, y, th, seg.i, lat, servo, pwm))
        v = vel_por_pwm(pwm)
        dist += v * DT_SIM
        rueda_real += (rueda_para_servo(servo) - rueda_real) * DT_SIM / TAU_RUEDA_S
        x += v * DT_SIM * math.cos(math.radians(th))
        y += v * DT_SIM * math.sin(math.radians(th))
        th += math.degrees(v * DT_SIM * math.tan(math.radians(rueda_real)) / BATALLA_MM)
        t += DT_SIM


def resumen(nombre, filas):
    if len(filas) == 0:
        return f"{nombre}: sin datos"
    lat = filas[:, 5]
    dist = np.sum(np.hypot(np.diff(filas[:, 1]), np.diff(filas[:, 2])))
    return (f"{nombre}: lateral rms {np.sqrt(np.mean(lat ** 2)):.0f} mm, max {np.max(np.abs(lat)):.0f} mm | "
            f"{dist:.0f} mm en {filas[-1, 0] / 1000:.1f} s | final x={filas[-1, 1]:.0f} y={filas[-1, 2]:.0f} "
            f"th={filas[-1, 3]:.1f}")


# ═══════════════════════════════════════════════════════════════════════════════
# Paredes: dónde dicen los ToF que estaba el carro
# ═══════════════════════════════════════════════════════════════════════════════

def segmentos(mapa):
    segs = []
    for x0, y0, x1, y1 in mapa:
        segs += [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    return segs


def rayo(ox, oy, dx, dy, segs):
    """Distancia a la primera pared, su índice y dónde cae en ella (0..1); (-1, -1, 0) si no hay."""
    best, kb, ub = -1.0, -1, 0.0
    for k, ((ax, ay), (bx, by)) in enumerate(segs):
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        wx, wy = ax - ox, ay - oy
        tr = (wx * ey - wy * ex) / den
        us = (wx * dy - wy * dx) / den
        if tr > 0 and 0 <= us <= 1 and (best < 0 or tr < best):
            best, kb, ub = tr, k, us
    return best, kb, ub


def leer_tof(ruta_log):
    """Renglones T del log: (t_ms, sensor, mm, status)."""
    T = []
    with open(ruta_log, encoding="utf-8", errors="replace") as f:
        for linea in f:
            if linea.startswith("T,"):
                c = linea.strip().split(",")
                try:
                    T.append((float(c[1]), c[2], float(c[3]), int(c[4])))
                except (ValueError, IndexError):
                    continue
    return T


def sets_de(a):
    """--set clave=valor -> {clave: valor}."""
    d = {}
    for kv in getattr(a, "set", None) or []:
        if "=" in kv:
            k, v = kv.split("=", 1)
            d[k.strip()] = v.strip()
    return d


def tofs_de(par):
    """Montajes de los ToF con los parámetros del carro (# par del log) si los cambiaron."""
    t = {k: list(v) for k, v in TOFS.items()}
    for sen, clave in (("F", "tofFAdelante"), ("L", "tofLAdelante"), ("R", "tofRAdelante")):
        if clave in par:
            t[sen][0] = float(par[clave])
    for sen, clave in (("L", "tofLIzq"), ("R", "tofRIzq")):
        if clave in par:
            t[sen][1] = float(par[clave])
    return t


def pared_vs_odometria(P, T, mapa, centro, tofs=None):
    """Las mismas reglas que corregirConPared(): por cada lectura lateral útil en
    recta, e = cuánto más lejos de la pared está el carro de lo que cree (mm,
    perpendicular) y el error lateral contra la ruta que tendría si la pared tiene
    razón. Filas: (t_ms, 0=L/1=R, e, x_pared, y_pared, lat_pared)."""
    if len(P) < 2 or not T:
        return np.zeros((0, 6))
    segs = segmentos(mapa)
    rueda_max_servo = PARED_RUEDA_MAX * SERVO_POR_RUEDA_DER
    out = []
    for t, sen, mm, st in T:
        if sen not in ("L", "R") or st not in (0, 2, 7) or not 40 <= mm <= 650:
            continue
        i = np.searchsorted(P[:, 0], t)
        if i == 0 or i >= len(P):
            continue
        f = (t - P[i - 1, 0]) / max(P[i, 0] - P[i - 1, 0], 1)
        x, y, th, lat = (P[i - 1, k] + f * (P[i, k] - P[i - 1, k]) for k in (1, 2, 3, 5))
        if abs(((th + 45) % 90) - 45) > PARED_RECTA_DEG or abs(P[i, 6] - centro) > rueda_max_servo:
            continue
        adel, izq, ang = (tofs or TOFS)[sen]
        h = math.radians(th)
        ox = x + adel * math.cos(h) - izq * math.sin(h)
        oy = y + adel * math.sin(h) + izq * math.cos(h)
        dx, dy = math.cos(h + math.radians(ang)), math.sin(h + math.radians(ang))
        esperado, k, u = rayo(ox, oy, dx, dy, segs)
        if esperado < 0:
            continue
        (ax, ay), (bx, by) = segs[k]
        largo = math.hypot(bx - ax, by - ay)
        nx, ny = -(by - ay) / largo, (bx - ax) / largo
        if (ox - ax) * nx + (oy - ay) * ny < 0:
            nx, ny = -nx, -ny
        cos_inc = -(dx * nx + dy * ny)
        if cos_inc < math.cos(math.radians(PARED_INC_MAX_DEG)):
            continue
        margen = esperado * math.tan(math.radians(CONO_MEDIO_DEG)) / cos_inc + 30
        if u * largo < margen or (1 - u) * largo < margen:
            continue
        e = (mm - esperado) * cos_inc
        if abs(e) > PARED_GATE_MM:
            continue
        # n·(izquierda del carro): cuánto de e cae en el error lateral contra la ruta.
        n_izq = -nx * math.sin(h) + ny * math.cos(h)
        out.append((t, 0 if sen == "L" else 1, e, x + e * nx, y + e * ny, lat + e * n_izq))
    return np.array(out) if out else np.zeros((0, 6))


def linea_resumen(P, W, meta, T):
    """Una línea para comparar corridas: lo que dicen las paredes exteriores (ToF R
    en antihorario, L en horario) y el resto de lo importante."""
    n = {s_: sum(1 for z in T if z[1] == s_) for s_ in ("F", "L", "R")}
    partes = [f"t={P[-1, 0] / 1000:.1f}s" if len(P) else "t=?"]
    if len(P):
        partes.append(f"odo_rms={np.sqrt(np.mean(P[:, 5] ** 2)):.0f}")
    for sen, nom in ((0, "L"), (1, "R")):
        m = W[:, 1] == sen if len(W) else np.zeros(0, bool)
        if m.sum() >= 5:
            lat = W[m, 5]
            ult = lat[-min(8, len(lat)):]
            partes.append(f"{nom}_rms={np.sqrt(np.mean(lat ** 2)):.0f} {nom}_media={lat.mean():+.0f} "
                          f"{nom}_fin={ult.mean():+.0f}")
        else:
            partes.append(f"{nom}=sin_datos")
    if "fin" in meta:
        partes.append("fin=" + ",".join(f"{v:.0f}" for v in meta["fin"]))
    partes.append(f"tof=F{n['F']}/L{n['L']}/R{n['R']}")
    return "RESUMEN " + " ".join(partes)


def resumen_pared(W, P):
    """Por recta: error lateral según la odometría y según cada pared."""
    if len(W) == 0:
        return ["Paredes: no hay lecturas laterales útiles (¿sketch sin ToF, o ninguna recta?)"]
    lineas = ["Paredes (por recta): error lateral contra la ruta, mm (+ = carro a la izquierda)",
              "  recta  rumbo   odometría   según L    según R"]
    tP = P[:, 0]
    th_w = np.interp(W[:, 0], tP, P[:, 3])
    tramo = np.round(th_w / 90).astype(int)
    for k in np.unique(tramo):
        m = tramo == k
        tw = W[m, 0]
        mp = (tP >= tw.min()) & (tP <= tw.max())
        lat_odo = P[mp, 5].mean() if mp.any() else float("nan")
        celdas = []
        for sen in (0, 1):
            ms = m & (W[:, 1] == sen)
            celdas.append(f"{W[ms, 5].mean():+7.0f} ({ms.sum():2d})" if ms.sum() >= 3 else "      -     ")
        lineas.append(f"  {k:5d}  {90 * k:5d}°   {lat_odo:+7.0f}    {celdas[0]}  {celdas[1]}")
    return lineas


# ═══════════════════════════════════════════════════════════════════════════════
# Gráfica
# ═══════════════════════════════════════════════════════════════════════════════

def graficar(ruta, cerrada, salida, sim, real, titulo, png, mostrar, mapa=None, pared=None):
    import matplotlib
    if not mostrar:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(13, 7.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1], hspace=0.35, wspace=0.22)
    ax = fig.add_subplot(gs[:, 0])
    ax_lat = fig.add_subplot(gs[0, 1])
    ax_srv = fig.add_subplot(gs[1, 1], sharex=ax_lat)

    # Pista de fondo si la ruta cae dentro de ella.
    mapa = mapa or mapa_pista()
    if ruta[:, 0].min() > -50 and ruta[:, 1].min() > -50 and ruta.max() < mapa[0][2] + 50:
        for (x0, y0, x1, y1) in mapa:
            ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], color="#c9c8c2", lw=1.2, zorder=0)

    r = np.vstack([ruta, ruta[:1]]) if cerrada else ruta
    ax.plot(r[:, 0], r[:, 1], color=COLOR_RUTA, lw=6, alpha=0.35, solid_capstyle="round", label="ruta pedida")
    if len(sim):
        ax.plot(sim[:, 1], sim[:, 2], color=COLOR_SIM, lw=1.6, ls="--", label="simulación")
    if real is not None and len(real):
        ax.plot(real[:, 1], real[:, 2], color=COLOR_REAL, lw=2, label="carro (odometría)")
        ax.plot(real[-1, 1], real[-1, 2], "o", color=COLOR_REAL, ms=8, mec="white", mew=2)
    if pared is not None and len(pared):
        for sen, nombre in ((0, "L"), (1, "R")):
            m = pared[:, 1] == sen
            if m.any():
                ax.plot(pared[m, 3], pared[m, 4], ".", color=COLOR_PARED[nombre], ms=4,
                        label=f"según ToF {nombre} (pared)")
    sx, sy, sth = salida
    ax.annotate("", xy=(sx + 150 * math.cos(math.radians(sth)), sy + 150 * math.sin(math.radians(sth))),
                xytext=(sx, sy), arrowprops=dict(arrowstyle="-|>", color="#3d3d3a", lw=1.5))
    ax.plot(sx, sy, "s", color="#3d3d3a", ms=7)
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.grid(color="#ecebe6", lw=0.8)
    ax.legend(loc="best", frameon=False, fontsize=9)

    for filas, color, ls, nombre in ((sim, COLOR_SIM, "--", "simulación"),
                                     (real, COLOR_REAL, "-", "carro")):
        if filas is None or len(filas) == 0:
            continue
        t = filas[:, 0] / 1000
        ax_lat.plot(t, filas[:, 5], color=color, ls=ls, lw=1.6, label=nombre)
        ax_srv.plot(t, filas[:, 6], color=color, ls=ls, lw=1.6, label=nombre)
    if pared is not None and len(pared):
        for sen, nombre in ((0, "L"), (1, "R")):
            m = pared[:, 1] == sen
            if m.any():
                ax_lat.plot(pared[m, 0] / 1000, pared[m, 5], ".", color=COLOR_PARED[nombre], ms=5,
                            label=f"según ToF {nombre}")
    ax_lat.axhline(0, color="#3d3d3a", lw=0.8)
    ax_lat.set_ylabel("error lateral (mm)\n+ = carro a la izquierda", fontsize=9)
    ax_lat.set_title("Error lateral contra la ruta", fontsize=10, loc="left")
    ax_srv.axhline(CENTRO_SERVO, color="#3d3d3a", lw=0.8)
    ax_srv.set_ylabel("servo (°)\n> centro = izquierda", fontsize=9)
    ax_srv.set_xlabel("tiempo (s)")
    ax_srv.set_title("Servo", fontsize=10, loc="left")
    for a_ in (ax_lat, ax_srv):
        a_.grid(color="#ecebe6", lw=0.8)
        a_.legend(loc="best", frameon=False, fontsize=9)
        for lado in ("top", "right"):
            a_.spines[lado].set_visible(False)

    fig.suptitle(titulo, fontsize=11, x=0.06, ha="left")
    fig.savefig(png, dpi=110, bbox_inches="tight")
    print(f"Gráfica: {png}")
    if mostrar:
        plt.show()
    plt.close(fig)


# ═══════════════════════════════════════════════════════════════════════════════
# Log
# ═══════════════════════════════════════════════════════════════════════════════

def filas_p(lineas):
    """P,t,x,y,th,i,lat,servo,pwm -> array (N, 8)."""
    filas = []
    for linea in lineas:
        i = linea.find("P,")
        if i < 0 or (i > 0 and linea[i - 1] != " "):
            continue
        c = linea[i:].split(",")
        if len(c) < 9:
            continue
        try:
            filas.append(tuple(float(v) for v in c[1:9]))
        except ValueError:
            continue
    return np.array(filas) if filas else np.zeros((0, 8))


def leer_log(ruta_log):
    """Lo que el script escribió al principio (# PC ..., R,x,y) y las P del carro."""
    pts, meta, lineas = [], {}, []
    with open(ruta_log, encoding="utf-8", errors="replace") as f:
        for linea in f:
            linea = linea.rstrip("\r\n")
            lineas.append(linea)
            if linea.startswith("R,"):
                c = linea.split(",")
                pts.append((float(c[1]), float(c[2])))
            elif linea.startswith("# PC "):
                for kv in linea[5:].split():
                    if "=" in kv:
                        k, v = kv.split("=", 1)
                        meta[k] = v
            elif linea.startswith("# SeguirRuta") and "centro=" in linea:
                meta["centro_carro"] = linea.split("centro=")[1].split()[0]
                if "kd=" in linea:
                    meta["kd_carro"] = linea.split("kd=")[1].split()[0]
                if "pared=" in linea:
                    meta["pared_carro"] = linea.split("pared=")[1].split()[0]
            elif linea.startswith("# par ") or linea.startswith("# set # par "):
                k, _, v = linea.split("# par ", 1)[1].partition("=")
                meta.setdefault("par", {})[k.strip()] = v.strip()
            elif linea.startswith("# fin x="):
                c = linea[6:].split()
                meta["fin"] = tuple(float(kv.split("=")[1]) for kv in c[:3])   # x, y, th ya parado
    return np.array(pts), meta, filas_p(lineas)


# ═══════════════════════════════════════════════════════════════════════════════
# Conexión con el carro
# ═══════════════════════════════════════════════════════════════════════════════

class Carro:
    """Lee el puerto en un hilo: guarda todo en el archivo y en `lineas`."""

    def __init__(self, puerto, archivo):
        import serial
        print(f"Conectando a {puerto}... (si el carro está apagado, préndelo; Ctrl+C para salir)")
        intentos = 0
        while True:
            try:
                self.ser = serial.Serial(puerto, 115200, timeout=0.2)
                break
            except serial.SerialException as e:
                intentos += 1
                if intentos == 1 or intentos % 10 == 0:
                    print(f"  esperando al ESP32... ({e.__class__.__name__}; ¿otra app ya está conectada?)")
                time.sleep(0.5)
        self.f = archivo
        self.lineas = []
        self.vivo = True
        self.n_p = 0
        threading.Thread(target=self._leer, daemon=True).start()

    def _leer(self):
        listo_visto = False
        while self.vivo:
            try:
                linea = self.ser.readline().decode("utf-8", errors="replace").rstrip("\r\n")
            except Exception:
                break
            if not linea:
                continue
            self.f.write(linea + "\n")
            self.lineas.append(linea)
            if linea.startswith("T,"):
                continue   # ToF: solo al archivo (son ~60 por segundo)
            if linea.startswith("P,"):
                self.n_p += 1
                if self.n_p % 25 == 0:
                    c = linea.split(",")
                    print(f"\r  t={int(c[1]) / 1000:5.1f} s  x={c[2]:>5} y={c[3]:>5} th={c[4]:>7}  "
                          f"lateral={c[6]:>4} mm  servo={c[7]:>3} pwm={c[8]:>3}   ", end="", flush=True)
            elif linea.startswith("# listo"):
                if not listo_visto:
                    print(linea)
                listo_visto = True
            else:
                print(("\n" if self.n_p else "") + linea)

    def mandar(self, texto):
        self.ser.write(texto.encode())

    def esperar(self, *prefijos, desde=0, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for linea in self.lineas[desde:]:
                for p in prefijos:
                    if linea.startswith(p):
                        return linea
            time.sleep(0.05)
        return None

    def cerrar(self):
        self.vivo = False
        time.sleep(0.3)
        self.ser.close()


def encabezado_log(a, ruta, paso, cerrada, salida, titulo, tipo):
    """Lo que va al principio del log para poder regraficarlo con `ver`."""
    txt = (f"# PC tipo={tipo} cerrada={int(cerrada)} paso={paso:.3f} vueltas={a.vueltas} "
           f"salida={salida[0]:.0f},{salida[1]:.0f},{salida[2]:.1f} pwm={a.pwm} pwm_curva={a.pwm_curva} "
           f"lookahead={a.lookahead:.0f} centro={CENTRO_SERVO} kd={getattr(a, 'kd', None) or 0} "
           f"pared={getattr(a, 'pared', 0)} set={';'.join(getattr(a, 'set', None) or [])} enc_curva={getattr(a, 'enc_curva', 0.0)} "
           f"bno_escala={getattr(a, 'bno_escala', 1.0)} anticipo={getattr(a, 'anticipo', 0.0)} mapa={','.join(f'{v:.0f}' for r in mapa_de(a) for v in r)}\n"
           f"# PC titulo {titulo}\n")
    return txt + "".join(f"R,{px:.1f},{py:.1f}\n" for px, py in ruta)


def comandos_ruta(ruta, paso, cerrada):
    """RTB, RTP de 10 puntos por línea y RTE (protocolo de PurePursuit.ino)."""
    lineas = [f"RTB,{len(ruta)},{paso:.3f},{int(cerrada)}\n"]
    for i in range(0, len(ruta), 10):
        pares = ",".join(f"{int(round(px))},{int(round(py))}" for px, py in ruta[i:i + 10])
        lineas.append(f"RTP,{i},{pares}\n")
    lineas.append(f"RTE,{len(ruta)}\n")
    return lineas


def mapa_de(a):
    return mapa_pista(getattr(a, "pista", PISTA_MM), getattr(a, "interior", None))


def comando_mapa(a):
    return "RTM," + ",".join(f"{v:.0f}" for r in mapa_de(a) for v in r) + "\n"


def comando_salida(a, salida):
    # Los campos son posicionales: centro, kd y pared van siempre para que el ESP32
    # use los de aquí (con un sketch viejo los que no conoce se ignoran).
    kd = getattr(a, "kd", None)
    kd = 0.04 if kd is None else kd
    return (f"RTS,{salida[0]:.0f},{salida[1]:.0f},{salida[2]:.1f},{a.pwm},{a.pwm_curva},"
            f"{a.vueltas},{a.lookahead:.0f},{CENTRO_SERVO},{kd:.3f},{getattr(a, 'pared', 0)},"
            f"{getattr(a, 'enc_curva', 0.0):.3f},{getattr(a, 'bno_escala', 1.0):.4f},"
            f"{getattr(a, 'anticipo', 0.0):.3f}\n")


def subir_y_correr(a, ruta, paso, cerrada, salida, titulo, tipo):
    if len(ruta) > RUTA_MAX:
        sys.exit(f"La ruta tiene {len(ruta)} puntos y el ESP32 guarda {RUTA_MAX}: sube --paso")
    puerto = a.puerto
    if not puerto:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from grabar_bt import buscar_puerto
        puerto = buscar_puerto()
        if not puerto:
            sys.exit("No encontré el ESP32 entre los puertos Bluetooth. ¿Está emparejado en Windows?\n"
                     "Pasa uno a mano con --puerto (COM9 = cable USB), o prueba solo la simulación con --sim.")

    os.makedirs(os.path.join(RAIZ, "logs_tof"), exist_ok=True)
    nombre = os.path.join(RAIZ, "logs_tof", datetime.datetime.now().strftime(f"ruta_{tipo}_%Y%m%d_%H%M%S.txt"))
    f = open(nombre, "w", encoding="utf-8")
    f.write(encabezado_log(a, ruta, paso, cerrada, salida, titulo, tipo))
    f.flush()

    carro = Carro(puerto, f)
    try:
        if not carro.esperar("# listo", timeout=15):
            print("El ESP32 no dijo '# listo' en 15 s: ¿tiene cargado SeguirRuta.ino? ¿BNO y encoder OK?")
            return nombre

        desde = len(carro.lineas)
        for linea in comandos_ruta(ruta, paso, cerrada):
            carro.mandar(linea)
            time.sleep(0.02)   # que no se llene el buffer del Bluetooth
        r = carro.esperar("# ruta OK", "# ruta RECHAZADA", "# ignorado", desde=desde)
        if not r or not r.startswith("# ruta OK"):
            print("No se cargó la ruta" + ("" if r else " (sin respuesta)") + ". Vuelve a intentar.")
            return nombre
        desde = len(carro.lineas)
        carro.mandar(comando_mapa(a))
        if not carro.esperar("# mapa", desde=desde, timeout=3):
            print("  (el ESP32 no confirmó el mapa: ¿sketch viejo? Sigo, pero sin paredes)")
        desde = len(carro.lineas)
        carro.mandar(comando_salida(a, salida))
        if not carro.esperar("# salida", desde=desde):
            print("El ESP32 no confirmó la salida (RTS).")
            return nombre
        for clave, valor in sets_de(a).items():
            desde = len(carro.lineas)
            carro.mandar(f"SET,{clave},{valor}\n")
            r = carro.esperar("# set", "# SET:", desde=desde, timeout=3)
            if not r or r.startswith("# SET:"):
                print(f"El ESP32 no aceptó {clave}={valor}" + ("" if r else " (¿sketch viejo, sin SET?)"))
                return nombre
        carro.mandar("PAR\n")      # todos los parámetros quedan en el log
        time.sleep(0.5)

        print("\n" + "=" * 70)
        print(f"Pon el centro del EJE TRASERO en x={salida[0]:.0f} mm, y={salida[1]:.0f} mm, "
              f"viendo a {salida[2]:.0f}°")
        # Dónde quedan las paredes x = 0 y y = 0 según hacia dónde mira el carro.
        lados = {0: ("atrás", "la derecha"), 90: ("la izquierda", "atrás"),
                 180: ("adelante", "la izquierda"), 270: ("la derecha", "adelante")}
        lx, ly = lados[int(round(salida[2] / 90.0)) % 4 * 90]
        print(f"  = a {salida[0]:.0f} mm de la pared/marca de {lx} (x = 0) y a {salida[1]:.0f} mm "
              f"de la pared/marca de {ly} (y = 0)")
        print("  Alinéalo bien: un error de rumbo al arrancar NO lo ve el carro (5° = ~17 cm de error en 2 m).")
        print("=" * 70)
        if not a.auto:
            input("Enter para arrancar (luego: x + Enter lo para, Ctrl+C también)... ")

        def teclado():
            for linea in sys.stdin:
                if linea.strip().lower().startswith("x"):
                    carro.mandar("x")

        desde = len(carro.lineas)
        carro.mandar("g")
        r = carro.esperar("E,0,ARRANCA", "# no hay ruta", "# la salida", desde=desde, timeout=3)
        if not r or not r.startswith("E,0,ARRANCA"):
            print("El carro no arrancó.")
            return nombre
        threading.Thread(target=teclado, daemon=True).start()
        try:
            while not any(l.startswith("E,") and ",FIN" in l for l in carro.lineas[desde:]):
                time.sleep(0.1)
        except KeyboardInterrupt:
            carro.mandar("x")
            print("\nParado.")
        carro.esperar("# fin", desde=desde, timeout=2)
    finally:
        carro.cerrar()
        f.close()
    print(f"\nLog: {nombre}")
    return nombre


# ═══════════════════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    global CENTRO_SERVO
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="tipo", required=True)

    def comunes(p, x0=500.0, y0=500.0, con_salida=True):
        p.add_argument("--sim", action="store_true", help="solo simula y grafica, no se conecta")
        p.add_argument("--puerto", help="COM del ESP32 (sin esto busca el Bluetooth; COM9 = USB)")
        p.add_argument("--pwm", type=int, default=150, help="PWM en recta (180 ≈ 540 mm/s)")
        p.add_argument("--pwm-curva", type=int, default=120, help="PWM con la rueda muy cerrada")
        p.add_argument("--lookahead", type=float, default=275.0,
                       help="mm (2026-10-23 a 255: 150 bailaba, 200 oscilaba, 250-300 suave; 275 el elegido)")
        p.add_argument("--centro", type=int, help=f"servo con el que va derecho (sin esto: el del sketch, {CENTRO_SERVO})")
        p.add_argument("--kd", type=float, default=0.04,
                       help="amortiguamiento con el giro del BNO, ° de rueda por °/s (0 = apagado; 0.08 ya oscila a 255)")
        p.add_argument("--paso", type=float, default=20.0, help="mm entre puntos de la ruta")
        p.add_argument("--pista", type=float, default=PISTA_MM, help="lado de la pista, pared a pared por dentro (mm)")
        p.add_argument("--enc-curva", type=float, default=0.0,
                       help="corrección del encoder con la rueda girada (0 = apagada; 2026-10-23: 0.08-0.15 hacía "
                            "que el carro recorriera 30-50 cm DE MÁS en 3 vueltas, no usar)")
        p.add_argument("--bno-escala", type=float, default=1.0,
                       help="grados reales por grado del BNO (CalibFox a mano: 0.9998; 0.996 hacía crecer el error hacia adentro)")
        p.add_argument("--set", action="append", metavar="CLAVE=VALOR",
                       help="cambia un parámetro del ESP32 por nombre (se repite; la lista: PAR)")
        p.add_argument("--anticipo", type=float, default=0.0,
                       help="s que el pure pursuit se adelanta al servo (0 = apagado; en simulación 0.04 ayudó)")
        p.add_argument("--interior", type=float,
                       help="lado del cuadro interior, centrado (mm; sin esto: un tercio de --pista, todo a escala)")
        p.add_argument("--pared", type=int, default=0,
                       help="corrección con paredes: 1 posición, 2 rumbo, 4 avance, 8 también el cuadro interior "
                            "(se suman; 0 = solo registrar; sin el 8 solo las paredes exteriores)")
        p.add_argument("--auto", action="store_true", help="arranca sin esperar Enter")
        p.add_argument("--no-mostrar", action="store_true", help="solo guarda el PNG")
        if con_salida:   # vuelta y archivo salen del primer punto de su ruta
            p.add_argument("--x0", type=float, default=x0, help="salida: x del eje trasero (mm)")
            p.add_argument("--y0", type=float, default=y0, help="salida: y del eje trasero (mm)")

    p = sub.add_parser("recta", help="línea recta")
    comunes(p)
    p.add_argument("--largo", type=float, default=2000.0)
    p.set_defaults(gen=ruta_recta, vueltas=1)

    p = sub.add_parser("carril", help="recta con cambio de carril")
    comunes(p)
    p.add_argument("--desp", type=float, default=300.0, help="mm a un lado (+ = izquierda)")
    p.add_argument("--antes", type=float, default=500.0, help="recta antes del cambio (mm)")
    p.add_argument("--cambio", type=float, default=800.0, help="largo del cambio (mm)")
    p.add_argument("--despues", type=float, default=700.0, help="recta después (mm)")
    p.set_defaults(gen=ruta_carril, vueltas=1)

    p = sub.add_parser("s", help="slalom de arcos")
    comunes(p, y0=350.0)   # el slalom sube ~290 mm: queda centrado en el pasillo de abajo
    p.add_argument("--radio", type=float, default=400.0)
    p.add_argument("--angulo", type=float, default=50.0, help="grados del primer arco")
    p.add_argument("--curvas", type=int, default=2, help="arcos completos en medio")
    p.add_argument("--antes", type=float, default=300.0)
    p.add_argument("--despues", type=float, default=300.0)
    p.set_defaults(gen=ruta_s, vueltas=1)

    p = sub.add_parser("vuelta", help="vuelta a la pista (cerrada)")
    comunes(p, con_salida=False)
    p.add_argument("--sentido", choices=["izq", "der"], default="izq", help="izq = antihorario")
    p.add_argument("--radio", type=float, default=300.0, help="radio de las esquinas (mm)")
    p.add_argument("--carril", type=float, help="distancia a la pared exterior (mm; sin esto: medio pasillo, a escala)")
    p.add_argument("--inicio", type=float, help="salida a esto de la esquina (default: media recta)")
    p.add_argument("--vueltas", type=int, default=1)
    p.set_defaults(gen=ruta_vuelta)

    p = sub.add_parser("archivo", help="ruta de un CSV x,y en mm")
    comunes(p, con_salida=False)
    p.add_argument("csv")
    p.add_argument("--cerrada", action="store_true", help="la ruta es una vuelta")
    p.add_argument("--vueltas", type=int, default=1)
    p.set_defaults(gen=ruta_archivo)

    p = sub.add_parser("ver", help="vuelve a graficar un log de ruta_bt")
    p.add_argument("log")
    p.add_argument("--no-mostrar", action="store_true")
    p.add_argument("--sin-grafica", action="store_true", help="solo las tablas y el RESUMEN")
    a = ap.parse_args()

    if a.tipo == "ver":
        ruta, meta, real = leer_log(a.log)
        if len(ruta) == 0:
            sys.exit("Ese log no trae la ruta (renglones R,x,y): ¿es de ruta_bt.py?")
        cerrada = meta.get("cerrada") == "1"
        paso = float(meta.get("paso", 20))
        sx, sy, sth = (float(v) for v in meta.get("salida", f"{ruta[0][0]},{ruta[0][1]},0").split(","))
        a.vueltas = int(meta.get("vueltas", 1))
        a.pwm, a.pwm_curva = int(meta.get("pwm", 150)), int(meta.get("pwm_curva", 120))
        a.lookahead = float(meta.get("lookahead", 200))
        a.kd = float(meta.get("kd_carro", meta.get("kd", 0)))
        CENTRO_SERVO = int(meta.get("centro_carro", meta.get("centro", CENTRO_SERVO)))
        sim, motivo = simular(ruta, paso, cerrada, (sx, sy, sth), a)
        print(resumen(f"simulación ({motivo})", sim))
        print(resumen("carro", real))
        titulo = next((l[12:].strip() for l in open(a.log, encoding="utf-8", errors="replace")
                       if l.startswith("# PC titulo")), meta.get("tipo", ""))
        mapa = mapa_pista()
        if "mapa" in meta:
            v = [float(z) for z in meta["mapa"].split(",")]
            mapa = (tuple(v[:4]), tuple(v[4:8]))
        T = leer_tof(a.log)
        W = pared_vs_odometria(real, T, mapa, CENTRO_SERVO, tofs_de(meta.get("par", {})))
        for linea in resumen_pared(W, real):
            print(linea)
        print(linea_resumen(real, W, meta, T))
        if not a.sin_grafica:
            graficar(ruta, cerrada, (sx, sy, sth), sim, real, titulo,
                     os.path.splitext(a.log)[0] + ".png", not a.no_mostrar, mapa=mapa, pared=W)
        return

    if a.centro is not None:
        CENTRO_SERVO = a.centro
    pts, cerrada, titulo = a.gen(a)
    ruta, paso = remuestrear(pts, a.paso, cerrada)
    # Salida: el primer punto, viendo hacia el segundo.
    salida = (ruta[0, 0], ruta[0, 1], math.degrees(math.atan2(ruta[1, 1] - ruta[0, 1], ruta[1, 0] - ruta[0, 0])))
    titulo = (f"{titulo} | pwm {a.pwm}/{a.pwm_curva} lookahead {a.lookahead:.0f} centro {CENTRO_SERVO}"
              + (f" kd {a.kd:g}" if a.kd else "") + (f" pared {a.pared}" if a.pared else "")
              + (f" anticipo {a.anticipo:g}" if a.anticipo else ""))

    largo = np.sum(np.hypot(*np.diff(ruta, axis=0).T)) + (np.hypot(*(ruta[0] - ruta[-1])) if cerrada else 0)
    r = radios(ruta, cerrada)
    print(f"Ruta: {titulo}")
    print(f"  {len(ruta)} puntos cada {paso:.1f} mm, {largo:.0f} mm, {'cerrada' if cerrada else 'abierta'}, "
          f"radio mínimo {np.min(r):.0f} mm")
    if np.min(r) < RADIO_MIN_MM:
        print(f"  !! Hay curvas más cerradas que el radio mínimo del carro ({RADIO_MIN_MM:.0f} mm): "
              "no las va a poder seguir.")
    elif np.min(r) < RADIO_AVISO_MM:
        print(f"  ! Curvas de menos de {RADIO_AVISO_MM:.0f} mm: va a ir a tope, sin margen.")

    sim, motivo = simular(ruta, paso, cerrada, salida, a)
    print("  " + resumen(f"simulación ({motivo}, ~{vel_por_pwm(a.pwm):.0f} mm/s)", sim))

    if a.sim:
        os.makedirs(os.path.join(RAIZ, "logs_tof"), exist_ok=True)
        png = os.path.join(RAIZ, "logs_tof", f"sim_{a.tipo}.png")
        graficar(ruta, cerrada, salida, sim, None, titulo + " (solo simulación)", png, not a.no_mostrar,
                 mapa=mapa_de(a))
        return

    nombre = subir_y_correr(a, ruta, paso, cerrada, salida, titulo, a.tipo)
    _, meta, real = leer_log(nombre)
    print(resumen("carro", real))
    if "fin" in meta:
        fx, fy, fth = meta["fin"]
        print(f"Ya parado (con lo que rodó al frenar) el carro cree estar en x={fx:.0f} y={fy:.0f} th={fth:.1f}.")
        print("Mide dónde quedó DE VERDAD el eje trasero: la diferencia es error de odometría (encoder/BNO), "
              "no del seguidor.")
    T = leer_tof(nombre)
    W = pared_vs_odometria(real, T, mapa_de(a), CENTRO_SERVO, tofs_de(meta.get("par", {})))
    for linea in resumen_pared(W, real):
        print(linea)
    print(linea_resumen(real, W, meta, T))
    graficar(ruta, cerrada, salida, sim, real, titulo, os.path.splitext(nombre)[0] + ".png", not a.no_mostrar,
             mapa=mapa_de(a), pared=W)


if __name__ == "__main__":
    main()
