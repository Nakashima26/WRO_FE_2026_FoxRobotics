"""Replay offline de los votos del mapa digital sobre el .avi de una corrida real.

Uso (desde src/RASPI/cam):  python -m pure_pursuit.twin.tools.replay_votos run.avi run_tele.jsonl [t0 t1]
Corre Vision + BEV sobre el panel izquierdo (cámara) de cada cuadro, pone la pose de la telemetría
(map.pose) en un DigitalMap y vuelca por cada pie rojo/verde qué rama del voto lo descarta o lo vota.
"""
import json
import math
import sys

import os

import cv2
import numpy as np

_isdir = os.path.isdir   # los umbrales de la Pi 5 (vision.PI5_CAMARA) dependen de este directorio
os.path.isdir = lambda p: True if p == "/usr/share/libcamera/ipa/rpi/pisp" else _isdir(p)
from vision import Vision
from pure_pursuit import config as C
from pure_pursuit.bev import BEVTransformer
from pure_pursuit.centerline import map_obstacle_to_bev, seen_above_obstacles
from pure_pursuit.corner_lines import OrangeLineTracker
from pure_pursuit.digital_map import DigitalMap


def main(avi, tele, t_lo=0.0, t_hi=99.0, flags=None, quiet=False):
    for k, v in (flags or {}).items():
        setattr(C, k, v)
    rows = [json.loads(l) for l in open(tele, encoding="utf-8").read().splitlines()]
    t0 = rows[0]["t"]
    poses = [(r["t"] - t0, r["s"]["map"]["pose"], r["s"].get("tc", 0), r["s"].get("ack")) for r in rows[1:]
             if r.get("s") and r["s"].get("map") and r["s"]["map"].get("pose")]
    dur = rows[-1]["t"] - t0
    cap = cv2.VideoCapture(avi)
    frames = []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        frames.append(f[:, :640])
    n = len(frames)
    vis = Vision(open_cam=False)
    vis.use_lut = bool(getattr(C, "VISION_LUT", True))
    vis.GREEN_V_MIN_MED = float(getattr(C, "VISION_GREEN_V_MIN_MED", 0))
    vis.min_contour_area = 1000.0
    bev = BEVTransformer(None)
    trk = OrangeLineTracker()
    dm = DigitalMap()
    dm.set_direction("CCW")
    dm.in_stall = False
    dm.vote_log = []
    first = {}
    out = []
    ti = 0
    for i, fr in enumerate(frames):
        tv = 0.13 + (dur - 0.13) * i / max(1, n - 1)
        while ti + 1 < len(poses) and poses[ti + 1][0] <= tv:
            ti += 1
        tp, pose, tc, ack = poses[ti]
        _, positions = vis.detect_on(fr)
        bevf = bev.warp(fr)
        oi = trk.update(bevf, bev_hsv=cv2.cvtColor(bevf, cv2.COLOR_BGR2HSV), ds_px=0.0)
        feet = []
        for cn in ("Red", "Green"):
            for x, y, w, h in positions.get(cn, []):
                r = map_obstacle_to_bev(bev, x, y, w, h)
                if r is not None:
                    feet.append((r[0], r[1], cn))
        feet.extend(seen_above_obstacles(bev, positions))
        if not (t_lo <= tv <= t_hi):
            continue
        dm.pose_xy = (pose[0], pose[1])
        dm._view_heading = dm.heading = pose[2]
        dm._tc = tc
        dm.section = dm.section_of(tc)
        dm._note_orange(oi)
        dm.vote_log.clear()
        conf0 = dict(dm.confirmed)
        dm._vote(feet)
        out.append((round(tv, 2), [(round(a), round(b), c[0]) for a, b, c in feet], oi.get("near_y"), dm._orange_y,
                    list(dm.vote_log), dict(dm.confirmed)))
        if not quiet and (feet or dm.vote_log):
            print(out[-1], flush=True)
        if dm.confirmed != conf0:
            first.setdefault("conf", tv)
    print('loose_extra', dm.loose_extra(), 'confirmed', dm.confirmed)
    return out, dm


if __name__ == "__main__":
    a = sys.argv[1:]
    main(a[0], a[1], *(float(x) for x in a[2:4]), flags=json.loads(a[4]) if len(a) > 4 else None, quiet=len(a) > 5)
