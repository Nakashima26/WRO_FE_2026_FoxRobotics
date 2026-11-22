"""Labelled versions of the car photos in v-photos/ -> v-photos/labels/<view>_labels.jpg

python docs/img/make_labels.py

Each label: (text, target x, target y, side, label y) in the original photo's pixels.
The photo gets a dark margin on both sides; labels sit in the margins with a leader
line and a dot on the part.
"""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "v-photos/labels"
MARGIN = 520
YELLOW = (255, 214, 0)
FONT = ImageFont.truetype("C:/Windows/Fonts/bahnschrift.ttf", 38) if Path("C:/Windows/Fonts/bahnschrift.ttf").exists() \
    else ImageFont.load_default()

LABELS = {
    "front": [
        ("Camera ribbon (22 to 15 pin)", 690, 420, "R", 420),
        ("Battery lead, 16 AWG", 420, 330, "L", 330),
        ("Raspberry Pi 5 +\nActive Cooler", 420, 640, "L", 620),
        ("Camera IMX219 NoIR,\nwide lens, tilted 34°", 610, 930, "R", 880),
        ("Front HC-SR04\n(ToF cross-check)", 620, 1160, "R", 1160),
        ("Front wheels,\nAckermann steering", 300, 1080, "L", 1080),
    ],
    "rear": [
        ("Camera ribbon", 560, 300, "L", 260),
        ("Battery T-plug", 700, 630, "R", 560),
        ("Battery input (J1)", 870, 750, "R", 700),
        ("Power switch", 900, 840, "R", 840),
        ("ESP32 DevKitC", 380, 800, "L", 760),
        ("BNO085 IMU", 700, 810, "R", 980),
        ("Start button", 290, 960, "L", 940),
        ("READY LED", 440, 950, "L", 1080),
        ("Rear ToF VL53L1X", 570, 1050, "R", 1110),
        ("Logic board (top)\nPower board (below)", 200, 1030, "L", 1230),
    ],
    "top": [
        ("Camera (back of the board)", 600, 330, "R", 330),
        ("Raspberry Pi 5 +\nActive Cooler", 620, 690, "R", 640),
        ("Pi USB / Ethernet", 360, 700, "L", 700),
        ("Battery T-plug", 720, 870, "R", 860),
        ("Battery input (J1)", 930, 1000, "R", 990),
        ("Power switch", 950, 1110, "R", 1110),
        ("BNO085 IMU", 720, 1100, "R", 1240),
        ("ESP32 DevKitC", 330, 1100, "L", 1060),
        ("Start button", 370, 1230, "L", 1250),
    ],
    "left": [
        ("Camera on its\ntilted mount", 170, 650, "L", 560),
        ("Raspberry Pi 5", 530, 770, "L", 720),
        ("ESP32 DevKitC", 870, 770, "R", 760),
        ("Stacked PCBs\n(logic over power)", 930, 900, "R", 920),
        ("Left ToF VL53L1X", 300, 830, "L", 880),
        ("Left HC-SR04", 630, 1060, "R", 1180),
        ("Front wheel", 300, 1070, "L", 1100),
        ("Rear wheel", 960, 1070, "R", 1050),
    ],
    "right": [
        ("Camera", 1000, 640, "R", 560),
        ("Raspberry Pi 5 heatsink", 650, 750, "R", 400),
        ("Power switch", 240, 760, "L", 700),
        ("Battery input (J1)", 370, 760, "L", 820),
        ("Stacked PCBs\n(logic over power)", 300, 900, "L", 960),
        ("Right ToF VL53L1X", 920, 820, "R", 800),
        ("Right HC-SR04", 580, 1050, "R", 1180),
        ("Rear wheel", 250, 1056, "L", 1120),
        ("Front wheel", 912, 1056, "R", 1040),
    ],
}


def label_view(name, items):
    im = Image.open(REPO / f"v-photos/{name}.jpeg").convert("RGB")
    w, h = im.size
    canvas = Image.new("RGB", (w + 2 * MARGIN, h), (18, 18, 18))
    canvas.paste(im, (MARGIN, 0))
    d = ImageDraw.Draw(canvas)
    for text, tx, ty, side, ly in items:
        px, py = tx + MARGIN, ty
        box = d.multiline_textbbox((0, 0), text, font=FONT, spacing=6)
        bw, bh = box[2] - box[0], box[3] - box[1]
        if side == "L":
            lx = 24
            anchor_x = lx + bw + 14
        else:
            lx = w + 2 * MARGIN - 24 - bw
            anchor_x = lx - 14
        top = ly - bh // 2
        d.line([(anchor_x, ly), (px, py)], fill=YELLOW, width=4)
        d.ellipse([px - 11, py - 11, px + 11, py + 11], fill=YELLOW, outline=(0, 0, 0), width=3)
        d.multiline_text((lx, top), text, font=FONT, fill=(245, 245, 245), spacing=6,
                         align="right" if side == "L" else "left")
    canvas.thumbnail((1400, 1400), Image.LANCZOS)
    OUT.mkdir(exist_ok=True)
    out = OUT / f"{name}_labels.jpg"
    canvas.save(out, quality=88, optimize=True)
    print(out)


if __name__ == "__main__":
    for name, items in LABELS.items():
        label_view(name, items)
