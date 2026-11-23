"""Portadas 1920x1080 para los videos v2 (logo, render isométrico, bandera, tipo de prueba).

python video/portadas/portada.py   -> portada_open.png, portada_obstacle.png, portada_surprise.png
"""
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

AQUI = Path(__file__).resolve().parent
REPO = AQUI.parents[1]
W, H = 1920, 1080
AMARILLO = (255, 230, 0)
BLANCO = (245, 245, 245)
GRIS = (150, 150, 150)
FUENTES = Path("C:/Windows/Fonts")


def fuente(nombre, tam):
    return ImageFont.truetype(str(FUENTES / nombre), tam)


def recorta_render(ruta):
    """Quita el fondo blanco del render CAD: flood-fill desde los bordes sobre gris claro
    (fondo + sombra), luego la sombra queda como alfa suave."""
    bgr = cv2.imread(str(ruta))
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # fondo + sombra: relleno flotante desde una esquina (la sombra es un degradado suave;
    # las líneas negras del render lo frenan en el contorno del carro)
    mascara = np.zeros((h + 2, w + 2), np.uint8)
    flags = 4 | cv2.FLOODFILL_MASK_ONLY | (255 << 8)
    for semilla in [(0, 0), (w - 1, 0), (0, h - 1), (w - 1, h - 1)]:
        cv2.floodFill(bgr.copy(), mascara, semilla, 0, (4, 4, 4), (4, 4, 4), flags)
    fondo = mascara[1:-1, 1:-1] > 0
    # huecos blancos puros encerrados (ventana del soporte de cámara)
    blanco = (bgr.min(axis=2) >= 253).astype(np.uint8)
    n, et, st, _ = cv2.connectedComponentsWithStats(blanco, connectivity=4)
    for i in range(1, n):
        if st[i, cv2.CC_STAT_AREA] > 300:
            fondo |= et == i
    v = hsv[..., 2].astype(np.float32) / 255.0
    alfa = np.where(fondo, np.clip((1.0 - v) * 1.3, 0, 0.75), 1.0)  # sombra suave
    alfa = cv2.GaussianBlur(alfa.astype(np.float32), (3, 3), 0)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    rgb[fondo] = 0  # la sombra se ve negra translúcida
    img = Image.fromarray(np.dstack([rgb, (alfa * 255).astype(np.uint8)]), "RGBA")
    return img.crop(img.getbbox())


def fondo():
    y = np.linspace(0, 1, H)[:, None]
    x = np.linspace(0, 1, W)[None, :]
    base = 14 + 16 * (1 - y) * (1 - 0.5 * x)
    img = Image.fromarray(np.repeat(base[..., None], 3, 2).astype(np.uint8), "RGB")
    d = ImageDraw.Draw(img, "RGBA")
    for gx in range(0, W, 60):  # rejilla tenue tipo pista
        d.line([(gx, 0), (gx, H)], fill=(255, 255, 255, 7))
    for gy in range(0, H, 60):
        d.line([(0, gy), (W, gy)], fill=(255, 255, 255, 7))
    # panel amarillo inclinado detrás del carro
    d.polygon([(1080, 0), (W, 0), (W, H), (820, H)], fill=AMARILLO + (255,))
    d.polygon([(1040, 0), (1065, 0), (805, H), (780, H)], fill=AMARILLO + (255,))
    d.polygon([(1000, 0), (1012, 0), (752, H), (740, H)], fill=AMARILLO + (140,))
    return img


def portada(titulo, subs, salida):
    img = fondo()
    d = ImageDraw.Draw(img)

    logo = Image.open(AQUI / "logo_foxrobotics.png").convert("RGBA")
    logo = logo.crop(logo.getbbox())
    lw = 600
    logo = logo.resize((lw, round(logo.height * lw / logo.width)), Image.LANCZOS)
    img.paste(logo, (90, 70), logo)

    y = 70 + logo.height + 60
    d.text((96, y), "WRO 2026  ·  FUTURE ENGINEERS", font=fuente("bahnschrift.ttf", 44), fill=GRIS)
    y += 70
    ft = fuente("AGENCYB.TTF", 150)
    for i, linea in enumerate(titulo):
        d.text((90, y), linea, font=ft, fill=AMARILLO if i == 0 else BLANCO)
        y += 140
    y += 30
    d.rectangle([96, y, 96 + 120, y + 8], fill=AMARILLO)
    for j, sub in enumerate(subs):
        d.text((96, y + 30 + j * 56), sub, font=fuente("bahnschrift.ttf", 44), fill=BLANCO)

    # bandera + equipo, abajo a la izquierda
    bandera = Image.open(AQUI / "bandera_mexico.png").convert("RGB").resize((126, 72), Image.LANCZOS)
    by = H - 72 - 70
    d.rectangle([94, by - 2, 96 + 126 + 1, by + 72 + 1], fill=(60, 60, 60))
    img.paste(bandera, (96, by))
    d.text((250, by - 4), "MÉXICO", font=fuente("bahnschrift.ttf", 40), fill=BLANCO)
    d.text((250, by + 42), "Mexicali, Baja California", font=fuente("bahnschrift.ttf", 30), fill=GRIS)

    # carro
    # foto real (v-photos/isometric.jpeg) sin fondo: rembg u2net + reflejo del vidrio quitado a mano
    carro = Image.open(AQUI / "carro_recorte.png").convert("RGBA")
    ch = 900
    carro = carro.resize((round(carro.width * ch / carro.height), ch), Image.LANCZOS)
    cw = carro.width
    cx, cy = 1040, (H - ch) // 2 + 20
    sombra = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    ImageDraw.Draw(sombra).ellipse([cx - 10, cy + ch - 140, cx + cw - 10, cy + ch + 10],
                                   fill=(0, 0, 0, 110))
    img = Image.alpha_composite(img.convert("RGBA"), sombra.filter(ImageFilter.GaussianBlur(30)))
    img.alpha_composite(carro, (cx, cy))
    ImageDraw.Draw(img).text((W - 70, H - 60), "Car v2", font=fuente("bahnschrift.ttf", 32),
                             fill=(20, 20, 20), anchor="rs")

    # logo CETYS (negro sobre el panel amarillo), arriba a la derecha
    cetys = Image.open(AQUI / "logo_cetys.png").convert("RGBA")
    lh = 150
    cetys = cetys.resize((round(cetys.width * lh / cetys.height), lh), Image.LANCZOS)
    img.alpha_composite(cetys, (W - 70 - cetys.width, 55))

    img.convert("RGB").save(AQUI / salida)
    print(AQUI / salida)


if __name__ == "__main__":
    portada(["OPEN", "CHALLENGE"], ["3 laps", "fully autonomous"], "portada_open.png")
    portada(["OBSTACLE", "CHALLENGE"], ["3 laps with traffic signs", "parallel parking"], "portada_obstacle.png")
    portada(["SURPRISE", "RULE"], ["2026 extra task", "special-height pillar"], "portada_surprise.png")


def banner(salida=REPO / "docs/img/banner.png"):
    """Banner 2000x560 para el inicio del README (mismos elementos que las portadas)."""
    BW, BH = 2000, 560
    y = np.linspace(0, 1, BH)[:, None]
    x = np.linspace(0, 1, BW)[None, :]
    base = 14 + 16 * (1 - y) * (1 - 0.5 * x)
    img = Image.fromarray(np.repeat(base[..., None], 3, 2).astype(np.uint8), "RGB")
    d = ImageDraw.Draw(img, "RGBA")
    for gx in range(0, BW, 50):
        d.line([(gx, 0), (gx, BH)], fill=(255, 255, 255, 7))
    for gy in range(0, BH, 50):
        d.line([(0, gy), (BW, gy)], fill=(255, 255, 255, 7))
    d.polygon([(1300, 0), (BW, 0), (BW, BH), (1160, BH)], fill=AMARILLO + (255,))
    d.polygon([(1262, 0), (1282, 0), (1142, BH), (1122, BH)], fill=AMARILLO + (255,))
    d.polygon([(1226, 0), (1236, 0), (1096, BH), (1086, BH)], fill=AMARILLO + (140,))
    d = ImageDraw.Draw(img)

    logo = Image.open(AQUI / "logo_foxrobotics.png").convert("RGBA")
    logo = logo.crop(logo.getbbox())
    lw = 430
    logo = logo.resize((lw, round(logo.height * lw / logo.width)), Image.LANCZOS)
    img.paste(logo, (80, 55), logo)
    yy = 55 + logo.height + 30
    d.text((84, yy), "WRO 2026  ·  FUTURE ENGINEERS", font=fuente("bahnschrift.ttf", 36), fill=GRIS)
    yy += 52
    d.text((80, yy), "SELF-DRIVING CAR", font=fuente("AGENCYB.TTF", 118), fill=AMARILLO)
    yy += 146
    d.rectangle([84, yy, 84 + 100, yy + 7], fill=AMARILLO)
    d.text((84, yy + 22), "International Final  ·  Car v2", font=fuente("bahnschrift.ttf", 38), fill=BLANCO)

    bandera = Image.open(AQUI / "bandera_mexico.png").convert("RGB").resize((84, 48), Image.LANCZOS)
    bx, by = 800, BH - 48 - 52
    d.rectangle([bx - 2, by - 2, bx + 84 + 1, by + 48 + 1], fill=(60, 60, 60))
    img.paste(bandera, (bx, by))
    d.text((bx + 104, by - 6), "MÉXICO", font=fuente("bahnschrift.ttf", 32), fill=BLANCO)
    d.text((bx + 104, by + 30), "Mexicali, B.C.", font=fuente("bahnschrift.ttf", 22), fill=GRIS)

    carro = Image.open(AQUI / "carro_recorte.png").convert("RGBA")
    ch = 470
    carro = carro.resize((round(carro.width * ch / carro.height), ch), Image.LANCZOS)
    cx, cy = 1385, (BH - ch) // 2 + 5
    sombra = Image.new("RGBA", (BW, BH), (0, 0, 0, 0))
    ImageDraw.Draw(sombra).ellipse([cx, cy + ch - 80, cx + carro.width - 10, cy + ch + 8], fill=(0, 0, 0, 110))
    img = Image.alpha_composite(img.convert("RGBA"), sombra.filter(ImageFilter.GaussianBlur(20)))
    img.alpha_composite(carro, (cx, cy))

    cetys = Image.open(AQUI / "logo_cetys.png").convert("RGBA")
    lh = 90
    cetys = cetys.resize((round(cetys.width * lh / cetys.height), lh), Image.LANCZOS)
    img.alpha_composite(cetys, (BW - 50 - cetys.width, 35))
    img.convert("RGB").save(salida, optimize=True)
    print(salida)
