#!/usr/bin/env python3
"""生成动图红绿灯的整面贴图，供材质按红、黄、绿顺序播放。"""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter


MODEL_DIR = Path(__file__).resolve().parents[1] / "models"
SOURCE = MODEL_DIR / "traffic_light/materials/textures"
OUTPUT = MODEL_DIR / "traffic_light_animated/materials/textures"
FRAME_STATES = ("red", "red", "yellow", "green", "green", "green")
COLORS = ("red", "yellow", "green")
FACE_SIZE = (1180, 280)
LAMP_SIZE = 260
CENTERS = (190, 590, 990)


def build_frame(active):
    frame = Image.new("RGB", FACE_SIZE, (15, 16, 18))
    mask = Image.new("L", (LAMP_SIZE, LAMP_SIZE))
    ImageDraw.Draw(mask).ellipse((2, 2, LAMP_SIZE - 3, LAMP_SIZE - 3), fill=255)
    mask = mask.filter(ImageFilter.GaussianBlur(2))

    for color, center in zip(COLORS, CENTERS):
        state = "on" if color == active else "dark"
        with Image.open(SOURCE / "{}_{}.png".format(color, state)) as source:
            lamp = source.convert("RGB").resize(
                (LAMP_SIZE, LAMP_SIZE), Image.Resampling.LANCZOS)
        frame.paste(lamp, (center - LAMP_SIZE // 2, 10), mask)
    return frame


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    # 使用模型专属名称，避免 OGRE 从全局资源组命中同名旧贴图。
    # 重复播放同一张图片的时长由 material 中的显式帧列表决定。
    for active in COLORS:
        build_frame(active).save(OUTPUT / "traffic_light_animated_{}_v2.png".format(active))


if __name__ == "__main__":
    main()
