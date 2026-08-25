#!/usr/bin/env python3
"""Crop a generated floor plan and add a compact metric summary."""

import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--length", type=float, required=True)
    parser.add_argument("--width", type=float, required=True)
    parser.add_argument("--area", type=float, required=True)
    args = parser.parse_args()

    source = Image.open(args.input).convert("RGB")
    pixels = source.load()
    active = []
    for y in range(source.height):
        for x in range(source.width):
            red, green, blue = pixels[x, y]
            if min(red, green, blue) < 245:
                active.append((x, y))
    if not active:
        raise RuntimeError("floor plan contains no visible geometry")
    left = max(0, min(point[0] for point in active) - 28)
    top = max(0, min(point[1] for point in active) - 28)
    right = min(source.width, max(point[0] for point in active) + 29)
    bottom = min(source.height, max(point[1] for point in active) + 29)
    cropped = source.crop((left, top, right, bottom))

    header_height = 74
    canvas = Image.new("RGB", (max(520, cropped.width),
                                cropped.height + header_height), "white")
    canvas.paste(cropped, ((canvas.width - cropped.width) // 2, header_height))
    draw = ImageDraw.Draw(canvas)
    font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    title_font = ImageFont.truetype(font_path, 22)
    metric_font = ImageFont.truetype(font_path, 17)
    title = "最新户型图结果"
    metrics = (f"长：{args.length:.2f} m    宽：{args.width:.2f} m    "
               f"闭合轮廓面积：{args.area:.2f} m²")
    title_box = draw.textbbox((0, 0), title, font=title_font)
    metric_box = draw.textbbox((0, 0), metrics, font=metric_font)
    draw.text(((canvas.width - (title_box[2] - title_box[0])) / 2, 7),
              title, fill=(20, 20, 20), font=title_font)
    draw.text(((canvas.width - (metric_box[2] - metric_box[0])) / 2, 40),
              metrics, fill=(70, 70, 70), font=metric_font)
    draw.line((0, header_height - 1, canvas.width, header_height - 1),
              fill=(210, 210, 210), width=1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(args.output, format="PNG", optimize=True)


if __name__ == "__main__":
    main()
