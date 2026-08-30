from pathlib import Path
import sys

from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"


def draw_cube(draw: ImageDraw.ImageDraw, center_x: int, center_y: int, size: int, colors: tuple) -> None:
    half = size // 2
    quarter = size // 4
    top = (
        (center_x, center_y - half),
        (center_x + half, center_y - quarter),
        (center_x, center_y),
        (center_x - half, center_y - quarter),
    )
    left = (
        (center_x - half, center_y - quarter),
        (center_x, center_y),
        (center_x, center_y + half),
        (center_x - half, center_y + quarter),
    )
    right = (
        (center_x, center_y),
        (center_x + half, center_y - quarter),
        (center_x + half, center_y + quarter),
        (center_x, center_y + half),
    )
    draw.polygon(left, fill=colors[1])
    draw.polygon(right, fill=colors[2])
    draw.polygon(top, fill=colors[0])
    for polygon in (top, left, right):
        draw.line(tuple(polygon) + (polygon[0],), fill=(255, 255, 255, 150), width=9, joint="curve")


def build_icon() -> Image.Image:
    image = Image.new("RGBA", (1024, 1024), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image, "RGBA")
    draw.rounded_rectangle((44, 44, 980, 980), radius=218, fill=(16, 24, 40, 255))
    draw.rounded_rectangle((76, 76, 948, 948), radius=190, outline=(127, 86, 217, 130), width=18)
    draw.ellipse((168, 156, 856, 844), fill=(46, 144, 250, 22))
    draw_cube(draw, 512, 354, 300, ((154, 230, 255, 255), (18, 183, 106, 255), (46, 144, 250, 255)))
    draw_cube(draw, 366, 574, 274, ((191, 164, 255, 255), (105, 65, 198, 255), (127, 86, 217, 255)))
    draw_cube(draw, 658, 574, 274, ((254, 240, 199, 255), (247, 144, 9, 255), (253, 176, 34, 255)))
    draw.arc((280, 704, 744, 934), start=198, end=340, fill=(255, 255, 255, 210), width=24)
    draw.polygon(((728, 850), (754, 916), (680, 898)), fill=(255, 255, 255, 225))
    return image


def main() -> int:
    ASSETS.mkdir(parents=True, exist_ok=True)
    image = build_icon()
    png_path = ASSETS / "app_icon.png"
    ico_path = ASSETS / "app_icon.ico"
    image.save(png_path)
    image.save(ico_path, sizes=[(16, 16), (24, 24), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])
    if sys.platform == "darwin":
        image.save(ASSETS / "app_icon.icns", format="ICNS")
    print(png_path)
    print(ico_path)
    if (ASSETS / "app_icon.icns").is_file():
        print(ASSETS / "app_icon.icns")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
