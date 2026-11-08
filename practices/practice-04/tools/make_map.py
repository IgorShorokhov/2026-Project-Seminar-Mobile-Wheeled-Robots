import math
from pathlib import Path

RES = 0.01
N = 1200
X0 = -6.0
Y0 = -6.0
MAPS = Path(__file__).resolve().parent.parent / "practice_webots_localization" / "maps"


def fill_rect(img, cx, cy, w, h, ang=0.0):
    c, s = math.cos(ang), math.sin(ang)
    reach = math.hypot(w, h) / 2
    c0 = max(0, int((cx - reach - X0) / RES))
    c1 = min(N - 1, int((cx + reach - X0) / RES) + 1)
    r0 = max(0, int((Y0 + N * RES - (cy + reach)) / RES))
    r1 = min(N - 1, int((Y0 + N * RES - (cy - reach)) / RES) + 1)
    for r in range(r0, r1 + 1):
        y = Y0 + (N - r - 0.5) * RES
        for col in range(c0, c1 + 1):
            x = X0 + (col + 0.5) * RES
            dx, dy = x - cx, y - cy
            u = c * dx + s * dy
            v = -s * dx + c * dy
            if abs(u) <= w / 2 and abs(v) <= h / 2:
                img[r][col] = 0


def pixel_at(img, x, y):
    col = int((x - X0) / RES)
    r = int((Y0 + N * RES - y) / RES)
    return img[r][col]


def save_pgm(img, path):
    with open(path, "wb") as f:
        f.write(f"P5\n{N} {N}\n255\n".encode())
        for row in img:
            f.write(row)


def main():
    img = [bytearray(b"\xff" * N) for _ in range(N)]

    wall = 0.03
    fill_rect(img, 0.0, 6.0 - wall / 2, 12.0, wall)
    fill_rect(img, 0.0, -6.0 + wall / 2, 12.0, wall)
    fill_rect(img, 6.0 - wall / 2, 0.0, wall, 12.0)
    fill_rect(img, -6.0 + wall / 2, 0.0, wall, 12.0)

    fill_rect(img, -1.0, 0.0, 0.1, 4.0)
    fill_rect(img, -1.5, 1.5, 0.1, 1.414, -math.pi / 4)

    fill_rect(img, 1.0, 0.0, 0.1, 4.0)
    fill_rect(img, 3.0, 0.0, 0.1, 4.0)
    fill_rect(img, 2.0, 2.0, 2.0, 0.1)
    fill_rect(img, 2.0, 0.0, 2.0, 0.1)
    fill_rect(img, 2.0, -2.0, 2.0, 0.1)

    checks = [
        (-1.0, 0.0, 0), (0.0, 0.0, 255), (-1.5, 1.5, 0), (-1.5, 0.0, 255),
        (2.0, 0.0, 0), (2.0, 0.5, 255), (1.0, 1.0, 0), (5.0, 0.0, 255),
        (0.0, 5.985, 0), (0.0, 5.0, 255),
    ]
    for x, y, expected in checks:
        got = pixel_at(img, x, y)
        if got != expected:
            raise SystemExit(f"check failed at ({x}, {y}): got {got}, expected {expected}")
    print("pixel checks: OK")

    MAPS.mkdir(parents=True, exist_ok=True)
    save_pgm(img, MAPS / "my_world.pgm")
    (MAPS / "my_world.yaml").write_text(
        "image: my_world.pgm\n"
        "mode: trinary\n"
        "resolution: 0.01\n"
        "origin: [-6.0, -6.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    print(f"written: {MAPS / 'my_world.pgm'} and {MAPS / 'my_world.yaml'}")


if __name__ == "__main__":
    main()
