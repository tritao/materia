#!/usr/bin/env python3
"""Compare the CAD lighting orbit gallery with small, checked-in image references."""

import argparse
import base64
import json
import os
from pathlib import Path
import subprocess
import tempfile
import zlib


GRID_WIDTH = 24
GRID_HEIGHT = 12
FRAMES = 8


def reduced_image(path: Path) -> bytes:
    header, dimensions, maximum, pixels = path.read_bytes().split(b"\n", 3)
    if header != b"P6" or maximum != b"255":
        raise ValueError(f"unsupported PPM: {path}")
    width, height = map(int, dimensions.split())
    if len(pixels) != width * height * 3:
        raise ValueError(f"incomplete PPM: {path}")
    result = bytearray()
    for row in range(GRID_HEIGHT):
        y0, y1 = row * height // GRID_HEIGHT, (row + 1) * height // GRID_HEIGHT
        for column in range(GRID_WIDTH):
            x0, x1 = column * width // GRID_WIDTH, (column + 1) * width // GRID_WIDTH
            sums = [0, 0, 0]
            for y in range(y0, y1):
                for x in range(x0, x1):
                    index = (y * width + x) * 3
                    for channel in range(3):
                        sums[channel] += pixels[index + channel]
            count = (x1 - x0) * (y1 - y0)
            result.extend((value + count // 2) // count for value in sums)
    return bytes(result)


def capture(gallery: Path, xvfb: Path) -> dict[str, bytes]:
    output = {}
    environment = os.environ.copy()
    environment["LIBGL_ALWAYS_SOFTWARE"] = "1"
    with tempfile.TemporaryDirectory(prefix="materia-lighting-") as directory:
        for variant in ("bright", "dark"):
            target = Path(directory) / variant
            target.mkdir()
            command = [str(xvfb), "-a", str(gallery), str(target)]
            if variant == "dark":
                command.append("dark")
            subprocess.run(command, check=True, env=environment, timeout=60)
            output[variant] = b"".join(
                reduced_image(target / f"orbit-{frame}.ppm") for frame in range(FRAMES)
            )
    return output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gallery", type=Path, required=True)
    parser.add_argument("--xvfb", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--update", action="store_true")
    args = parser.parse_args()
    current = capture(args.gallery, args.xvfb)
    if args.update:
        document = {
            "grid": [GRID_WIDTH, GRID_HEIGHT],
            "frames": FRAMES,
            "variants": {
                name: base64.b64encode(zlib.compress(pixels, 9)).decode("ascii")
                for name, pixels in current.items()
            },
        }
        args.reference.write_text(json.dumps(document, indent=2) + "\n")
        print(f"updated {args.reference}")
        return

    reference = json.loads(args.reference.read_text())
    if reference["grid"] != [GRID_WIDTH, GRID_HEIGHT] or reference["frames"] != FRAMES:
        raise SystemExit("lighting reference dimensions do not match the gallery")
    for variant, pixels in current.items():
        expected = zlib.decompress(base64.b64decode(reference["variants"][variant]))
        if len(expected) != len(pixels):
            raise SystemExit(f"{variant}: reference length differs")
        delta = [abs(a - b) for a, b in zip(pixels, expected)]
        mean = sum(delta) / len(delta)
        maximum = max(delta)
        if mean > 2.0 or maximum > 20:
            raise SystemExit(f"{variant}: lighting changed (mean={mean:.2f}, max={maximum})")
        print(f"{variant}: mean difference {mean:.2f}, maximum {maximum}")


if __name__ == "__main__":
    main()
