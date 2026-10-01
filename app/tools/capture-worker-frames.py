#!/usr/bin/env python3
"""Capture frames of the rack-to-table worker demo at chosen moments, to judge a pose by eye.

Numbers catch flips and snaps, but only a frame shows whether a pose looks natural. This runs the built
app headless under software GL, once per moment, and writes one frame for each plus a contact sheet.

    python3 app/tools/capture-worker-frames.py --seconds 1.3,2,5 --crop 1130,380,1530,680 --out /tmp/worker

The app runs in real time, so a moment is wall-clock seconds after launch and lands a little before the
same simulation time (launch takes a moment). Frames are 2560x1600 by default; --crop takes a box in
those pixels (x0,y0,x1,y1) so the worker fills a tile, and the contact sheet needs Pillow.
"""

import argparse
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / "app/run-built.sh"


def capture(seconds, directory, width, height, demo):
    directory.mkdir(parents=True, exist_ok=True)
    command = [
        "xvfb-run", "-a", "-s", f"-screen 0 {width + 40}x{height + 100}x24", str(APP),
        f"--worker-demo={demo}", "--perspective", f"--width={width}", f"--height={height}",
        f"--capture-dir={directory}", f"--capture-seconds={seconds}",
    ]
    environment = dict(os.environ, LIBGL_ALWAYS_SOFTWARE="1")
    result = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=300)
    frame = directory / "frame.png"
    if result.returncode != 0 or not frame.exists():
        sys.exit(f"capture at {seconds}s failed ({result.returncode}):\n{result.stdout}\n{result.stderr}")
    return frame


def contact_sheet(frames, crop, columns, output):
    from PIL import Image, ImageDraw

    tiles = []
    for seconds, path in frames:
        image = Image.open(path)
        if crop:
            image = image.crop(crop)
        label = seconds if isinstance(seconds, str) and not seconds.replace(".", "").isdigit() else f"{seconds}s"
        ImageDraw.Draw(image).text((8, 8), label, fill=(0, 0, 0))
        tiles.append(image)
    width, height = tiles[0].size
    rows = (len(tiles) + columns - 1) // columns
    sheet = Image.new("RGB", (width * columns, height * rows), "white")
    for index, tile in enumerate(tiles):
        sheet.paste(tile, ((index % columns) * width, (index // columns) * height))
    sheet.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", required=True, help="comma-separated moments, in seconds after launch")
    parser.add_argument("--out", type=Path, default=Path("worker-frames"), help="directory for the frames")
    parser.add_argument("--demo", default="rack-to-table")
    parser.add_argument("--width", type=int, default=2560)
    parser.add_argument("--height", type=int, default=1600)
    parser.add_argument("--crop", help="x0,y0,x1,y1 in frame pixels for the contact sheet")
    parser.add_argument("--columns", type=int, default=2)
    arguments = parser.parse_args()
    if not APP.exists():
        sys.exit("build the app first: ./haxeon/scripts/haxeon build --project=app/haxeon.json")
    moments = [value.strip() for value in arguments.seconds.split(",") if value.strip()]
    crop = tuple(int(part) for part in arguments.crop.split(",")) if arguments.crop else None
    if crop is not None and len(crop) != 4:
        sys.exit("--crop takes x0,y0,x1,y1")
    frames = []
    for seconds in moments:
        frame = capture(seconds, arguments.out / f"t{seconds}", arguments.width, arguments.height, arguments.demo)
        print(f"{seconds}s -> {frame}")
        frames.append((seconds, frame))
    try:
        sheet = arguments.out / "contact-sheet.png"
        contact_sheet(frames, crop, arguments.columns, sheet)
        print(f"contact sheet -> {sheet}")
    except ImportError:
        print("Pillow is not installed; frames were written without a contact sheet")


if __name__ == "__main__":
    main()
