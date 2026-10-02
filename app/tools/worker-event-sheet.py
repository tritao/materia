#!/usr/bin/env python3
"""A contact sheet of the rack-to-table job at the moments that matter, labelled with what the worker was doing.

Numbers say a motion stayed inside its limits; a frame says whether it looks right. Picking the moments by wall-clock
guess misses the grasp or the release, so this first runs the job headless with a trace (`--worker-demo-trace`),
reads when each action began, and then captures a frame just after each, with the action's name on its tile.

    python3 app/tools/worker-event-sheet.py --out /tmp/events --crop 700,300,1700,1000
    python3 app/tools/worker-event-sheet.py --asset animkit/assets/quaternius-ual/ual-work.glb --surface 0.55

--asset and --surface make a temporary variant of the demo (the character, and the height of the rack and table tops)
and put the example back afterwards. The captures run in real time, so a frame lands a little before the traced moment;
--after nudges it later.
"""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
EXAMPLE = ROOT / "app/examples/worker-rack-to-table.materia"
APP = ROOT / "app/run-built.sh"


def load_capture():
    spec = importlib.util.spec_from_file_location("capture_worker_frames", HERE / "capture-worker-frames.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def variant(asset, surface):
    """Rewrites the demo's character and surface heights; returns the original text to put back."""
    original = EXAMPLE.read_text()
    document = json.loads(original)
    for item in document["objects"]:
        if surface is not None and item["id"] in ("worker-demo-rack", "worker-demo-table"):
            item["z"] = surface - 0.05
        if surface is not None and item["id"] == "worker-demo-part":
            item["z"] = surface + 0.04
        if asset is not None and item["id"] == "worker-demo":
            item["worker"]["asset"] = asset
    EXAMPLE.write_text(json.dumps(document, indent=1))
    return original


def trace(ticks):
    command = ["xvfb-run", "-a", str(APP), "--snapshot", "--worker-demo=rack-to-table", f"--worker-demo-step={ticks}", "--worker-demo-trace"]
    result = subprocess.run(command, env=dict(os.environ, LIBGL_ALWAYS_SOFTWARE="1"), capture_output=True, text=True, timeout=1500)
    for line in result.stdout.splitlines():
        if line.startswith('{"workerDemo"'):
            return json.loads(line)
    sys.exit(f"the trace printed no result ({result.returncode}):\n{result.stdout[-2000:]}\n{result.stderr[-2000:]}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=Path("worker-events"))
    parser.add_argument("--asset", help="the character's glTF, relative to the repository")
    parser.add_argument("--surface", type=float, help="height of the rack and table tops, in metres")
    parser.add_argument("--ticks", type=int, default=1500, help="how many simulation ticks to trace")
    parser.add_argument("--after", type=float, default=0.25, help="seconds after an action began to take its frame")
    parser.add_argument("--width", type=int, default=2560)
    parser.add_argument("--height", type=int, default=1600)
    parser.add_argument("--crop", help="x0,y0,x1,y1 in frame pixels")
    parser.add_argument("--columns", type=int, default=3)
    arguments = parser.parse_args()
    if not APP.exists():
        sys.exit("build the app first: ./haxeon/scripts/haxeon build --project=app/haxeon.json")
    capture = load_capture()
    original = variant(arguments.asset, arguments.surface) if arguments.asset or arguments.surface is not None else None
    try:
        result = trace(arguments.ticks)
        events = result["timeline"]
        print(f"job {'done' if result['jobDone'] else 'not done'}; {len(events)} changes of action")
        frames = []
        for event in events:
            label = f"{event['action']} (step {event['step']}){' crouched' if event['crouch'] > 0.02 else ''}{' walking' if event['walking'] else ''}"
            moment = round(event["seconds"] + arguments.after, 2)
            frame = capture.capture(str(moment), arguments.out / f"t{moment}", arguments.width, arguments.height, "rack-to-table")
            print(f"{moment}s {label} -> {frame}")
            frames.append((f"{moment}s {label}", frame))
    finally:
        if original is not None:
            EXAMPLE.write_text(original)
    crop = tuple(int(part) for part in arguments.crop.split(",")) if arguments.crop else None
    sheet = arguments.out / "contact-sheet.png"
    capture.contact_sheet(frames, crop, arguments.columns, sheet)
    print(f"contact sheet -> {sheet}")


if __name__ == "__main__":
    main()
