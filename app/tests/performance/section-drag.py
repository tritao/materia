#!/usr/bin/env python3
"""Matched AP9 editor captures; GPU cost is independent of X11 frame pacing.

Build haxeon.presentation-capture.json first. Run from any working directory.
The diagnostic switches affect only this process, and no project is saved.
"""
import argparse
import bisect
import json
import os
from pathlib import Path
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[3]
VIEWS = (
    ("socket", "see-inside", "machinekit/examples/flexspline-hobber/build/socket-fixture/socket-fixture-assembled.mtrg"),
    ("gripper", "jaw-detail", "machinekit/examples/dh-gripper/build/pgc-custom-fingers.mtrg"),
)


def summarize(directory):
    lines = (directory / "run.log").read_text().splitlines()
    frames = [json.loads(line.removeprefix("section-drag-frame ")) for line in lines if line.startswith("section-drag-frame ")]
    summary = next(json.loads(line.removeprefix("section-drag ")) for line in lines if line.startswith("section-drag "))
    gpu = sorted((json.loads(line) for line in (directory / "gpu-timings.jsonl").read_text().splitlines()), key=lambda row: row["startedAtSeconds"])
    starts = [row["startedAtSeconds"] for row in gpu]
    measured = [gpu[bisect.bisect_right(starts, frame["sampledAtSeconds"]) - 1] for frame in frames]
    assert len(frames) == 60 and len({row["frame"] for row in measured}) == 60, "GPU/UI frame correspondence lost"
    assert all(row["gpuMilliseconds"] is not None for row in measured), "GPU timer results unavailable"
    camera=measured[0]["viewProjection"]
    assert all(row["viewProjection"] == camera for row in measured), "Camera moved during the measured sweep"
    expected_planes=0 if summary["mode"]=="none" else 1
    assert all(row["clipPlaneCount"]==expected_planes for row in measured), "Unexpected section state"
    summary["viewProjection"] = camera
    summary["gpuMeanMs"] = statistics.mean(row["gpuMilliseconds"] for row in measured)
    summary["gpuP95Ms"] = sorted(row["gpuMilliseconds"] for row in measured)[57]
    summary["prepareMeanMs"] = statistics.mean(row["prepareMilliseconds"] for row in measured)
    summary["rendererSubmitMeanMs"] = statistics.mean(row["submitMilliseconds"] for row in measured)
    passes = {key for row in measured for key in row["gpuPassMilliseconds"]}
    summary["gpuPassMeanMs"] = {key: statistics.mean(row["gpuPassMilliseconds"].get(key, 0) for row in measured) for key in sorted(passes)}
    (directory / "measured-gpu.json").write_text(json.dumps(measured, indent=2) + "\n")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "build/presentation-ap9-baseline")
    parser.add_argument("--seconds", type=int, default=12)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--example", choices=[row[0] for row in VIEWS])
    parser.add_argument("--pacing", choices=("normal", "bypass"))
    parser.add_argument("--mode", choices=("none", "caps", "contours"))
    options = parser.parse_args()
    native = ROOT / "app/tests/presentation-capture/build/host/native"
    environment = dict(os.environ, HAXEON_HOME=str(ROOT / "haxeon"))
    environment["LD_LIBRARY_PATH"] = ":".join(map(str, [ROOT / "haxeon/out", ROOT / "haxeon/.tools/hashlink", *sorted(native.iterdir())])) + ":" + environment.get("LD_LIBRARY_PATH", "")
    environment["NK_SCENE_GPU_TIMING_INTERVAL"] = "1"
    summaries = []
    cameras = {}
    for repeat in range(options.repeat):
        for name, view, project in VIEWS:
            if options.example and name != options.example:
                continue
            for bypass in (False, True):
                if options.pacing and bypass != (options.pacing == "bypass"):
                    continue
                for mode in ("none", "caps", "contours"):
                    if options.mode and mode != options.mode:
                        continue
                    directory = options.output.resolve() / f"{repeat}-{name}-{'bypass' if bypass else 'normal'}-{mode}"
                    directory.mkdir(parents=True, exist_ok=True)
                    environment["NK_X11_DISABLE_FRAME_SYNC"] = "1" if bypass else "0"
                    environment["NK_SCENE_GPU_TIMINGS"] = str(directory / "gpu-timings.jsonl")
                    (directory / "gpu-timings.jsonl").write_text("")
                    args = [str(ROOT / "haxeon/.tools/hashlink/hl"), str(ROOT / "app/tests/presentation/build/host/capture.hl"), f"--view={view}", f"--section-mode={mode}", "--section-drag=60", f"--project={ROOT / project}", f"--capture-dir={directory}", f"--capture-seconds={options.seconds}", "--msaa=4", "--reset-workspace", "--perspective", "--width=1800", "--height=900"]
                    for attempt in range(4):
                        directory.mkdir(parents=True,exist_ok=True)
                        (directory / "gpu-timings.jsonl").write_text("")
                        with (directory / "run.log").open("w") as log:
                            subprocess.run(args, cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=120)
                        try:
                            summary = dict(summarize(directory), example=name, bypass=bypass, repeat=repeat)
                            reference=cameras.setdefault(name,summary["viewProjection"])
                            assert summary["viewProjection"]==reference, "Camera differs from the matched baseline"
                            break
                        except AssertionError as error:
                            print(f"Rejecting {directory.name}: {error}",flush=True)
                            directory.rename(directory.with_name(directory.name+f"-rejected-{attempt}"))
                            if attempt==3:
                                raise
                    summaries.append(summary)
                    (options.output / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")
                    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
