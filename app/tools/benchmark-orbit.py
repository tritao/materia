#!/usr/bin/env python3
"""Measure a fixed full-turn desktop orbit, including the ordinary UI frame path."""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import statistics
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
DRAIN_FRAMES = 8  # Let the final measured GPU queries become ready without a blocking wait.
SLEEPING_DISPLAY_STATES = {"Off", "Standby", "Suspend"}


def display_sample():
    state = "unavailable"
    if shutil.which("xset"):
        try:
            result = subprocess.run(["xset", "q"], capture_output=True, text=True, timeout=2)
            for line in result.stdout.splitlines():
                if "Monitor is " in line:
                    state = line.split("Monitor is ", 1)[1].strip()
        except subprocess.TimeoutExpired:
            pass
    return {"atSeconds": time.time(), "state": state}


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def inventory_summary(path):
    with path.open() as source:
        nodes = list(csv.DictReader(source, delimiter="\t"))
    visible = [node for node in nodes if node["hidden"] == "0"]
    return {
        "nodes": len(nodes), "shownNodes": len(visible),
        "shownGeometries": len({node["geometry"] for node in visible}),
        "shownMaterials": len({node["material"] for node in visible}),
        "shownGeometryMaterialPairs": len({(node["geometry"], node["material"]) for node in visible}),
        "largestMeshes": [dict(name=node["name"], triangles=int(node["triangles"]),
                               strokes=int(node["strokes"]), geometry=node["geometry"], material=node["material"])
                          for node in sorted(visible, key=lambda node: int(node["triangles"]), reverse=True)[:20]],
        "mostStrokes": [dict(name=node["name"], strokes=int(node["strokes"]))
                        for node in sorted(visible, key=lambda node: int(node["strokes"]), reverse=True)[:20]],
    }


def summarize(output, warmup, count):
    state = json.loads((output / "app-state.json").read_text())
    objects = state["scene"]["objects"]
    if not any(item["id"].startswith("project:") for item in objects):
        raise RuntimeError("benchmark project did not load; inspect app-state.json and launch.log")
    inputs = state["perspective"]["benchmarkInputs"]
    gpu = records(output / "scene-timeline.jsonl")
    host = records(output / "frame-timeline.jsonl")
    # Loading can render many frames. The last N are the bounded ready capture;
    # each measured frame must contain an actual scene render, matched by wall time.
    host = host[-(warmup + count + DRAIN_FRAMES):][warmup:warmup + count]
    if len(host) != count:
        raise RuntimeError(f"expected {count} measured host frames, got {len(host)}")
    display_path = output / "display-timeline.jsonl"
    if display_path.exists():
        display = records(display_path)
        start = host[0]["startedAtSeconds"]
        end = host[-1]["startedAtSeconds"] + host[-1]["frameSeconds"]
        for index, sample in enumerate(display):
            next_at = display[index + 1]["atSeconds"] if index + 1 < len(display) else float("inf")
            if sample["state"] in SLEEPING_DISPLAY_STATES and sample["atSeconds"] <= end and next_at >= start:
                raise RuntimeError("display entered DPMS power saving during the measured window; run is invalid")
    rows = []
    for frame in host:
        start = frame["startedAtSeconds"]
        end = start + frame["frameSeconds"]
        samples = [sample for sample in gpu if start <= sample["startedAtSeconds"] <= end]
        if len(samples) != 1:
            raise RuntimeError(f"host frame {frame['frame']} contains {len(samples)} scene samples")
        sample = samples[0]
        input_samples = [item for item in inputs if start <= item["startedAtSeconds"] <= end]
        if len(input_samples) != 1:
            raise RuntimeError(f"host frame {frame['frame']} contains {len(input_samples)} orbit inputs")
        input_sample = input_samples[0]
        if not input_sample["orbiting"]:
            raise RuntimeError("scripted middle drag did not capture the perspective viewport")
        if sample["gpuMilliseconds"] is None:
            raise RuntimeError(f"missing GPU timer result for scene frame {sample['frame']}")
        rows.append(dict(
            frame=frame["frame"], startedAtSeconds=start,
            cpuMs=frame["frameSeconds"] * 1000,
            inputMs=input_sample["inputMilliseconds"], yaw=input_sample["yaw"], pitch=input_sample["pitch"],
            viewSetupMs=input_sample["viewSetupMilliseconds"], planPrepareMs=input_sample["planPrepareMilliseconds"],
            uiSubmitMs=frame["applicationSubmitSeconds"] * 1000,
            customPaintMs=frame["customPaintSeconds"] * 1000,
            nativeUiRenderMs=frame["nativeRenderSeconds"] * 1000,
            gcMs=frame["frameGcSeconds"] * 1000,
            scenePrepareMs=sample["prepareMilliseconds"],
            sceneSubmitMs=sample["submitMilliseconds"],
            gpuMs=sample["gpuMilliseconds"],
            gpuPassMs=sample["gpuPassMilliseconds"],
            passWork=sample.get("passWork", {}),
            drawCalls=sample["drawCalls"], triangles=sample["triangles"],
            uploadBytes=sample["uploadBytes"],
            lodDrawCalls=sample.get("lodDrawCalls", 0), lodTrianglesSaved=sample.get("lodTrianglesSaved", 0),
        ))
    for previous, current in zip(rows, rows[1:]):
        if not math.isclose(current["yaw"] - previous["yaw"], math.pi / 180, abs_tol=1e-7):
            raise RuntimeError("camera did not advance by one degree per frame")
        if not math.isclose(current["pitch"], previous["pitch"], abs_tol=1e-7):
            raise RuntimeError("horizontal orbit changed the camera pitch")
    (output / "orbit-frames.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    cadence = [(b["startedAtSeconds"] - a["startedAtSeconds"]) * 1000 for a, b in zip(rows, rows[1:])]
    summary = {"frames": len(rows), "warmupFrames": warmup,
               "fps": 1000 / statistics.mean(cadence),
               "cadenceMs": statistics.mean(cadence)}
    for name in ("cpuMs", "inputMs", "viewSetupMs", "planPrepareMs", "uiSubmitMs", "customPaintMs", "nativeUiRenderMs", "gcMs",
                 "scenePrepareMs", "sceneSubmitMs", "gpuMs",
                 "drawCalls", "triangles", "uploadBytes", "lodDrawCalls", "lodTrianglesSaved"):
        values = sorted(row[name] for row in rows)
        summary[name] = {"mean": statistics.mean(values), "median": statistics.median(values),
                         "p95": values[int((len(values) - 1) * .95)], "max": max(values)}
    summary["gpuPassMs"] = {}
    summary["passWork"] = {}
    for name in sorted(rows[0]["gpuPassMs"]):
        values = sorted(row["gpuPassMs"][name] for row in rows)
        summary["gpuPassMs"][name] = {"median": statistics.median(values),
                                         "p95": values[int((len(values) - 1) * .95)]}
    for name in sorted(rows[0]["passWork"]):
        summary["passWork"][name] = {
            metric: statistics.median(row["passWork"][name][metric] for row in rows)
            for metric in ("drawCalls", "triangles")}
    inventory = output / "scene-timeline.jsonl.inventory.tsv"
    if inventory.exists():
        summary["inventory"] = inventory_summary(inventory)
    (output / "orbit-summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print("frame,cpu_ms,gpu_ms,draw_calls,triangles,upload_bytes")
    for row in rows:
        print(f"{row['frame']},{row['cpuMs']:.3f},{row['gpuMs']:.3f},"
              f"{row['drawCalls']},{row['triangles']},{row['uploadBytes']}")
    print(json.dumps(summary, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=360)
    parser.add_argument("--warmup", type=int, default=60)
    parser.add_argument("--msaa", type=int, choices=(1, 2, 4), default=4)
    parser.add_argument("--frame-object", help="frame a scene object before orbiting (for close-zoom checks)")
    parser.add_argument("--project", type=Path,
                        default=ROOT / "machinekit/examples/flexspline-hobber/materia.project.json",
                        help="project to orbit (default: flexspline hobber)")
    parser.add_argument("--launcher", default="", help="optional command prefix, e.g. 'heavy-run --slots 4 --need 4G --'")
    parser.add_argument("--skip-build", action="store_true", help="reuse the built application and native libraries")
    parser.add_argument("--native-library-dir", type=Path,
                        help="prefer a native library directory for a paired renderer variant; requires --skip-build")
    parser.add_argument("--report-only", action="store_true")
    args = parser.parse_args()
    if args.frames < 2 or args.warmup < 1:
        parser.error("use at least two measured frames and one warmup frame")
    if args.native_library_dir is not None and not args.skip_build:
        parser.error("--native-library-dir requires --skip-build")
    output = args.output.resolve()
    if not args.report_only:
        output.mkdir(parents=True, exist_ok=False)
        initial_display = display_sample()
        if initial_display["state"] in SLEEPING_DISPLAY_STATES:
            raise RuntimeError("display is in DPMS power saving; wake it before benchmarking (xset dpms force on)")
        environment = os.environ.copy()
        environment.update(NK_SCENE_GPU_TIMINGS=str(output / "scene-timeline.jsonl"),
                           NK_SCENE_GPU_TIMING_INTERVAL="1",
                           REFERENCE_EDITOR_WORKSPACE=str(output / "workspace.json"),
                           MATERIA_ROOT=str(ROOT), TMPDIR=str(output / "tmp"),
                           HAXEON_COMPILER_IDLE_SECONDS="1")
        environment.pop("MATERIA_NATIVE_LIBRARY_PATH", None)
        (output / "tmp").mkdir()
        if args.native_library_dir is not None:
            directory = args.native_library_dir.resolve()
            if not (directory / "libnativekit_scene_render.so").is_file():
                raise RuntimeError("native library directory must contain libnativekit_scene_render.so")
            environment["MATERIA_NATIVE_LIBRARY_PATH"] = str(directory)
        launch = ([str(ROOT / "app/run-built.sh")] if args.skip_build else
                  [str(ROOT / "haxeon/scripts/haxeon"), "run", "--project", str(ROOT / "app/haxeon.json"), "--"])
        command = shlex.split(args.launcher) + launch + ["--orbit-benchmark",
                   "--perspective", "--width=1320", "--height=900", f"--msaa={args.msaa}",
                   f"--capture-dir={output}", f"--frames={args.frames + args.warmup + DRAIN_FRAMES}",
                   "--project=" + str(args.project.resolve())]
        if args.frame_object:
            command.append("--frame=" + args.frame_object)
        (output / "command.json").write_text(json.dumps(command, indent=2) + "\n")
        metadata = {"arguments": {key: str(value) if isinstance(value, Path) else value
                                  for key, value in vars(args).items()} | {"output": str(output), "project": str(args.project.resolve())},
                    "display": environment.get("DISPLAY")}
        for name, query in (("commit", ["git", "rev-parse", "HEAD"]),
                            ("haxeonCommit", ["git", "-C", "haxeon", "rev-parse", "HEAD"]),
                            ("gpu", ["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"])):
            if shutil.which(query[0]) is None:
                metadata[name] = None
                continue
            result = subprocess.run(query, cwd=ROOT, capture_output=True, text=True)
            metadata[name] = result.stdout.strip()
        (output / "environment.json").write_text(json.dumps(metadata, indent=2) + "\n")
        with (output / "launch.log").open("w") as log, (output / "display-timeline.jsonl").open("w") as display_log:
            with subprocess.Popen(command, cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT) as process:
                while True:
                    display_log.write(json.dumps(display_sample()) + "\n")
                    display_log.flush()
                    try:
                        result = process.wait(timeout=1)
                        break
                    except subprocess.TimeoutExpired:
                        pass
            display_log.write(json.dumps(display_sample()) + "\n")
        if result:
            raise RuntimeError(f"editor exited {result}; see {output / 'launch.log'}")
        renderer = ((args.native_library_dir.resolve() if args.native_library_dir is not None else
                     ROOT / "app/build/host/native/app") / "libnativekit_scene_render.so").resolve()
        metadata["sceneRendererBinary"] = {"path": str(renderer), "sha256": hashlib.sha256(renderer.read_bytes()).hexdigest()}
        (output / "environment.json").write_text(json.dumps(metadata, indent=2) + "\n")
    summarize(output, args.warmup, args.frames)


if __name__ == "__main__":
    main()
