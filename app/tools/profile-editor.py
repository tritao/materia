#!/usr/bin/env python3
"""Capture a bounded editor run with HashLink, frame, and RSS timelines."""

import argparse
import json
import os
from pathlib import Path
import re
import socket
import statistics
import subprocess
import sys
import threading
import time


ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / "app"
HAXEON = ROOT / "haxeon/scripts/haxeon"


def rss_kb(pid):
    try:
        for line in (Path("/proc") / str(pid) / "status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1])
    except OSError:
        pass
    return None


def cpu_seconds(pid):
    try:
        stat = (Path("/proc") / str(pid) / "stat").read_text()
        fields = stat[stat.rfind(")") + 2 :].split()
        return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")
    except (OSError, ValueError, IndexError):
        return None


def run_scenario(pid, output):
    """Replay a small editor interaction against the test process's own window."""
    actions = output / "actions.jsonl"
    try:
        deadline = time.monotonic() + 10
        window = None
        while time.monotonic() < deadline:
            listing = subprocess.check_output(["wmctrl", "-lp"], text=True)
            for line in listing.splitlines():
                parts = line.split(maxsplit=4)
                if len(parts) == 5 and parts[2] == str(pid) and "Materia Reference Editor" in parts[4]:
                    window = parts[0]
                    break
            if window is not None:
                break
            time.sleep(0.1)
        if window is None:
            raise RuntimeError("editor window did not appear")
        geometry = subprocess.check_output(["xwininfo", "-id", window], text=True)
        x = int(re.search(r"Absolute upper-left X:\s*(-?\d+)", geometry).group(1))
        y = int(re.search(r"Absolute upper-left Y:\s*(-?\d+)", geometry).group(1))
        subprocess.run(["xdotool", "windowactivate", window], check=True)
        time.sleep(0.5)
        with actions.open("w") as log:
            for name, px, py in (("hierarchy", 40, 60), ("sensors", 120, 60),
                                 ("inspector-name", 1180, 320)):
                subprocess.run(["xdotool", "mousemove", str(x + px), str(y + py), "click", "1"], check=True)
                log.write(json.dumps({"action": name, "timeSeconds": time.time()}) + "\n")
                log.flush()
                time.sleep(0.35)
            subprocess.run(["xdotool", "windowactivate", "--sync", window], check=True)
            log.write(json.dumps({"action": "focus", "window": window,
                                  "activeWindow": subprocess.check_output(["xdotool", "getactivewindow"], text=True).strip(),
                                  "timeSeconds": time.time()}) + "\n")
            subprocess.run(["xdotool", "key", "ctrl+a"], check=True)
            subprocess.run(["xdotool", "type", "--clearmodifiers", "--delay", "1",
                            "Profile box"], check=True)
            subprocess.run(["xdotool", "key", "Return"], check=True)
            log.write(json.dumps({"action": "rename", "timeSeconds": time.time()}) + "\n")
    except (OSError, ValueError, AttributeError, subprocess.CalledProcessError, RuntimeError) as error:
        actions.write_text(json.dumps({"error": str(error)}) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=240)
    parser.add_argument("--seconds", type=float, help="capture normal event-driven frames for this interval")
    parser.add_argument("--idle-seconds", type=float, help="sample an unbounded editor, then stop it")
    parser.add_argument("--no-profile", action="store_true", help="measure without profiler overhead")
    parser.add_argument("--scenario", choices=["tab-inspector"], help="replay a fixed UI interaction")
    parser.add_argument("--skip-build", action="store_true", help="reuse the existing compiled editor")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("editor_args", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be positive")
    if args.seconds is not None and args.seconds <= 0:
        parser.error("--seconds must be positive")
    if args.idle_seconds is not None and args.idle_seconds <= 0:
        parser.error("--idle-seconds must be positive")
    if args.seconds is not None and args.idle_seconds is not None:
        parser.error("--seconds and --idle-seconds are mutually exclusive")
    if args.scenario is not None and args.idle_seconds is not None:
        parser.error("--scenario requires a frame or timed capture")
    output = (args.output_dir or APP / "build/profiles" / time.strftime("%Y%m%d-%H%M%S")).resolve()
    output.mkdir(parents=True, exist_ok=False)
    editor_args = args.editor_args[1:] if args.editor_args[:1] == ["--"] else args.editor_args
    runtime = ROOT / "haxeon/.tools/hashlink/hl"
    profiler = ROOT / "haxeon/.tools/hashlink/hlprof-live"
    with (output / "launch.log").open("w") as log, (output / "memory.jsonl").open("w") as memory:
        if args.skip_build:
            if not (APP / "build/host/main.hl").exists():
                parser.error("--skip-build requires an existing app/build/host/main.hl")
            log.write("Reusing existing app/build/host/main.hl; source changes are not included.\n")
        else:
            build = subprocess.run([str(HAXEON), "build", "--project", str(APP / "haxeon.json")],
                                   cwd=APP, stdout=log, stderr=subprocess.STDOUT)
            if build.returncode:
                print(f"build failed; see {output / 'launch.log'}", file=sys.stderr)
                return build.returncode
        native_dirs = sorted({str(path.parent) for path in (APP / "build/host/native").rglob("*.so")})
        environment = os.environ.copy()
        environment["LD_LIBRARY_PATH"] = os.pathsep.join(
            [str(runtime.parent), str(ROOT / "haxeon/out"), *native_dirs,
             environment.get("LD_LIBRARY_PATH", "")])
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            port = reservation.getsockname()[1]
        command = [str(runtime)]
        if not args.no_profile:
            command += ["--diagnostics", str(port), "--diagnostics-wait"]
        command += [str(APP / "build/host/main.hl")]
        if args.idle_seconds is None:
            command += ["--capture-dir=" + str(output)]
            command += (["--capture-seconds=" + str(args.seconds)] if args.seconds is not None
                        else ["--frames=" + str(args.frames)])
        command += editor_args
        process = subprocess.Popen(command, cwd=APP, env=environment,
                                   stdout=log, stderr=subprocess.STDOUT)
        scenario = None
        if args.scenario is not None:
            scenario = threading.Thread(target=run_scenario, args=(process.pid, output), daemon=True)
            scenario.start()
        with (output / "profiler.log").open("w") as profile_log:
            capture = None if args.no_profile else subprocess.Popen(
                [str(profiler), "--connect-timeout", "15", "--rate", "500",
                 "--alloc-interval", "65536", "--interval", "2000",
                 "--output", str(output / "editor.hlpc"), str(port)],
                cwd=APP, stdout=profile_log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + (args.idle_seconds if args.idle_seconds is not None else 60)
                while process.poll() is None and time.monotonic() < deadline:
                    rss = rss_kb(process.pid)
                    if rss is not None:
                        memory.write(json.dumps({"timeSeconds": time.time(), "pid": process.pid,
                                                 "rssBytes": rss * 1024,
                                                 "cpuSeconds": cpu_seconds(process.pid)}) + "\n")
                    if capture is not None and capture.poll() not in (None, 0):
                        raise RuntimeError("hlprof-live failed to attach")
                    time.sleep(0.1)
                if process.poll() is None:
                    if args.idle_seconds is None:
                        raise TimeoutError("editor did not finish within 60 seconds")
                    process.terminate()
                result = process.wait(timeout=5)
                if args.idle_seconds is not None and result == -15:
                    result = 0
                if capture is not None:
                    capture.wait(timeout=10)
                if capture is not None and capture.returncode:
                    raise RuntimeError("hlprof-live failed; see profiler.log")
                if scenario is not None:
                    scenario.join(timeout=2)
            except (RuntimeError, TimeoutError, subprocess.TimeoutExpired) as error:
                process.terminate()
                if capture is not None:
                    capture.terminate()
                process.wait(timeout=5)
                if capture is not None:
                    capture.wait(timeout=5)
                print(f"{error}; see {output}", file=sys.stderr)
                return 1
    frames_file = output / "frame-timeline.jsonl"
    frames = [json.loads(line) for line in frames_file.read_text().splitlines()] if frames_file.exists() else []
    samples = [json.loads(line) for line in (output / "memory.jsonl").read_text().splitlines()]
    export = None if args.no_profile else subprocess.run(
        [str(profiler), "export", "--format", "perfetto",
         "--output", str(output / "editor.perfetto.json"),
         str(output / "editor.hlpc")], capture_output=True, text=True)
    if export is not None and export.returncode:
        print(f"Perfetto export failed: {export.stderr.strip()}", file=sys.stderr)
    if frames:
        durations = sorted(frame["frameSeconds"] * 1000 for frame in frames)
        print(f"frames={len(frames)} median={statistics.median(durations):.1f}ms "
              f"p95={durations[int((len(durations) - 1) * .95)]:.1f}ms max={durations[-1]:.1f}ms")
    if samples:
        start = next((row for row in samples if frames and row["timeSeconds"] >= frames[0]["startedAtSeconds"]), samples[0])
        print(f"RSS start={start['rssBytes'] / 2**20:.1f}MiB "
              f"last={samples[-1]['rssBytes'] / 2**20:.1f}MiB "
              f"peak={max(row['rssBytes'] for row in samples) / 2**20:.1f}MiB")
        if samples[0]["cpuSeconds"] is not None and samples[-1]["cpuSeconds"] is not None:
            print(f"CPU consumed={samples[-1]['cpuSeconds'] - samples[0]['cpuSeconds']:.2f}s")
        if args.idle_seconds is not None:
            recent = next(row for row in samples if row["timeSeconds"] >= samples[-1]["timeSeconds"] - 5)
            print(f"final 5s RSS change={(samples[-1]['rssBytes'] - recent['rssBytes']) / 2**20:.1f}MiB "
                  f"CPU={samples[-1]['cpuSeconds'] - recent['cpuSeconds']:.2f}s")
    if export is not None and export.returncode == 0:
        events = json.loads((output / "editor.perfetto.json").read_text())["traceEvents"]
        counters = [event["args"] for event in events if event.get("cat") == "hl.gc" and
                    event.get("ph") == "C" and event.get("args")]
        if len(counters) > 1:
            first, last = counters[0], counters[-1]
            print(f"HashLink allocations={(last['allocated_bytes'] - first['allocated_bytes']) / 2**20:.1f}MiB "
                  f"collections={last['collections'] - first['collections']} "
                  f"GC mark={(last['mark_micros'] - first['mark_micros']) / 1000:.1f}ms "
                  f"heap change={(last['heap_bytes'] - first['heap_bytes']) / 2**20:.1f}MiB")
    if args.scenario == "tab-inspector":
        try:
            actions = [json.loads(line) for line in (output / "actions.jsonl").read_text().splitlines()]
            state = json.loads((output / "app-state.json").read_text())
            labels = {item["id"]: item["label"] for item in state["scene"]["objects"]}
            if labels.get("box") != "Profile box" or not any(row.get("action") == "rename" for row in actions):
                raise ValueError("tab and inspector scenario did not commit the expected rename")
            print("scenario=tab-inspector verified")
        except (OSError, KeyError, ValueError) as error:
            print(f"scenario verification failed: {error}", file=sys.stderr)
            result = 1
    print(f"capture={output}")
    if result:
        print(f"editor/profile exited with status {result}; see {output / 'launch.log'}", file=sys.stderr)
    return result


if __name__ == "__main__":
    sys.exit(main())
