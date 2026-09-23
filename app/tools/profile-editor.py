#!/usr/bin/env python3
"""Capture a bounded editor run with HashLink, frame, and RSS timelines."""

import argparse
import json
import os
from pathlib import Path
import socket
import statistics
import subprocess
import sys
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=240)
    parser.add_argument("--seconds", type=float, help="capture normal event-driven frames for this interval")
    parser.add_argument("--idle-seconds", type=float, help="sample an unbounded editor, then stop it")
    parser.add_argument("--no-profile", action="store_true", help="measure without profiler overhead")
    parser.add_argument("--scenario", choices=["tab-inspector"], help="replay a headless UI interaction")
    parser.add_argument("--cycles", type=int, default=20, help="headless Hierarchy/Sensors cycles (default: 20)")
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
    if args.scenario is not None and (args.seconds is not None or args.idle_seconds is not None):
        parser.error("--scenario runs headlessly and cannot use --seconds or --idle-seconds")
    if args.cycles < 1:
        parser.error("--cycles must be positive")
    if args.scenario is None and args.cycles != 20:
        parser.error("--cycles requires --scenario")
    if args.scenario is not None and args.editor_args:
        parser.error("editor arguments after -- are not supported by headless scenarios")
    output = (args.output_dir or APP / "build/profiles" / time.strftime("%Y%m%d-%H%M%S")).resolve()
    output.mkdir(parents=True, exist_ok=False)
    editor_args = args.editor_args[1:] if args.editor_args[:1] == ["--"] else args.editor_args
    runtime = ROOT / "haxeon/.tools/hashlink/hl"
    profiler = ROOT / "haxeon/.tools/hashlink/hlprof-live"
    with (output / "launch.log").open("w") as log, (output / "memory.jsonl").open("w") as memory:
        headless_binary = APP / "build/host/headless-profile.hl"
        if args.skip_build:
            binary = headless_binary if args.scenario is not None else APP / "build/host/main.hl"
            if not binary.exists():
                parser.error(f"--skip-build requires an existing {binary}")
            log.write(f"Reusing existing {binary}; source changes are not included.\n")
        else:
            build = subprocess.run([str(HAXEON), "build", "--project", str(APP / "haxeon.json")],
                                   cwd=APP, stdout=log, stderr=subprocess.STDOUT)
            if build.returncode:
                print(f"build failed; see {output / 'launch.log'}", file=sys.stderr)
                return build.returncode
            if args.scenario is not None:
                build = subprocess.run([str(HAXEON), "build", "--compiler-only",
                                        "--project", str(APP / "tests/performance/haxeon.json"),
                                        "--output", str(headless_binary)],
                                       cwd=APP, stdout=log, stderr=subprocess.STDOUT)
                if build.returncode:
                    print(f"headless build failed; see {output / 'launch.log'}", file=sys.stderr)
                    return build.returncode
        native_dirs = sorted({str(path.parent) for path in (APP / "build/host/native").rglob("*.so")})
        environment = os.environ.copy()
        environment["LD_LIBRARY_PATH"] = os.pathsep.join(
            [str(runtime.parent), str(ROOT / "haxeon/out"), *native_dirs,
             environment.get("LD_LIBRARY_PATH", "")])
        if args.scenario is not None:
            environment.pop("DISPLAY", None)
            environment.pop("WAYLAND_DISPLAY", None)
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            port = reservation.getsockname()[1]
        command = [str(runtime)]
        if not args.no_profile:
            command += ["--diagnostics", str(port), "--diagnostics-wait"]
        command += [str(headless_binary if args.scenario is not None else APP / "build/host/main.hl")]
        if args.scenario is not None:
            command += [str(output), str(args.cycles)]
        elif args.idle_seconds is None:
            command += ["--capture-dir=" + str(output)]
            command += (["--capture-seconds=" + str(args.seconds)] if args.seconds is not None
                        else ["--frames=" + str(args.frames)])
        command += editor_args
        process = subprocess.Popen(command, cwd=APP, env=environment,
                                   stdout=log, stderr=subprocess.STDOUT)
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
        if args.scenario is not None:
            for action in ("sensors", "hierarchy", "inspector-focus", "rename"):
                action_frames = [frame for frame in frames if frame.get("action") == action]
                if not action_frames:
                    continue
                median = statistics.median(frame["frameSeconds"] * 1000 for frame in action_frames)
                tree = statistics.median(frame["treeAndStyleSeconds"] * 1000 for frame in action_frames)
                nodes = statistics.median(frame["nodeCount"] for frame in action_frames)
                misses = statistics.median(frame["styleCacheMisses"] for frame in action_frames)
                print(f"{action}: frames={len(action_frames)} median={median:.1f}ms "
                      f"tree/style={tree:.1f}ms nodes={nodes:.0f} cache misses={misses:.0f}")
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
            tab_counts = {name: sum(row.get("action") == name for row in actions)
                          for name in ("hierarchy", "sensors")}
            if (labels.get("box") != "Profile box" or
                    any(count != args.cycles for count in tab_counts.values()) or
                    not any(row.get("action") == "rename" for row in actions)):
                raise ValueError("tab and inspector scenario did not commit the expected rename")
            print(f"scenario=tab-inspector verified cycles={args.cycles}")
        except (OSError, KeyError, ValueError) as error:
            print(f"scenario verification failed: {error}", file=sys.stderr)
            result = 1
    print(f"capture={output}")
    if result:
        print(f"editor/profile exited with status {result}; see {output / 'launch.log'}", file=sys.stderr)
    return result


if __name__ == "__main__":
    sys.exit(main())
