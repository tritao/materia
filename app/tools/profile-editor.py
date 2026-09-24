#!/usr/bin/env python3
"""Capture a bounded editor run with HashLink, frame, and RSS timelines."""

import argparse
import json
import os
from pathlib import Path
import shutil
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
    parser.add_argument("--sample-rate", type=int, help="profiler samples per second")
    parser.add_argument("--allocation-interval", type=int, help="allocation sampling interval in bytes; 0 disables it")
    parser.add_argument("--heap-dump", action="store_true",
                        help="save a full GC heap dump and its exact bytecode (headless scenario only)")
    parser.add_argument("--scenario", choices=["tab-inspector", "tab-matrix"],
                        help="replay a headless UI interaction")
    parser.add_argument("--cycles", type=int, default=20, help="headless scenario cycles (default: 20)")
    parser.add_argument("--skip-build", action="store_true", help="reuse the compiled editor; still ensure the Release HashLink runtime")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("editor_args", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be positive")
    if args.sample_rate is not None and args.sample_rate < 1:
        parser.error("--sample-rate must be positive")
    if args.allocation_interval is not None and args.allocation_interval < 0:
        parser.error("--allocation-interval must be nonnegative")
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
    if args.heap_dump and args.scenario is None:
        parser.error("--heap-dump requires a headless scenario")
    output = (args.output_dir or APP / "build/profiles" / time.strftime("%Y%m%d-%H%M%S")).resolve()
    output.mkdir(parents=True, exist_ok=False)
    editor_args = args.editor_args[1:] if args.editor_args[:1] == ["--"] else args.editor_args
    runtime = ROOT / "haxeon/.tools/hashlink/hl"
    profiler = ROOT / "haxeon/.tools/hashlink/hlprof-live"
    with (output / "launch.log").open("w") as log, (output / "memory.jsonl").open("w") as memory:
        configure = subprocess.run(["cmake", "--preset", "release", "-S", str(ROOT / "haxeon")],
                                   cwd=ROOT / "haxeon", stdout=log, stderr=subprocess.STDOUT)
        if configure.returncode:
            print(f"release runtime configure failed; see {output / 'launch.log'}", file=sys.stderr)
            return configure.returncode
        native = subprocess.run(["cmake", "--build", "--preset", "release"],
                                cwd=ROOT / "haxeon", stdout=log, stderr=subprocess.STDOUT)
        if native.returncode:
            print(f"release runtime build failed; see {output / 'launch.log'}", file=sys.stderr)
            return native.returncode
        headless_binary = APP / "build/host/headless-profile.hl"
        binary = headless_binary if args.scenario is not None else APP / "build/host/main.hl"
        if args.skip_build:
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
        if args.scenario == "tab-matrix" and not args.no_profile:
            environment["HAXEON_PROFILE_SPANS"] = "1"
        environment["LD_LIBRARY_PATH"] = os.pathsep.join(
            [str(runtime.parent), str(ROOT / "haxeon/out"), *native_dirs,
             environment.get("LD_LIBRARY_PATH", "")])
        bytecode_name = "headless-profile.hl" if args.scenario is not None else "main.hl"
        shutil.copy2(binary, output / bytecode_name)
        if args.scenario is not None:
            environment.pop("DISPLAY", None)
            environment.pop("WAYLAND_DISPLAY", None)
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            port = reservation.getsockname()[1]
        command = [str(runtime)]
        if not args.no_profile:
            command += ["--diagnostics", str(port), "--diagnostics-wait"]
        command += [str(binary)]
        if args.scenario is not None:
            command += [str(output), str(args.cycles), args.scenario]
            if args.heap_dump:
                command.append(str(output / "heap.dump"))
        elif args.idle_seconds is None:
            command += ["--capture-dir=" + str(output)]
            command += (["--capture-seconds=" + str(args.seconds)] if args.seconds is not None
                        else ["--frames=" + str(args.frames)])
        command += editor_args
        process = subprocess.Popen(command, cwd=APP, env=environment,
                                   stdout=log, stderr=subprocess.STDOUT)
        with (output / "profiler.log").open("w") as profile_log:
            sample_rate = args.sample_rate or (50 if args.scenario == "tab-matrix" else 500)
            allocation_interval = (args.allocation_interval if args.allocation_interval is not None else
                                   0 if args.scenario == "tab-matrix" else 65536)
            capture = None if args.no_profile else subprocess.Popen(
                [str(profiler), "--connect-timeout", "15", "--rate", str(sample_rate),
                 "--alloc-interval", str(allocation_interval), "--interval",
                 "20" if args.scenario == "tab-matrix" else "2000",
                 "--output", str(output / "editor.hlpc"), str(port)],
                cwd=APP, stdout=profile_log, stderr=subprocess.STDOUT)
            try:
                run_timeout = (args.idle_seconds if args.idle_seconds is not None else
                               args.seconds + 30 if args.seconds is not None else
                               max(60, args.cycles * 0.5 + 30) if args.scenario is not None else 60)
                deadline = time.monotonic() + run_timeout
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
                        raise TimeoutError(f"editor did not finish within {run_timeout:g} seconds")
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
    retained_file = output / "retained.jsonl"
    retained = [json.loads(line) for line in retained_file.read_text().splitlines() if line.strip()] if retained_file.exists() else []
    samples = [json.loads(line) for line in (output / "memory.jsonl").read_text().splitlines()]
    export = None if args.no_profile else subprocess.run(
        [str(profiler), "export", "--format", "perfetto",
         "--output", str(output / "editor.perfetto.json"),
         str(output / "editor.hlpc")], capture_output=True, text=True)
    if export is not None and export.returncode:
        print(f"Perfetto export failed: {export.stderr.strip()}", file=sys.stderr)
    if args.scenario == "tab-matrix" and export is not None and export.returncode == 0:
        span_report = subprocess.run(
            [sys.executable, str(ROOT / "haxeon/scripts/hlprof-spans.py"),
             "--min-ms", "20", "--top", "20", "--output", str(output / "span-spikes.json"),
             str(output / "editor.perfetto.json")], capture_output=True, text=True)
        if span_report.returncode:
            print(f"Span report failed: {span_report.stderr.strip()}", file=sys.stderr)
            result = 1
        else:
            print(span_report.stdout, end="")
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
            if args.scenario == "tab-matrix":
                transitions = [frame for frame in frames if "->" in frame.get("action", "")]
                def latency_ms(frame):
                    return (frame["frameSeconds"] + frame["inputSeconds"]) * 1000
                for action in sorted({frame["action"] for frame in transitions}):
                    group = [frame for frame in transitions if frame["action"] == action]
                    steady = [frame for frame in group if frame["cycle"] > 0] or group
                    durations = sorted(latency_ms(frame) for frame in steady)
                    p95 = durations[int((len(durations) - 1) * .95)]
                    allocations = statistics.median(frame["allocatedBytes"] for frame in steady)
                    collections = sum(frame["gcCollections"] for frame in steady)
                    print(f"{action}: cycles={len(group)} steady median={statistics.median(durations):.1f}ms "
                          f"p95={p95:.1f}ms max={durations[-1]:.1f}ms "
                          f"median alloc={allocations / 2**20:.2f}MiB GC={collections:.0f}")
                spikes = [{"transition": frame["action"], "cycle": frame["cycle"],
                           "latencyMs": latency_ms(frame),
                           "inputMs": frame["inputSeconds"] * 1000,
                           "inputPhasesMs": {name: seconds * 1000 for name, seconds in
                                             (frame.get("inputPhases") or {}).items()
                                             if isinstance(seconds, (int, float))},
                           "pointerUpPhasesMs": {name: seconds * 1000 for name, seconds in
                                                 ((frame.get("inputPhases") or {}).get("eventPhases") or {}).items()},
                           "inputGcCollections": frame.get("inputGcCollections"),
                           "inputGcMarkMicros": frame.get("inputGcMarkMicros"),
                           "submitGcCollections": frame.get("submitGcCollections"),
                           "submitGcMarkMicros": frame.get("submitGcMarkMicros"),
                           "frameMs": frame["frameSeconds"] * 1000,
                           "treeAndStyleMs": frame["treeAndStyleSeconds"] * 1000,
                           "allocatedBytes": frame["allocatedBytes"],
                           "gcCollections": frame["gcCollections"],
                           "gcMarkMicros": frame["gcMarkMicros"],
                           "styleCacheMisses": frame["styleCacheMisses"]}
                          for frame in sorted(transitions, key=latency_ms, reverse=True)[:20]]
                (output / "tab-spikes.json").write_text(json.dumps(spikes, indent=2) + "\n")
                for spike in spikes[:5]:
                    phases = spike["pointerUpPhasesMs"] or spike["inputPhasesMs"]
                    phase_text = (" ".join(f"{name}={seconds:.1f}ms" for name, seconds in phases.items())
                                  if phases else "input phases unavailable")
                    print(f"spike {spike['transition']} cycle={spike['cycle']} "
                          f"latency={spike['latencyMs']:.1f}ms "
                          f"input={spike['inputMs']:.1f}ms {phase_text} "
                          f"tree/style={spike['treeAndStyleMs']:.1f}ms "
                          f"alloc={spike['allocatedBytes'] / 2**20:.2f}MiB "
                          f"GC={spike['gcCollections']:.0f} "
                          f"input GC={spike['inputGcCollections']} "
                          f"cache misses={spike['styleCacheMisses']}")
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
    if retained:
        first, last = retained[0], retained[-1]
        print(f"retained cycles={first['cycle']}..{last['cycle']} "
              f"workspace listeners={first['workspaceListeners']}..{last['workspaceListeners']} "
              f"widget resources={first['state']['resources']}..{last['state']['resources']} "
              f"style entries={first['styles']['styles']}..{last['styles']['styles']}")
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
    elif args.scenario == "tab-matrix":
        try:
            actions = [json.loads(line) for line in (output / "actions.jsonl").read_text().splitlines()]
            groups = (("hierarchy", "sensors"),
                      ("viewport", "perspective", "console", "telemetry"))
            expected = {f"{source}->{target}" for group in groups
                        for source in group for target in group if source != target}
            counts = {name: sum(row.get("action") == name for row in actions) for name in expected}
            if any(count != args.cycles for count in counts.values()) or len(actions) != len(expected) * args.cycles:
                raise ValueError("tab matrix did not complete every ordered transition")
            measured = [frame for frame in frames if frame.get("action") in expected]
            frame_counts = {name: sum(frame["action"] == name for frame in measured) for name in expected}
            if (any(count != args.cycles for count in frame_counts.values()) or
                    any(frame["allocatedBytes"] is None or frame["allocatedBytes"] < 0 or
                        frame["gcCollections"] is None or frame["gcCollections"] < 0 or
                        frame["gcMarkMicros"] is None or frame["gcMarkMicros"] < 0
                        for frame in measured)):
                raise ValueError("tab matrix has missing or invalid frame/GC measurements")
            print(f"scenario=tab-matrix verified transitions={len(expected)} cycles={args.cycles}")
        except (OSError, KeyError, ValueError) as error:
            print(f"scenario verification failed: {error}", file=sys.stderr)
            result = 1
    artifacts = {"bytecode": bytecode_name}
    for name, filename in (("heap", "heap.dump"), ("profile", "editor.hlpc"),
                           ("perfetto", "editor.perfetto.json"), ("memory", "memory.jsonl"),
                           ("frames", "frame-timeline.jsonl"), ("retained", "retained.jsonl"),
                           ("spikes", "tab-spikes.json")):
        if (output / filename).is_file():
            artifacts[name] = filename
    (output / "capture.json").write_text(json.dumps({"schemaVersion": 1,
                                                       "kind": "haxeon.capture",
                                                       "artifacts": artifacts,
                                                       "workload": {"scenario": args.scenario,
                                                                    "cycles": args.cycles if args.scenario else None}},
                                                      indent=2) + "\n")
    print(f"capture={output}")
    if result:
        print(f"editor/profile exited with status {result}; see {output / 'launch.log'}", file=sys.stderr)
    return result


if __name__ == "__main__":
    sys.exit(main())
