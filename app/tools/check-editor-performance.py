#!/usr/bin/env python3
"""Run or inspect bounded editor captures for sustained performance regressions."""

import argparse
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "app/tools/profile-editor.py"
HAXEON = ROOT / "haxeon/scripts/haxeon"
MIB = 2**20


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[int((len(ordered) - 1) * fraction)]


def nearest_rss(samples, timestamp):
    return min(samples, key=lambda row: abs(row["timeSeconds"] - timestamp))["rssBytes"]


def inspect_interaction(directory, cycles):
    frames = rows(directory / "frame-timeline.jsonl")
    actions = rows(directory / "actions.jsonl")
    retained = rows(directory / "retained.jsonl")
    memory = rows(directory / "memory.jsonl")
    state = json.loads((directory / "app-state.json").read_text())
    if len(frames) < cycles * 2 or len(retained) < 5 or len(memory) < 5:
        raise ValueError("interaction capture is incomplete")
    if sum(row.get("action") == "sensors" for row in actions) != cycles or \
            sum(row.get("action") == "hierarchy" for row in actions) != cycles:
        raise ValueError("interaction actions are incomplete")
    if not any(row.get("action") == "rename" for row in actions) or \
            not any(item.get("id") == "box" and item.get("label") == "Profile box"
                    for item in state["scene"]["objects"]):
        raise ValueError("inspector rename was not committed")
    if retained[-1]["cycle"] != cycles:
        raise ValueError("retained-state capture ended before the last cycle")

    warm = [row for row in retained if row["cycle"] >= 100]
    if len(warm) < 5:
        raise ValueError("at least 500 cycles are needed to measure sustained growth")
    checks = {
        "workspace listeners": lambda row: row["workspaceListeners"],
        "widget resources": lambda row: row["state"]["resources"],
        "widget state": lambda row: row["state"]["values"],
        "widget paths": lambda row: row["state"]["paths"],
        "style entries": lambda row: row["styles"]["styles"],
        "cached key paths": lambda row: row["keys"]["paths"],
    }
    failures = []
    for name, get_value in checks.items():
        values = [get_value(row) for row in warm]
        if max(values) != min(values):
            failures.append(f"{name} changed after warmup: {min(values)}..{max(values)}")
    if warm[-1]["workspaceListeners"] > 3:
        failures.append("workspace has more than three retained listeners")
    if warm[-1]["state"]["resources"] > 32:
        failures.append("more than 32 widget resources remain mounted")

    action_times = {row["cycle"] + 1: row["timeSeconds"] for row in actions
                    if row.get("action") == "hierarchy"}
    checkpoints = list(range(100, cycles + 1, 100))
    rss = {cycle: nearest_rss(memory, action_times[cycle]) for cycle in checkpoints}
    growth_mib = (rss[cycles] - rss[100]) / MIB
    if growth_mib > 32:
        failures.append(f"RSS grew {growth_mib:.1f} MiB after warmup (limit 32 MiB)")
    windows = [(rss[end] - rss[end - 100]) / MIB for end in checkpoints[1:]]
    if any(left > 8 and right > 8 for left, right in zip(windows, windows[1:])):
        failures.append("RSS grew more than 8 MiB per 100 cycles in consecutive windows")

    p95_ms = percentile([row["frameSeconds"] * 1000 for row in frames], .95)
    if p95_ms > 30:
        failures.append(f"frame p95 is {p95_ms:.1f} ms (limit 30 ms)")
    return {"capture": str(directory), "cycles": cycles, "frameP95Ms": round(p95_ms, 2),
            "rssAfterWarmupMiB": round(growth_mib, 2),
            "rssWindowGrowthMiB": [round(value, 2) for value in windows],
            "workspaceListeners": warm[-1]["workspaceListeners"], "failures": failures}


def inspect_idle(directory, seconds):
    memory = rows(directory / "memory.jsonl")
    if len(memory) < 20 or memory[-1]["timeSeconds"] - memory[0]["timeSeconds"] < seconds - 1:
        raise ValueError("idle capture is incomplete")
    start = memory[0]["timeSeconds"]
    windows = []
    for offset in range(5, seconds - 4, 5):
        before = min(memory, key=lambda row: abs(row["timeSeconds"] - (start + offset)))
        after = min(memory, key=lambda row: abs(row["timeSeconds"] - (start + offset + 5)))
        windows.append({"cpuSeconds": after["cpuSeconds"] - before["cpuSeconds"],
                        "rssMiB": (after["rssBytes"] - before["rssBytes"]) / MIB})
    failures = []
    if any(a["cpuSeconds"] > 1.0 and b["cpuSeconds"] > 1.0
           for a, b in zip(windows, windows[1:])):
        failures.append("idle CPU exceeded 1 second per 5 seconds in consecutive windows")
    if any(a["rssMiB"] > 16 and b["rssMiB"] > 16
           for a, b in zip(windows, windows[1:])):
        failures.append("idle RSS grew more than 16 MiB per 5 seconds in consecutive windows")
    if not windows:
        raise ValueError("idle capture is too short for post-startup windows")
    return {"capture": str(directory), "seconds": seconds,
            "maxIdleCpuPer5s": round(max(row["cpuSeconds"] for row in windows), 2),
            "maxIdleRssGrowthPer5sMiB": round(max(row["rssMiB"] for row in windows), 2),
            "failures": failures}


def run(command):
    subprocess.run(command, cwd=ROOT, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycles", type=int, default=500)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--idle-seconds", type=int, default=30)
    parser.add_argument("--headless-only", action="store_true",
                        help="skip the desktop idle check when no display server is available")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--capture", action="append", type=Path,
                        help="inspect an existing interaction capture; may be repeated")
    parser.add_argument("--idle-capture", type=Path, help="inspect an existing idle capture")
    args = parser.parse_args()
    if args.cycles < 500 or args.cycles % 100 or args.runs < 1 or args.idle_seconds < 15:
        parser.error("cycles must be a multiple of 100 and at least 500; runs >= 1; idle >= 15s")
    if not args.headless_only and not args.idle_capture and not args.capture and \
            not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error("desktop idle check requires a display; use --headless-only for CI")
    if args.capture:
        captures = [path.resolve() for path in args.capture]
        idle = args.idle_capture.resolve() if args.idle_capture else None
    else:
        root = (args.output_root or ROOT / "app/build/profiles" /
                time.strftime("gate-%Y%m%d-%H%M%S")).resolve()
        root.mkdir(parents=True, exist_ok=False)
        if not args.skip_build:
            if not args.headless_only:
                run([str(HAXEON), "build", "--project", str(ROOT / "app/haxeon.json")])
            run([str(HAXEON), "build", "--compiler-only", "--project",
                 str(ROOT / "app/tests/performance/haxeon.json"),
                 "--output", str(ROOT / "app/build/host/headless-profile.hl")])
        captures = []
        for index in range(args.runs):
            capture = root / f"interaction-{index + 1}"
            run([sys.executable, str(PROFILE), "--skip-build", "--no-profile",
                 "--scenario", "tab-inspector", "--cycles", str(args.cycles),
                 "--output-dir", str(capture)])
            captures.append(capture)
        idle = None
        if not args.headless_only:
            idle = root / "idle"
            run([sys.executable, str(PROFILE), "--skip-build", "--no-profile",
                 "--idle-seconds", str(args.idle_seconds), "--output-dir", str(idle)])
    results = [inspect_interaction(path, args.cycles) for path in captures]
    if idle is not None:
        results.append(inspect_idle(idle, args.idle_seconds))
    failures = [failure for result in results for failure in result["failures"]]
    for result in results:
        print(json.dumps(result, sort_keys=True))
    if failures:
        print(f"FAIL: {len(failures)} editor performance regression(s)", file=sys.stderr)
        return 1
    print("PASS: editor performance captures")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"editor performance check failed: {error}", file=sys.stderr)
        sys.exit(1)
