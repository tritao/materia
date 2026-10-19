#!/usr/bin/env python3
"""Profile an already-built reach study; stop our child before exhausting the desktop.

No builds, cache deletion, or changes to system OOM settings.
The default command is the bounded setup probe. Pass --search for a read-only
study or --store to regenerate the certificate after successful certification. JSONL samples contain OS memory; reach.log contains VM counters.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def fields(path):
    result = {}
    for line in Path(path).read_text().splitlines():
        parts = line.split()
        if len(parts) > 1 and parts[1].isdigit():
            result[parts[0].rstrip(":")] = int(parts[1]) * 1024
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--search", action="store_true")
    mode.add_argument("--store", action="store_true", help="Write the certificate only if the study succeeds")
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--max-rss-mib", type=float, default=2048)
    parser.add_argument("--min-available-mib", type=float, default=4096)
    parser.add_argument("--census", action="store_true")
    args = parser.parse_args()
    if min(args.seconds, args.max_rss_mib, args.min_available_mib) <= 0:
        parser.error("Limits must be positive")
    root = Path(__file__).resolve().parents[3]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    artifact = root / "machinekit/examples/machine-tending/build/host/main.hl"
    if not artifact.exists():
        parser.error("Build the reach project first")
    if fields("/proc/meminfo")["MemAvailable"] < args.min_available_mib * 1024**2:
        parser.error("Not enough available system memory for this probe")
    native = sorted(artifact.parent.glob("native/*"))
    environment = os.environ.copy()
    environment["LD_LIBRARY_PATH"] = ":".join(map(str, [root / "haxeon/out", root / "haxeon/.tools/hashlink", *native])) + ":" + environment.get("LD_LIBRARY_PATH", "")
    command = [str(root / "haxeon/.tools/hashlink/hl"), str(artifact), "--profile-memory"]
    command.append("--store" if args.store else "--profile-search" if args.search else "--profile-setup")
    if args.census:
        command.append("--profile-census=" + str(output / "allocations.json"))
    started = time.monotonic()
    reason = None
    peak = 0
    with (output / "reach.log").open("w") as log, (output / "memory.jsonl").open("w") as samples:
        child = subprocess.Popen(command, cwd=root, env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while child.poll() is None:
                elapsed = time.monotonic() - started
                try:
                    status = fields(f"/proc/{child.pid}/status")
                except FileNotFoundError:
                    break
                system = fields("/proc/meminfo")
                rss = status.get("VmRSS", 0)
                peak = max(peak, rss)
                samples.write(json.dumps({"seconds": elapsed, "rssBytes": rss,
                    "anonBytes": status.get("RssAnon", 0), "fileBytes": status.get("RssFile", 0),
                    "swapBytes": status.get("VmSwap", 0), "availableBytes": system["MemAvailable"]}) + "\n")
                samples.flush()
                if rss > args.max_rss_mib * 1024**2:
                    reason = "process RSS limit"
                elif system["MemAvailable"] < args.min_available_mib * 1024**2:
                    reason = "system available-memory reserve"
                elif elapsed >= args.seconds:
                    reason = "time limit"
                if reason:
                    try:
                        os.killpg(child.pid, signal.SIGINT)
                    except ProcessLookupError:
                        break
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                    break
                time.sleep(0.1)
        finally:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGKILL)
            code = child.wait()
    report = {"exitCode": code, "guardStop": reason, "peakRssBytes": peak,
        "seconds": time.monotonic() - started, "command": command}
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if code == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
