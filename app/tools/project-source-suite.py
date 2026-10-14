#!/usr/bin/env python3
"""Runs the project-source suite with one HashLink process per case, several at a time.

Cases come from the suite itself (`PROJECT_SOURCE_LIST=1`). Each runs as `PROJECT_SOURCE_ONLY=<case>` with its
own log. The longest cases start first, from the times recorded by earlier runs, and a case starts only while the
machine keeps its recorded peak memory plus a margin free, counting what the running cases may still grow to.
Prints a table of wall time and peak memory per case and exits non-zero if any case failed.

The caller provides the environment the suite needs (LD_LIBRARY_PATH, CADKIT_OCCT_DIR, ...).
"""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

REPORT = "project-source case "
DEFAULT_PEAK_MB = 2048


def mem_available_mb() -> int:
    with open("/proc/meminfo") as meminfo:
        for line in meminfo:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) // 1024
    raise RuntimeError("no MemAvailable in /proc/meminfo")


def resident_mb(pid: int) -> int:
    try:
        with open(f"/proc/{pid}/status") as status:
            for line in status:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) // 1024
    except OSError:
        pass
    return 0


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hl", type=Path, default=root / "haxeon/.tools/hashlink/hl")
    parser.add_argument("--suite", type=Path, default=root / "app/build/host/project-source.hl")
    parser.add_argument("--jobs", type=int, default=max(1, min(6, (os.cpu_count() or 4) // 3)))
    parser.add_argument("--only", help="comma-separated cases or groups instead of the default suite")
    parser.add_argument("--skip", default="", help="comma-separated cases to leave out")
    parser.add_argument("--margin-mb", type=int, default=3072,
                        help="memory left free besides the cases' expected peaks (earlyoom acts near 3 GiB)")
    parser.add_argument("--log-dir", type=Path, default=root / "app/build/host/project-source-logs")
    parser.add_argument("--timings", type=Path, default=root / "app/build/host/project-source-timings.json",
                        help="seconds and peak MB per case, read for scheduling and updated after the run")
    parser.add_argument("--stop-on-failure", action="store_true", help="start no new case after a failure")
    args = parser.parse_args()

    listing = subprocess.run([str(args.hl), str(args.suite)], env={**os.environ, "PROJECT_SOURCE_LIST": "1"},
                             capture_output=True, text=True)
    if listing.returncode != 0:
        sys.stderr.write(listing.stdout + listing.stderr)
        return 2
    known = [line for line in listing.stdout.splitlines() if re.fullmatch(r"[a-z0-9-]+", line)]
    cases = [name.strip() for name in args.only.split(",")] if args.only else known
    skipped = {name.strip() for name in args.skip.split(",") if name.strip()}
    cases = [name for name in cases if name not in skipped]
    timings = json.loads(args.timings.read_text()) if args.timings.exists() else {}
    # Longest first, so the slowest case does not start last; unmeasured cases count as slowest.
    pending = sorted(cases, key=lambda name: -timings.get(name, {}).get("seconds", 1e9))
    args.log_dir.mkdir(parents=True, exist_ok=True)

    running = {}  # pid -> (name, process, started, expected peak MB, log file)
    results = {}
    failed = False
    started_at = time.time()

    def expected_peak(name: str) -> int:
        return int(timings.get(name, {}).get("peak_mb", DEFAULT_PEAK_MB) * 1.15) + 64

    def stop_all(*_):
        for _, process, *_ in running.values():
            process.send_signal(signal.SIGTERM)
        sys.exit(130)

    signal.signal(signal.SIGINT, stop_all)
    signal.signal(signal.SIGTERM, stop_all)

    while pending or running:
        launched = False
        if pending and len(running) < args.jobs and not (failed and args.stop_on_failure):
            name = pending[0]
            headroom = sum(max(0, peak - resident_mb(pid)) for pid, (_, _, _, peak, _) in running.items())
            need = expected_peak(name)
            # A case too big for the machine still runs alone rather than never.
            if not running or mem_available_mb() - headroom >= need + args.margin_mb:
                pending.pop(0)
                log = open(args.log_dir / f"{name}.log", "w")
                process = subprocess.Popen([str(args.hl), str(args.suite)], stdout=log, stderr=subprocess.STDOUT,
                                           env={**os.environ, "PROJECT_SOURCE_ONLY": name}, cwd=root)
                running[process.pid] = (name, process, time.time(), need, log)
                launched = True
        if launched:
            continue
        pid, status, usage = os.wait4(-1, os.WNOHANG) if running else (0, 0, None)
        if pid == 0:
            time.sleep(0.5)
            continue
        name, process, started, _, log = running.pop(pid)
        log.close()
        code = os.waitstatus_to_exitcode(status)
        seconds = time.time() - started
        peak_mb = usage.ru_maxrss // 1024
        results[name] = {"seconds": round(seconds, 1), "peak_mb": peak_mb, "code": code}
        if code == 0:
            timings[name] = {"seconds": round(seconds, 1), "peak_mb": peak_mb}
        else:
            failed = True
        print(f"{'ok  ' if code == 0 else 'FAIL'} {name:32s} {seconds:7.1f} s {peak_mb:6d} MB"
              + ("" if code == 0 else f"  exit {code}, log {args.log_dir / (name + '.log')}"), flush=True)

    args.timings.parent.mkdir(parents=True, exist_ok=True)
    args.timings.write_text(json.dumps(dict(sorted(timings.items())), indent=1) + "\n")
    wall = time.time() - started_at
    serial = sum(result["seconds"] for result in results.values())
    print(f"\n{len(results)} cases in {wall:.0f} s wall ({serial:.0f} s serial, {args.jobs} jobs); "
          f"largest peak {max((r['peak_mb'] for r in results.values()), default=0)} MB")
    for name, result in sorted(results.items(), key=lambda item: -item[1]["seconds"]):
        print(f"  {name:32s} {result['seconds']:7.1f} s {result['peak_mb']:6d} MB"
              + ("" if result["code"] == 0 else "  FAILED"))
    for name, result in results.items():
        if result["code"] != 0:
            tail = (args.log_dir / f"{name}.log").read_text(errors="replace").splitlines()[-15:]
            print(f"\n--- {name} (exit {result['code']}) ---\n" + "\n".join(tail))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
