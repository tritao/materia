#!/usr/bin/env python3
"""Compare two captures written by profile-editor.py.

  profile-compare.py BEFORE AFTER            # capture directories (or their baseline/ subdirectory)

Frames are grouped by action. A change is reported as real only when the difference in medians is larger than the noise
of the two runs (the spread of each run's frames, scaled by its size) and at least 3%; everything else is shown as noise,
so a rerun of an unchanged build reads as "no change" instead of a few percent either way.

The profile section compares each function's share of frame-submission samples. A share moves by chance too, so a
difference counts only when it exceeds two standard errors of the two binomial estimates.
"""
import argparse
import json
import math
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
# Two runs of the same build differ by up to ~8% in a frame's median (frequency scaling and scheduling shift a whole
# process, which the spread of frames inside a run cannot see), so a smaller change is not evidence of anything.
MIN_RELATIVE_CHANGE = 0.10
PHASES = (("frameSeconds", "frame", 1000.0, "ms"), ("treeAndStyleSeconds", "tree/style", 1000.0, "ms"),
          ("nativeLayoutSeconds", "native layout", 1000.0, "ms"), ("allocatedBytes", "alloc", 1.0 / 1024, "KiB"))


def load_frames(capture: Path):
    path = capture / "frame-timeline.jsonl"
    if not path.exists():
        raise SystemExit(f"{capture}: no frame-timeline.jsonl")
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def group(frames):
    groups = {}
    for frame in frames:
        groups.setdefault(frame.get("action") or "frames", []).append(frame)
    return groups


def values(frames, key, scale):
    return [frame[key] * scale for frame in frames if isinstance(frame.get(key), (int, float))]


def noise(samples):
    """Standard error of the median, from the interquartile range (about 1.25 * sigma / sqrt(n) for a median)."""
    if len(samples) < 4:
        return float("inf")
    ordered = sorted(samples)
    quartile = len(ordered) // 4
    sigma = (ordered[-quartile - 1] - ordered[quartile]) / 1.349
    return 1.2533 * sigma / math.sqrt(len(ordered))


def compare_series(before, after, minimum=None):
    """(median before, median after, change, relative, is_real)"""
    b, a = statistics.median(before), statistics.median(after)
    change = a - b
    combined = math.hypot(noise(before), noise(after))
    relative = change / b if b else 0.0
    real = abs(change) > 2.0 * combined and abs(relative) >= (MIN_RELATIVE_CHANGE if minimum is None else minimum)
    return b, a, change, relative, real


def compare_frames(before_dir: Path, after_dir: Path, minimum=None):
    before, after = group(load_frames(before_dir)), group(load_frames(after_dir))
    print(f"frames: {sum(map(len, before.values()))} before, {sum(map(len, after.values()))} after")
    print(f"  {'action':<22}{'metric':<15}{'before':>10}{'after':>10}{'change':>10}  verdict")
    for action in sorted(set(before) & set(after)):
        for key, label, scale, unit in PHASES:
            b, a = values(before[action], key, scale), values(after[action], key, scale)
            if len(b) < 2 or len(a) < 2:
                continue
            median_before, median_after, change, relative, real = compare_series(b, a, minimum)
            if median_before == 0 and median_after == 0:
                continue
            note = ("faster" if change < 0 else "slower") if real else "noise"
            print(f"  {action:<22}{label:<15}{median_before:>8.2f}{unit:>2}{median_after:>8.2f}{unit:>2}"
                  f"{100.0 * relative:>+9.1f}%  {note}")
    only = sorted(set(before) ^ set(after))
    if only:
        print(f"  (only in one capture: {', '.join(only)})")


def profile_shares(capture: Path):
    perfetto = capture / "editor.perfetto.json"
    if not perfetto.exists():
        return None
    result = subprocess.run([sys.executable, str(ROOT / "haxeon/scripts/hlprof-report.py"), str(perfetto),
                             "--within", "UiContext.submit", "--top", "400", "--json"], capture_output=True, text=True)
    if result.returncode:
        return None
    data = json.loads(result.stdout)
    kept = data["kept"]
    return kept, {name: count / kept for name, count in data["self"].items()} if kept else {}


def compare_profiles(before_dir: Path, after_dir: Path, top: int):
    before, after = profile_shares(before_dir), profile_shares(after_dir)
    if before is None or after is None or not before[0] or not after[0]:
        print("profile: a capture has no usable profile; skipped")
        return
    (count_before, share_before), (count_after, share_after) = before, after
    rows = []
    for name in set(share_before) | set(share_after):
        p, q = share_before.get(name, 0.0), share_after.get(name, 0.0)
        error = math.sqrt(p * (1 - p) / count_before + q * (1 - q) / count_after)
        rows.append((abs(q - p), name, p, q, error))
    rows.sort(reverse=True)
    print(f"\nprofile (share of frame-submission samples; {count_before} before, {count_after} after):")
    print("  (shares are relative: when one function gets cheaper, the others' shares grow; compare times for absolutes)")
    print(f"  {'function':<58}{'before':>8}{'after':>8}{'change':>9}  verdict")
    shown = 0
    for delta, name, p, q, error in rows:
        if shown >= top:
            break
        real = delta > 2.0 * error and delta >= 0.005
        if not real and shown >= top // 2:
            continue
        label = name if len(name) <= 56 else name[:55] + "…"
        note = ("smaller" if q < p else "larger") if real else "noise"
        print(f"  {label:<58}{100 * p:>7.1f}%{100 * q:>7.1f}%{100 * (q - p):>+8.1f}%  {note}")
        shown += 1


def resolve(path: str) -> Path:
    directory = Path(path)
    if (directory / "frame-timeline.jsonl").exists():
        return directory
    if (directory / "baseline" / "frame-timeline.jsonl").exists():
        return directory / "baseline"
    raise SystemExit(f"{directory}: not a capture directory")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("before")
    parser.add_argument("after")
    parser.add_argument("--top", type=int, default=14, help="profile rows to show")
    parser.add_argument("--no-profile", action="store_true")
    parser.add_argument("--min-change", type=float, default=MIN_RELATIVE_CHANGE * 100,
                        help="smallest change in percent reported as real (default %(default).0f)")
    arguments = parser.parse_args()
    before, after = Path(arguments.before), Path(arguments.after)
    frames_before, frames_after = resolve(arguments.before), resolve(arguments.after)
    compare_frames(frames_before, frames_after, arguments.min_change / 100.0)
    if not arguments.no_profile:
        compare_profiles(before, after, arguments.top)
    return 0


if __name__ == "__main__":
    sys.exit(main())
