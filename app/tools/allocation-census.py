#!/usr/bin/env python3
"""Summarizes a census.json written by `profile-editor.py --scenario ... --census`.

Prints allocation per cycle by type (exact) and by allocating function (sampled, byte-weighted).
"""
import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("census", type=Path, help="census.json, or the capture directory holding it")
    parser.add_argument("--cycles", type=int, required=True, help="cycles the census covered (the tab-matrix skips cycle 0)")
    parser.add_argument("--top", type=int, default=25)
    parser.add_argument("--depth", type=int, default=1, help="stack frames to group by (default: allocating function)")
    parser.add_argument("--exclude", action="append", default=[], metavar="PREFIX",
                        help="drop sampled stacks whose top frame starts with PREFIX (e.g. tests. haxe.format.)")
    parser.add_argument("--json", action="store_true", help="print machine-readable output")
    args = parser.parse_args()
    path = args.census / "census.json" if args.census.is_dir() else args.census
    data = json.loads(path.read_text())
    cycles = max(args.cycles, 1)
    types = sorted(data["types"], key=lambda item: -item["bytes"])
    total_bytes = sum(item["bytes"] for item in types) or 1
    functions = defaultdict(lambda: [0, defaultdict(int)])
    for stack in data["stacks"]:
        if any(stack["frames"] and stack["frames"][0].startswith(prefix) for prefix in args.exclude):
            continue
        frames = stack["frames"][:args.depth] or ["(native)"]
        key = " < ".join(frames)
        functions[key][0] += stack["bytes"]
        functions[key][1][stack["type"]] += stack["bytes"]
    sampled = sum(entry[0] for entry in functions.values()) or 1
    if args.json:
        print(json.dumps({"cycles": cycles, "allocations": data["allocations"], "types": types,
                          "functions": {k: v[0] for k, v in functions.items()}}))
        return 0
    print(f"{data['allocations'] / cycles:,.0f} allocations, {total_bytes / cycles / 1024:,.0f} KiB per cycle")
    print("\nby type (exact)")
    print(f"{'type':44}{'count/cycle':>13}{'KiB/cycle':>11}{'share':>8}{'avg B':>8}")
    for item in types[:args.top]:
        print(f"{item['type'][:43]:44}{item['count'] / cycles:>13,.0f}{item['bytes'] / cycles / 1024:>11,.1f}"
              f"{100 * item['bytes'] / total_bytes:>7.1f}%{item['bytes'] / max(item['count'], 1):>8.0f}")
    print(f"\nby allocating function (sampled every {data['every']} bytes)")
    print(f"{'function':64}{'KiB/cycle':>11}{'share':>8}  main types")
    for key, (bytes_, by_type) in sorted(functions.items(), key=lambda item: -item[1][0])[:args.top]:
        main = ", ".join(f"{name} {100 * value / bytes_:.0f}%" for name, value in
                         sorted(by_type.items(), key=lambda item: -item[1])[:3])
        print(f"{key[:63]:64}{bytes_ / cycles / 1024:>11,.1f}{100 * bytes_ / sampled:>7.1f}%  {main}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
