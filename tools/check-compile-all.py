#!/usr/bin/env python3
"""Compile every Haxe target in the repository and report the ones that do not build.

A compiler change (a new rule, a changed default) can break code that nothing in the compiler's own tests compiles, and
the breakage only shows up when somebody builds that kit. This builds all of them:

  * every tracked haxeon.json project, with `haxeon build --compiler-only`, and
  * the commands listed in tools/compile-sweep.json, for targets that are built by a script and have no project file
    (the UIKit showcase, the simulation binding checks).

Examples:
  tools/check-compile-all.py                     # everything, two at a time
  tools/check-compile-all.py --only uikit        # targets whose name contains "uikit"
  tools/check-compile-all.py --list              # what would be built
  tools/check-compile-all.py --jobs 4 --json out.json
"""
import argparse
import concurrent.futures
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
HAXEON = ROOT / "haxeon/scripts/haxeon"
REGISTRY = ROOT / "tools/compile-sweep.json"
CODE = re.compile(r"\bE\d{4}\b")


def discover_projects():
    tracked = subprocess.run(["git", "ls-files", "*haxeon.json"], cwd=ROOT, capture_output=True, text=True, check=True).stdout
    return sorted(line for line in tracked.splitlines() if line.endswith("haxeon.json"))


def load_registry():
    if not REGISTRY.exists():
        return {"skip": {}, "commands": []}
    return json.loads(REGISTRY.read_text())


def targets(arguments):
    registry = load_registry()
    skipped = registry.get("skip", {})
    found = []
    for manifest in discover_projects():
        name = str(Path(manifest).parent) or "."
        if name in skipped:
            found.append({"name": name, "kind": "skipped", "reason": skipped[name]})
            continue
        if "entry" not in json.loads((ROOT / manifest).read_text()):
            found.append({"name": name, "kind": "skipped", "reason": "library package, built through the projects that depend on it"})
            continue
        found.append({"name": name, "kind": "project", "manifest": manifest})
    for command in registry.get("commands", []):
        found.append({"name": command["name"], "kind": "command", "command": command["command"],
                      "slow": command.get("slow", False)})
    if arguments.only:
        found = [target for target in found if any(text in target["name"] for text in arguments.only)]
    if arguments.fast:
        found = [target for target in found if not target.get("slow")]
    return found


def first_error(output):
    """The most telling line of a failed build: a diagnostic with a code, an uncaught exception, or the last error."""
    lines = [line.strip() for line in output.splitlines() if line.strip()]
    for line in lines:
        if CODE.search(line) or "Uncaught exception" in line:
            return line[:300]
    for line in reversed(lines):
        if "error" in line.lower() or "failed" in line.lower():
            return line[:300]
    return lines[-1][:300] if lines else "no output"


def build(target, arguments):
    started = time.monotonic()
    environment = dict(os.environ)
    if target["kind"] == "project":
        manifest = ROOT / target["manifest"]
        command = [str(HAXEON), "build", "--compiler-only", "--project", str(manifest), "--output",
                   str(Path(arguments.scratch) / (target["name"].replace("/", "_") + ".hl"))]
        cwd = manifest.parent
    else:
        command = target["command"]
        cwd = ROOT
    try:
        result = subprocess.run(command, cwd=cwd, env=environment, capture_output=True, text=True, timeout=arguments.timeout)
        output = result.stdout + result.stderr
        ok = result.returncode == 0
    except subprocess.TimeoutExpired:
        ok, output = False, f"timed out after {arguments.timeout}s"
    return {"name": target["name"], "ok": ok, "seconds": round(time.monotonic() - started, 1),
            "error": None if ok else first_error(output), "output": None if ok else output[-4000:]}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--jobs", type=int, default=2, help="targets built at once (each uses a compiler worker of ~1.5 GB)")
    parser.add_argument("--only", action="append", help="build only targets whose name contains this text; repeatable")
    parser.add_argument("--fast", action="store_true", help="skip the registry commands marked slow")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--timeout", type=int, default=1800)
    parser.add_argument("--json", type=Path, help="write the results here")
    parser.add_argument("--scratch", default="/tmp/materia-compile-sweep")
    parser.add_argument("--fail-fast", action="store_true")
    arguments = parser.parse_args()
    os.makedirs(arguments.scratch, exist_ok=True)

    found = targets(arguments)
    if arguments.list:
        for target in found:
            print(f"{target['kind']:8} {target['name']}" + (f"  ({target['reason']})" if target["kind"] == "skipped" else ""))
        return 0
    runnable = [target for target in found if target["kind"] != "skipped"]
    skipped = [target for target in found if target["kind"] == "skipped"]
    print(f"building {len(runnable)} targets, {len(skipped)} skipped, {arguments.jobs} at a time")
    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=arguments.jobs) as pool:
        futures = {pool.submit(build, target, arguments): target for target in runnable}
        for future in concurrent.futures.as_completed(futures):
            result = future.result()
            results.append(result)
            print(f"{'ok  ' if result['ok'] else 'FAIL'} {result['name']} ({result['seconds']}s)"
                  + ("" if result["ok"] else f"\n       {result['error']}"), flush=True)
            if arguments.fail_fast and not result["ok"]:
                for pending in futures:
                    pending.cancel()
                break
    failed = [result for result in results if not result["ok"]]
    if arguments.json:
        arguments.json.write_text(json.dumps({"results": results, "skipped": skipped}, indent=1) + "\n")
    if skipped:
        print(f"skipped {len(skipped)}: " + ", ".join(target["name"] for target in skipped[:6]) + (", ..." if len(skipped) > 6 else ""))
    print(f"\n{len(results) - len(failed)} of {len(results)} built" + (f"; {len(failed)} failed" if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
