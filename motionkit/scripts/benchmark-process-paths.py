#!/usr/bin/env python3
"""Run compiled acceptance checks and retain their measured planning/quality records.

Build app/haxeon.project-source.json and robotkit/tests/haxeon.json first.
For wall-finishing-mujoco, also build robotkit/tests/mujoco/haxeon.json.
No historical values or estimates are substituted for missing measurements.
Set PROCESS_PATH_PROFILE=1 to retain structured planner phase timings too.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time


ROOT = Path(__file__).resolve().parents[2]
CASES = {
    "track-weld": ("app", "PROJECT_SOURCE_ONLY", "track-weld"),
    "welder": ("app", "PROJECT_SOURCE_ONLY", "welder"),
    "gantry-welder": ("app", "PROJECT_SOURCE_ONLY", "gantry-welder"),
    "wall-finishing": ("robotkit/tests", "ROBOTKIT_ONLY", "wall-finishing"),
    "wall-finishing-mujoco": ("robotkit/tests/mujoco", "ROBOTKIT_ONLY", "wall-finishing"),
    "handling": ("app", "PROJECT_SOURCE_ONLY", "arm"),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--app-module", default="process-path-baseline.hl")
    parser.add_argument("--robot-module", default="main.hl")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report = {"revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
              "haxeonRevision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT / "haxeon", text=True).strip(),
              "cases": []}
    failed = False
    for name in args.cases:
        project, selector, value = CASES[name]
        directory = ROOT / project
        env = os.environ.copy()
        for key in ("PROJECT_SOURCE_ONLY", "ROBOTKIT_ONLY", "MOTIONKIT_HOMING_ONLY"):
            env.pop(key, None)
        env.update({selector: value, "PROCESS_PATH_BENCHMARK": "1", "HAXEON_HOME": str(ROOT / "haxeon")})
        paths = []
        if name == "wall-finishing-mujoco":
            # This backend's robotd/runtime must precede the standard test
            # project's staged libraries. The compiler-only project shares
            # the already-built native workspace outputs.
            paths.append(ROOT / "build/workspace/host/cmake/robotkit-robotd-native-mujoco--robotd_native/out")
        paths.extend([ROOT / "haxeon/out", ROOT / "haxeon/.tools/hashlink"])
        for native in (directory / "build/host/native", ROOT / "robotkit/tests/build/host/native"):
            if native.exists():
                paths.extend(sorted({p.parent for pattern in ("*.hdll", "*.so", "*.so.*") for p in native.rglob(pattern)}))
        if env.get("LD_LIBRARY_PATH"):
            paths.extend(env["LD_LIBRARY_PATH"].split(":"))
        env["LD_LIBRARY_PATH"] = ":".join(map(str, dict.fromkeys(paths)))
        module = args.app_module if project == "app" else args.robot_module
        command = [str(ROOT / "haxeon/.tools/hashlink/hl"), str(directory / "build/host" / module)]
        started = time.monotonic()
        records = []
        log = args.output / (name + ".log")
        with log.open("w") as out:
            process = subprocess.Popen(command, cwd=ROOT / "robotkit/tests" if project.startswith("robotkit/tests") else ROOT,
                                       env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            for line in process.stdout:
                out.write(line)
                out.flush()
                if line.startswith(("PROCESS_PATH_RUN ", "PROCESS_PATH_QUALITY ", "PROCESS_PATH_PROFILE ")):
                    kind, data = line.split(" ", 1)
                    records.append({"kind": kind, **json.loads(data)})
                    print(name, line.rstrip(), flush=True)
            code = process.wait()
        missing = [kind for kind in ("PROCESS_PATH_RUN", "PROCESS_PATH_QUALITY")
                   if not any(record["kind"] == kind for record in records)]
        failed |= code != 0 or bool(missing)
        report["cases"].append({"name": name, "exitCode": code, "wallSeconds": time.monotonic() - started,
                                "log": str(log.resolve()), "missingRecords": missing, "records": records})
        (args.output / "results.json").write_text(json.dumps(report, indent=2) + "\n")
        print(f"{name}: exit={code}, records={len(records)}, missing={missing}, log={log}", flush=True)
        if code or missing:
            break
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
