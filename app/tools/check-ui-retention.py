#!/usr/bin/env python3
"""Check that the editor's retained UI matches a fresh build.

Runs headless editor scenarios with HAXEON_UI_VERIFY_RETENTION=1: after every frame that did work, UiContext builds the view
again from scratch, bypassing every retained cache, patch and in-place edit, and fails the run if the tree differs (see
UiContext.verifyRetention). A failure means some cache key, quiet state update or in-place edit leaves out something the UI
shows. Timings from these runs are meaningless: every frame is built twice.
"""

import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "app/tools/profile-editor.py"
SCENARIOS = ["interaction", "tab-matrix", "dock-drag", "inspector-edits", "inspector-flows", "panel-flows", "selection-stress", "tab-inspector",
    "primitives"]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", action="append", choices=SCENARIOS, help="scenario to check (repeatable; default all)")
    parser.add_argument("--cycles", type=int, default=4)
    parser.add_argument("--skip-build", action="store_true")
    args = parser.parse_args()
    scenarios = args.scenario or SCENARIOS
    environment = dict(os.environ, HAXEON_UI_VERIFY_RETENTION="1")
    failures = []
    skip_build = args.skip_build
    with tempfile.TemporaryDirectory(prefix="ui-retention-") as scratch:
        for scenario in scenarios:
            output = Path(scratch) / scenario
            command = [sys.executable, str(PROFILE), "--scenario", scenario, "--cycles", str(args.cycles), "--no-profile",
                "--output-dir", str(output)]
            if skip_build:
                command.append("--skip-build")
            result = subprocess.run(command, env=environment, capture_output=True, text=True)
            # The first run builds; the rest reuse it.
            skip_build = True
            log = output / "launch.log"
            text = log.read_text() if log.exists() else result.stdout + result.stderr
            mismatch = text.find("Retained UI differs")
            if result.returncode == 0:
                print(f"ok   {scenario}")
            elif mismatch >= 0:
                print(f"FAIL {scenario}\n" + "\n".join(text[mismatch:].splitlines()[:14]))
                failures.append(scenario)
            else:
                tail = "\n".join(text.strip().splitlines()[-3:])
                print(f"FAIL {scenario} (did not run to the end)\n{tail}")
                failures.append(scenario)
    if failures:
        print(f"FAIL: {len(failures)} scenario(s): {', '.join(failures)}")
        return 1
    print("PASS: retained UI matches a fresh build on every frame")
    return 0


if __name__ == "__main__":
    sys.exit(main())
