#!/usr/bin/env python3
"""Check that an incremental build produces what a clean build of the same source produces.

The compiler keeps a session between builds and retypes only what an edit changed. Anything that leaks history into the
output (a counter shared by the whole run, a cache entry that outlives the code it was made for, a span taken from whichever
call site was typed first) makes the two differ, and nothing else notices: both programs run. This builds one project three
ways for each edit:

  1. cold, in a fresh compiler worker (the baseline for the edit's starting source),
  2. the same worker again after the edit, which is incremental,
  3. cold, in another fresh worker, from the edited source,

and requires 2 and 3 to be byte-identical. When they are not, `hldump` (vendor/hashlink/tools) says what differs.

Two edits are tried: a line inserted at the top of a file, which moves every source offset in it, and a longer string literal
inside one function, which moves what follows it along its line. Either one, not retyping something it should, shows up as
a difference in the output.

  tools/check-determinism.py
  tools/check-determinism.py --edit shift --project app/tests/performance/haxeon.json --file haxeon/packages/ui/haxe/haxeon/ui/core/BuildContext.hx
"""
import argparse
import os
from pathlib import Path
import random
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
HAXEON = ROOT / "haxeon/scripts/haxeon"


def find_hldump(explicit=None):
    if explicit or os.environ.get("HLDUMP"):
        return Path(explicit or os.environ["HLDUMP"])
    for candidate in (ROOT / "haxeon/out/cmake/release/hashlink/bin/hldump", ROOT / "haxeon/.tools/hashlink/hldump"):
        if candidate.exists():
            return candidate
    return None


def build(project, output, port):
    """One build in the compiler worker selected by `port` (a distinct port is a distinct worker, so a cold one)."""
    environment = dict(os.environ)
    environment["HAXEON_COMPILER_PROFILE_PORT"] = str(port)
    started = time.monotonic()
    result = subprocess.run([str(HAXEON), "build", "--compiler-only", "--project", str(project), "--output", str(output)],
                            cwd=project.parent, env=environment, capture_output=True, text=True)
    if result.returncode:
        raise SystemExit(f"build failed for {project}:\n{(result.stdout + result.stderr)[-2000:]}")
    return time.monotonic() - started


def shift_edit(text):
    """A line inserted after the first one: every offset below it moves."""
    head, _, rest = text.partition("\n")
    return head + "\n// determinism check: a line inserted above everything\n" + rest


def literal_edit(text):
    """The first string literal of some length made longer: only what follows it moves."""
    match = re.search(r'"([A-Za-z][A-Za-z ,.]{20,})"', text)
    if not match:
        raise SystemExit("no string literal to lengthen in the edited file")
    return text[:match.end() - 1] + " (lengthened by the determinism check)" + text[match.end() - 1:]


EDITS = {"shift": shift_edit, "literal": literal_edit}


def dump(hldump, binary, *options):
    return subprocess.run([str(hldump), *options, str(binary)], capture_output=True, text=True).stdout


def explain(hldump, incremental, cold):
    """What differs when the bytes do: decoded, so a difference in indexes alone does not hide a real one."""
    if hldump is None:
        return ["hldump is not built (cmake --build --preset release --target hldump in haxeon); only the bytes were compared"]
    lines = []
    body = lambda binary: [line for line in dump(hldump, binary).splitlines() if not line.startswith("section")]
    identities = lambda binary: [line for line in dump(hldump, binary, "--raw", "--identities").splitlines() if line.startswith("identity")]
    spans = lambda binary: [line for line in dump(hldump, binary, "--spans").splitlines() if not line.startswith("section")]
    if body(incremental) != body(cold):
        first = next(((a, b) for a, b in zip(body(incremental), body(cold)) if a != b), None)
        lines.append("code, types, globals or strings differ" + (f"; first difference:\n    incremental: {first[0][:160]}\n    cold:        {first[1][:160]}" if first else ""))
    if identities(incremental) != identities(cold):
        lines.append("function identities differ (stable ids or source spans of generated functions)")
    if spans(incremental) != spans(cold):
        lines.append("opcode source spans differ (including the source hash they refer to)")
    return lines or ["the decoded code, identities and spans are equal: the difference is in a section this tool does not decode"]


def check(project, target, edit, hldump, scratch):
    original = target.read_text()
    port_a, port_b = random.randint(40000, 59000), random.randint(40000, 59000)
    outputs = {name: scratch / f"{edit}-{name}.hl" for name in ("start", "incremental", "cold")}
    try:
        seconds = build(project, outputs["start"], port_a)
        print(f"[{edit}] cold build of the starting source (worker {port_a}): {seconds:.0f}s")
        target.write_text(EDITS[edit](original))
        incremental = build(project, outputs["incremental"], port_a)
        print(f"[{edit}] incremental build after the edit: {incremental:.0f}s")
        seconds = build(project, outputs["cold"], port_b)
        print(f"[{edit}] cold build of the edited source (worker {port_b}): {seconds:.0f}s")
    finally:
        target.write_text(original)
    a, b = outputs["incremental"].read_bytes(), outputs["cold"].read_bytes()
    if a == b:
        print(f"[{edit}] PASS: incremental and cold builds are byte-identical ({len(a)} bytes)")
        return True
    print(f"[{edit}] DIFFERENT: incremental {len(a)} bytes, cold {len(b)} bytes")
    for line in explain(hldump, outputs["incremental"], outputs["cold"]):
        print("  " + line)
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--project", type=Path, default=ROOT / "app/tests/performance/haxeon.json")
    parser.add_argument("--file", type=Path, default=ROOT / "haxeon/packages/ui/haxe/haxeon/ui/core/BuildContext.hx",
                        help="the source file the edits are made to (restored afterwards)")
    parser.add_argument("--edit", choices=["shift", "literal", "all"], default="all")
    parser.add_argument("--hldump", type=Path, help="the hldump binary (default: $HLDUMP, or the one built in haxeon)")
    parser.add_argument("--scratch", type=Path, default=Path("/tmp/materia-determinism"))
    arguments = parser.parse_args()
    arguments.scratch.mkdir(parents=True, exist_ok=True)
    project, target = arguments.project.resolve(), arguments.file.resolve()
    hldump = find_hldump(arguments.hldump)
    results = [check(project, target, edit, hldump, arguments.scratch)
               for edit in (["shift", "literal"] if arguments.edit == "all" else [arguments.edit])]
    if not all(results):
        print("FAIL: an incremental build differs from a clean build of the same source", file=sys.stderr)
        return 1
    print("PASS: incremental builds match clean builds")
    return 0


if __name__ == "__main__":
    sys.exit(main())
