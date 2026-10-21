#!/usr/bin/env python3
"""Prints the hash of everything docs-runtime.wasm is built from: the runtime and kit sources, and the
compiler's sources at the pinned Haxeon commit. `inputs.py --check` fails when docs-runtime.inputs differs."""
import hashlib
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
ROOTS = ["docs/runtime/src", "cnckit/haxe", "toolpathkit/haxe", "robotkit/core/haxe", "projectkit/src", "trajectorykit/haxe",
         "collisionkit/haxe"]


def digest():
    hash = hashlib.sha256()
    for root in ROOTS:
        for parent, _, files in sorted(os.walk(os.path.join(REPO, root))):
            for name in sorted(files):
                if name.endswith(".hx"):
                    path = os.path.join(parent, name)
                    hash.update(os.path.relpath(path, REPO).encode() + b"\0")
                    with open(path, "rb") as handle:
                        hash.update(handle.read() + b"\0")
    # The compiler is Haxeon's src and stdlib at the pinned commit; its docs and tests do not change the output.
    pin = subprocess.run(["git", "-C", REPO, "rev-parse", "HEAD:haxeon"], capture_output=True, text=True, check=True).stdout.strip()
    for tree in ("src", "stdlib"):
        id = subprocess.run(["git", "-C", os.path.join(REPO, "haxeon"), "rev-parse", f"{pin}:{tree}"],
                            capture_output=True, text=True, check=True).stdout.strip()
        hash.update(f"haxeon {tree} {id}".encode())
    return hash.hexdigest()


if __name__ == "__main__":
    current = digest()
    if "--check" in sys.argv[1:]:
        with open(os.path.join(HERE, "docs-runtime.inputs"), encoding="utf-8") as handle:
            if handle.read().strip() != current:
                print("docs/runtime/docs-runtime.wasm is stale: run docs/runtime/build.sh", file=sys.stderr)
                sys.exit(1)
        print("docs-runtime.wasm is current")
    else:
        print(current)
