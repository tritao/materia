#!/usr/bin/env python3
"""Prints the hash of everything docs-runtime.wasm is built from: the runtime and kit sources, and the
Haxeon commit the compiler comes from. `inputs.py --check` fails when docs-runtime.inputs differs."""
import hashlib
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
ROOTS = ["docs/runtime/src", "cnckit/haxe", "toolpathkit/haxe"]


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
    haxeon = subprocess.run(["git", "-C", REPO, "rev-parse", "HEAD:haxeon"], capture_output=True, text=True, check=True)
    hash.update(b"haxeon " + haxeon.stdout.strip().encode())
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
