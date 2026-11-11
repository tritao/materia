#!/usr/bin/env python3
"""Prints the Haxeon compiler arguments for the editor's browser guest, one per line.

The desktop build gets these from `haxeon build`, which resolves app/haxeon.json's
package graph. Native providers cannot target wasm32 yet, so the browser build walks
the same manifests here: every package's source roots and sources, the package roots
of scoped packages, and its FFI projections, with each FFI interface replaced by the wasm32
(portable-abi32) copy generated into HXI_DIR.

--omit=PACKAGE leaves a package out of the guest: its sources, its interfaces and its own
dependencies (unless something else reaches them). It is for native packages the browser build
cannot provide yet; the application must keep their uses behind `#if !wasm`.
"""

import json
import os
import sys


def main() -> int:
    options = [argument for argument in sys.argv[1:] if argument.startswith("--")]
    positional = [argument for argument in sys.argv[1:] if not argument.startswith("--")]
    omitted = {option[len("--omit="):] for option in options if option.startswith("--omit=")}
    if len(positional) != 2 or len(omitted) != len(options):
        print("usage: guest-arguments.py APP_MANIFEST HXI_DIR [--omit=PACKAGE ...]", file=sys.stderr)
        return 2
    manifest_path, hxi_dir = os.path.abspath(positional[0]), os.path.abspath(positional[1])
    unused_omissions = set(omitted)
    arguments: list[str] = []
    sources: list[str] = []
    visited: set[str] = set()

    def visit(directory: str) -> None:
        directory = os.path.realpath(directory)
        if directory in visited:
            return
        visited.add(directory)
        with open(os.path.join(directory, "haxeon.json"), encoding="utf-8") as handle:
            manifest = json.load(handle)
        name = manifest["package"]["name"]
        if name in omitted:
            unused_omissions.discard(name)
            return
        roots = [os.path.join(directory, root) for root in manifest.get("sourceRoots", ["src"])]
        arguments.extend("--root=" + root for root in roots)
        # Like the package resolver: every .hx file below the roots, unless the manifest lists sources.
        listed = manifest.get("sources")
        if listed:
            sources.extend(os.path.join(directory, source) for source in listed)
        else:
            for root in roots:
                for parent, _, files in os.walk(root):
                    sources.extend(os.path.join(parent, file) for file in files if file.endswith(".hx"))
        if manifest.get("scopeSourceRoots", True):
            arguments.extend(f"--package-root={name}={root}" for root in roots)
        ffi = manifest.get("ffi", {})
        for interface in ffi.get("interfaces", []):
            wasm_interface = os.path.join(hxi_dir, os.path.basename(interface))
            if not os.path.exists(wasm_interface):
                print(f"guest-arguments: no wasm32 interface for {interface} in {hxi_dir}", file=sys.stderr)
                sys.exit(1)
            arguments.append("--ffi-interface=" + wasm_interface)
        arguments.extend("--ffi-projection=" + os.path.join(directory, projection)
                         for projection in ffi.get("projections", []))
        if ffi.get("imports"):
            print(f"guest-arguments: {name} uses ffi.imports, which the browser build does not handle yet",
                  file=sys.stderr)
            sys.exit(1)
        for dependency in manifest.get("dependencies", {}).values():
            if "path" not in dependency:
                print(f"guest-arguments: {name} has a non-path dependency", file=sys.stderr)
                sys.exit(1)
            visit(os.path.join(directory, dependency["path"]))

    visit(os.path.dirname(manifest_path))
    if unused_omissions:
        print(f"guest-arguments: --omit names packages outside the graph: {', '.join(sorted(unused_omissions))}",
              file=sys.stderr)
        return 1
    print("\n".join(arguments + sorted(set(sources))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
