#!/usr/bin/env python3
"""Generates the API reference data: one JSON file per kit in docs/api/data/.

Walks the editor's package graph (app/haxeon.json) like the browser build does, groups the
packages by kit (their top directory, or the package name for Haxeon's own packages), and runs
ApiXml over each kit's source roots with the pinned Haxe. ApiXml uses Haxeon's parser and its
dox-compatible XML writer; this script turns that XML into compact JSON for the site
(docs/site/scripts/sync.mjs renders it).

    docs/api/generate.py          rewrite docs/api/data/
    docs/api/generate.py --check  fail if docs/api/data/ is stale

Set MATERIA_API_SOURCE to a clean checkout (git worktree add --detach DIR HEAD) to generate from
committed sources only.
"""

import json
import os
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DATA = os.path.join(HERE, "data")
HAXE = os.environ.get("HAXE", os.path.join(REPO, "haxeon/.tools/haxe/haxe"))
# The kit sources to read; defaults to this checkout. Point it at a clean checkout of a commit to generate
# exactly what CI sees when the working tree holds uncommitted changes.
SOURCE = os.path.abspath(os.environ.get("MATERIA_API_SOURCE", REPO))

# Packages the editor depends on that are not a public API.
EXCLUDED = {"app"}


def packages():
    """Every package the editor depends on, as (directory, manifest)."""
    found = {}

    def visit(directory):
        directory = os.path.normpath(directory)
        if directory in found:
            return
        with open(os.path.join(directory, "haxeon.json"), encoding="utf-8") as handle:
            manifest = json.load(handle)
        found[directory] = manifest
        for dependency in manifest.get("dependencies", {}).values():
            if "path" in dependency:
                visit(os.path.join(directory, dependency["path"]))

    visit(os.path.join(SOURCE, "app"))
    return found


def kit_of(directory, manifest):
    relative = os.path.relpath(directory, SOURCE).split(os.sep)
    if relative[0] == "haxeon":
        return manifest["package"]["name"]
    return relative[0]


def kits():
    grouped = {}
    for directory, manifest in packages().items():
        kit = kit_of(directory, manifest)
        if kit in EXCLUDED:
            continue
        roots = [os.path.join(directory, root) for root in manifest.get("sourceRoots", ["src"])]
        grouped.setdefault(kit, []).extend(root for root in roots if os.path.isdir(root))
    return {kit: sorted(roots) for kit, roots in grouped.items() if roots}


def render(node):
    """A type, written the way Haxe source writes it."""
    if node is None:
        return "Dynamic"
    tag = node.tag
    if tag in ("c", "t", "x", "e"):
        name = node.get("path")
        arguments = [render(child) for child in node]
        return f"{name}<{', '.join(arguments)}>" if arguments else name
    if tag == "f":
        children = list(node)
        names = [name for name in node.get("a", "").split(":")] if node.get("a") else []
        arguments = [render(child) for child in children[:-1]]
        result = render(children[-1]) if children else "Void"
        if not arguments:
            return f"() -> {result}"
        return f"({', '.join(arguments)}) -> {result}"
    if tag == "a":
        fields = []
        for field in node:
            prefix = "?" if field.get("optional") == "1" else ""
            fields.append(f"{prefix}{field.tag}:{render(field[0]) if len(field) else 'Dynamic'}")
        return "{" + ", ".join(fields) + "}"
    if tag == "d":
        return "Dynamic"
    return tag


def signature(node, function):
    """`(name:Type, ...):Result` for a method or enum constructor."""
    names = function.get("a", "").split(":") if function.get("a") else []
    children = list(function)
    arguments = []
    for name, child in zip(names, children[:-1]):
        optional = child.tag == "c" and child.get("path") == "Null" and len(child) == 1
        arguments.append(f"{'?' if optional else ''}{name}:{render(child[0] if optional else child)}")
    result = render(children[-1]) if children else "Void"
    return f"({', '.join(arguments)}):{result}"


def documentation(node):
    doc = node.find("haxe_doc")
    return doc.text.strip() if doc is not None and doc.text else ""


def member(node, owner_kind):
    function = node.find("f")
    entry = {"name": node.tag}
    if owner_kind == "enum":
        entry["kind"] = "case"
        entry["signature"] = signature(node, function) if function is not None else ""
        entry["signature"] = entry["signature"].rsplit("):", 1)[0] + ")" if function is not None else ""
    elif node.get("set") == "method" and function is not None:
        entry["kind"] = "method"
        entry["signature"] = signature(node, function)
    else:
        entry["kind"] = "var"
        typed = [child for child in node if child.tag != "haxe_doc"]
        entry["signature"] = ":" + render(typed[0]) if typed else ""
    if node.get("static") == "1":
        entry["static"] = True
    if node.get("final") == "1":
        entry["final"] = True
    doc = documentation(node)
    if doc:
        entry["doc"] = doc
    return entry


def convert(xml_text, kit):
    root = ET.fromstring(xml_text)
    by_package = {}
    imports_by_file = {}
    for node in root:
        if node.get("private") == "1":
            continue
        path = node.get("path")
        # Decoder and disposal machinery are implementation details, not public platform API.
        if path and path.startswith("haxeon.platform.internal."):
            continue
        # Public signatures should name the actual type, not a file-local import alias.
        source_file = node.get("file", "")
        if source_file not in imports_by_file:
            source_path = source_file if os.path.isabs(source_file) else os.path.join(SOURCE, source_file)
            aliases = {}
            if os.path.isfile(source_path):
                with open(source_path, encoding="utf-8") as handle:
                    aliases = {alias: target for target, alias in re.findall(
                        r"^import\s+([\w.]+)\s+as\s+(\w+)\s*;", handle.read(), re.MULTILINE)}
            imports_by_file[source_file] = aliases
        aliases = imports_by_file[source_file]
        for typed in node.iter():
            if typed.tag in ("c", "t", "x", "e") and typed.get("path"):
                head, separator, tail = typed.get("path").partition(".")
                if head in aliases:
                    typed.set("path", aliases[head] + (separator + tail if separator else ""))
        kind = node.tag
        if kind == "class" and node.get("interface") == "1":
            kind = "interface"
        if kind == "class" and node.get("module"):
            kind = "module"
        package, _, name = path.rpartition(".")
        entry = {"name": name, "kind": kind}
        if node.get("params"):
            entry["params"] = node.get("params").split(":")
        entry["file"] = os.path.relpath(node.get("file"), SOURCE) if os.path.isabs(node.get("file", "")) else node.get("file")
        for relation in ("extends", "implements"):
            related = [render(child[0]) for child in node.findall(relation) if len(child)]
            if related:
                entry[relation] = related
        if kind == "abstract":
            this = node.find("this")
            if this is not None and len(this):
                entry["underlying"] = render(this[0])
        if kind == "typedef":
            body = [child for child in node if child.tag != "haxe_doc"]
            if body:
                if body[0].tag == "a":
                    entry["fields"] = [
                        {"name": field.tag, "kind": "var", "signature": ":" + render(field[0]) if len(field) else "",
                         **({"optional": True} if field.get("optional") == "1" else {})}
                        for field in body[0]
                    ]
                else:
                    entry["alias"] = render(body[0])
        doc = documentation(node)
        if doc:
            entry["doc"] = doc
        if kind != "typedef":
            members = [member(child, kind) for child in node
                       if child.tag not in ("haxe_doc", "extends", "implements", "this", "meta")]
            if members:
                entry["fields"] = members
        by_package.setdefault(package or "(top level)", []).append(entry)
    return {
        "kit": kit,
        "packages": [
            {"name": name, "types": sorted(types, key=lambda t: t["name"])}
            for name, types in sorted(by_package.items())
        ],
    }


def generate():
    outputs = {}
    with tempfile.TemporaryDirectory() as scratch:
        for kit, roots in sorted(kits().items()):
            xml_path = os.path.join(scratch, kit + ".xml")
            subprocess.run([HAXE, "-cp", os.path.join(HERE, "src"), "-cp", os.path.join(REPO, "haxeon/src"),
                            "--run", "ApiXml", xml_path, *roots], check=True, cwd=SOURCE, stdout=subprocess.DEVNULL)
            with open(xml_path, encoding="utf-8") as handle:
                data = convert(handle.read(), kit)
            outputs[kit + ".json"] = json.dumps(data, indent=1, ensure_ascii=False, sort_keys=False) + "\n"
    return outputs


def main():
    check = "--check" in sys.argv[1:]
    outputs = generate()
    existing = set(os.listdir(DATA)) if os.path.isdir(DATA) else set()
    stale = sorted(name for name, text in outputs.items()
                   if not os.path.exists(os.path.join(DATA, name)) or open(os.path.join(DATA, name), encoding="utf-8").read() != text)
    stale += sorted(name for name in existing - set(outputs) if name.endswith(".json"))
    if check:
        if stale:
            print("docs/api/data is stale (run docs/api/generate.py): " + ", ".join(stale), file=sys.stderr)
            return 1
        print(f"docs/api/data is current ({len(outputs)} kits)")
        return 0
    os.makedirs(DATA, exist_ok=True)
    for name in existing - set(outputs):
        if name.endswith(".json"):
            os.remove(os.path.join(DATA, name))
    for name, text in outputs.items():
        with open(os.path.join(DATA, name), "w", encoding="utf-8") as handle:
            handle.write(text)
    types = sum(len(p["types"]) for text in outputs.values() for p in json.loads(text)["packages"])
    print(f"docs/api/data: {len(outputs)} kits, {types} types")
    return 0


if __name__ == "__main__":
    sys.exit(main())
