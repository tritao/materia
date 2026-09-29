"""Check RKF1 Haxe @:wire declarations against published IDs and types."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

try:
    from .validate import ValidationError
except ImportError:  # Also support invoking this file by absolute path from any cwd.
    from validate import ValidationError

DECL = re.compile(r"@:wire\s*(class|enum)\s+(\w+)")
FIELD = re.compile(r"@:id\((\d+)\)\s+public\s+var\s+(\w+)\s*:\s*([^;=]+)")
CASE = re.compile(r"@:id\((\d+)\)\s+(\w+)\s*;")
MESSAGE = re.compile(r"^\s*var\s+(\w+)\s*=\s*(0x[\da-fA-F]+|\d+)\s*;", re.M)


def without_comments(source: str) -> str:
    """Blank comments while preserving offsets and quoted strings."""
    out = list(source)
    i = 0
    while i < len(source):
        if source[i] in "\"'":
            quote = source[i]
            i += 1
            while i < len(source):
                if source[i] == "\\":
                    i += 2
                elif source[i] == quote:
                    i += 1
                    break
                else:
                    i += 1
        elif source.startswith("//", i):
            end = source.find("\n", i)
            if end < 0:
                end = len(source)
            for pos in range(i, end):
                out[pos] = " "
            i = end
        elif source.startswith("/*", i):
            end = source.find("*/", i + 2)
            if end < 0:
                raise ValidationError("unterminated Haxe block comment")
            end += 2
            for pos in range(i, end):
                if out[pos] != "\n":
                    out[pos] = " "
            i = end
        else:
            i += 1
    return "".join(out)


def declaration_body(source: str, start: int) -> str:
    opening = source.find("{", start)
    if opening < 0:
        raise ValidationError("RKF1 declaration has no body")
    depth = 1
    i = opening + 1
    while i < len(source) and depth:
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
        i += 1
    if depth:
        raise ValidationError("unterminated RKF1 declaration")
    return source[opening + 1 : i - 1]


def canonical_type(value: str) -> str:
    return re.sub(r"\s+", "", value).replace("haxe.Int64", "Int64")


def scan(directory: Path) -> dict:
    declarations: dict[str, dict] = {}
    for path in sorted(directory.glob("*.hx")):
        source = without_comments(path.read_text(encoding="utf-8"))
        for match in DECL.finditer(source):
            kind, name = match.groups()
            if name in declarations:
                raise ValidationError(f"duplicate RKF1 declaration {name}")
            body = declaration_body(source, match.end())
            pattern = FIELD if kind == "class" else CASE
            entries = []
            for field in pattern.finditer(body):
                item = {"id": int(field[1]), "name": field[2]}
                if kind == "class":
                    item["type"] = canonical_type(field[3])
                entries.append(item)
            if len(entries) != body.count("@:id("):
                raise ValidationError(f"unparsed @:id in {path.name}:{name}")
            if len({item["id"] for item in entries}) != len(entries):
                raise ValidationError(f"duplicate @:id in {path.name}:{name}")
            declarations[name] = {"kind": kind, "fields": sorted(entries, key=lambda item: item["id"])}
    message_source = without_comments((directory / "RobotMessageType.hx").read_text(encoding="utf-8"))
    messages = {name: int(value, 0) for name, value in MESSAGE.findall(message_source)}
    if not declarations or not messages:
        raise ValidationError("incomplete RKF1 declarations or message types")
    if len(set(messages.values())) != len(messages):
        raise ValidationError("duplicate RKF1 message type value")
    return {"version": 1, "declarations": declarations, "messageTypes": messages}


def check(current: dict, previous: dict) -> None:
    if current.get("version") != previous.get("version"):
        raise ValidationError("RKF1 lock version changed")
    for name, value in previous["messageTypes"].items():
        if current["messageTypes"].get(name) != value:
            raise ValidationError(f"changed or removed RKF1 message type {name}")
    for name, old in previous["declarations"].items():
        new = current["declarations"].get(name)
        if new is None or new["kind"] != old["kind"]:
            raise ValidationError(f"changed or removed RKF1 declaration {name}")
        fields = {field["id"]: field for field in new["fields"]}
        for field in old["fields"]:
            expected = dict(field)
            if "type" in expected:
                expected["type"] = canonical_type(expected["type"])
            if fields.get(field["id"]) != expected:
                raise ValidationError(f"changed or removed RKF1 field {name}.@id({field['id']})")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("check", "generate"))
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--lock", type=Path, required=True)
    args = parser.parse_args()
    try:
        current = scan(args.source)
        if args.command == "generate":
            if args.lock.exists():
                check(current, json.loads(args.lock.read_text(encoding="utf-8")))
            args.lock.write_text(json.dumps(current, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        else:
            check(current, json.loads(args.lock.read_text(encoding="utf-8")))
    except (OSError, ValueError, KeyError, ValidationError) as exc:
        parser.exit(1, f"RKF1 schema lock error: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
