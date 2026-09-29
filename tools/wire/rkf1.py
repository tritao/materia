"""Check RKF1 Haxe @:wire declarations against their published IDs and types."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

from .validate import ValidationError

DECL = re.compile(r"@:wire\s*(class|enum)\s+(\w+)")
FIELD = re.compile(r"@:id\((\d+)\)\s+public\s+var\s+(\w+)\s*:\s*([^;=]+)")
CASE = re.compile(r"@:id\((\d+)\)\s+(\w+)\s*;")
MESSAGE = re.compile(r"^\s*var\s+(\w+)\s*=\s*(\d+)\s*;", re.M)


def scan(directory: Path) -> dict:
    declarations: dict[str, dict] = {}
    for path in sorted(directory.glob("*.hx")):
        source = path.read_text(encoding="utf-8")
        match = DECL.search(source)
        if not match:
            continue
        kind, name = match.groups()
        if name in declarations:
            raise ValidationError(f"duplicate RKF1 declaration {name}")
        pattern = FIELD if kind == "class" else CASE
        entries = []
        for field in pattern.finditer(source):
            item = {"id": int(field[1]), "name": field[2]}
            if kind == "class":
                item["type"] = re.sub(r"\s+", "", field[3])
            entries.append(item)
        if len(entries) != source.count("@:id("):
            raise ValidationError(f"unparsed @:id in {path.name}")
        if len({item["id"] for item in entries}) != len(entries):
            raise ValidationError(f"duplicate @:id in {path.name}")
        declarations[name] = {"kind": kind, "fields": sorted(entries, key=lambda item: item["id"])}
    message_source = (directory / "RobotMessageType.hx").read_text(encoding="utf-8")
    messages = {name: int(value) for name, value in MESSAGE.findall(message_source)}
    if not declarations or not messages or len(messages) != len(MESSAGE.findall(message_source)):
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
            if fields.get(field["id"]) != field:
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
