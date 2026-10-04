"""Embed the RKF1-locked recording class schemas in the Haxe recorder."""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

ROOTS = {
    "command": "RecordingCommandMsg",
    "snapshot": "RecordingSnapshotMsg",
    "sensor": "RecordingSensorMsg",
    "fault": "RecordingFaultMsg",
    "world": "RecordingWorldMsg",
    "world_event": "RecordingWorldEventMsg",
    "process_event": "RecordingProcessEventMsg",
    "perception.image_detections": "ImageDetectionObservationMsg",
}


def schema(root: str, declarations: dict) -> str:
    found: dict = {}

    def visit(name: str) -> None:
        if name in found:
            return
        entry = declarations[name]
        found[name] = entry
        for field in entry["fields"]:
            for dependency in re.findall(r"[A-Z]\w+Msg", field.get("type", "")):
                visit(dependency)

    visit(root)
    return json.dumps({"root": root, "declarations": found}, sort_keys=True, separators=(",", ":"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("generate", "check"))
    parser.add_argument("--lock", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    lock = json.loads(args.lock.read_text())
    lines = ["package robotkit.recording;", "", "// Generated from the RKF1 schema lock. Do not edit by hand.",
             "class RecordingSchemas {", "  public static function forChannel(name:String):String return switch name {"]
    for topic, root in ROOTS.items():
        lines.append(f'    case "{topic}": {json.dumps(schema(root, lock["declarations"]))};')
    lines.extend(['    case _: throw "Unknown core recording schema";', "  };"])
    # v6 recordings predating per-plan stop limits retain their exact schema.
    legacy = json.loads(json.dumps(lock["declarations"]))
    legacy["RecordingPlanMsg"]["fields"] = [field for field in legacy["RecordingPlanMsg"]["fields"]
                                            if field["name"] != "controlAcceleration"]
    lines.extend(["", "  /** Previous v6 command schema, before optional controlled-stop limits. */",
                  "  public static function legacyCommand():String return " +
                  json.dumps(schema("RecordingCommandMsg", legacy)) + ";", "}", ""])

    rendered = "\n".join(lines)
    if args.command == "check":
        if not args.output.exists() or args.output.read_text() != rendered:
            parser.exit(1, "recording schemas are stale\n")
    else:
        args.output.write_text(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
