"""Markdown wire field and packed-layout reference."""

from __future__ import annotations

from typing import Any

from ..model import Schema
from .common import PRIMITIVE_SIZE

def docs(schema: Schema, norm: dict[str, Any], *, title: str, intro: str, codec_name: str) -> str:
    lines = [
        f"# {title}",
        "",
        intro,
        "",
        "Integer ranges follow the declared primitive. Haxe `u32` fields use `Int64` so the full unsigned 32-bit range is representable; "
        "message fields cannot use `u64` because Haxe `Int64` cannot represent its full range. "
        "`i64 nonnegative` fields match Haxe `Int64` and restrict the protocol value to `0..2^63-1`. "
        f"Use the generated `{codec_name}` entry points in Haxe to enforce required fields, duplicates, constants, and integer ranges.",
        "",
    ]
    for enum_name, enum in norm["enums"].items():
        lines.append(f"## {enum_name}")
        lines.append("")
        lines.extend(["| Name | Value |", "| --- | ---: |"])
        lines.extend(f"| `{name}` | {value} |" for name, value in enum["values"].items())
        lines.append("")
    for message in schema.messages:
        lines.append(f"## {message.name}")
        lines.append("")
        lines.extend(["| ID | Field | Type | Rule |", "| ---: | --- | --- | --- |"])
        for field in norm["messages"][message.name]["fields"]:
            rule = f"constant `{field['value']}`" if field["constant"] else "required"
            if field["nonnegative"]:
                rule += ", non-negative"
            lines.append(f"| {field['id']} | `{field['name']}` | `{field['type']}` | {rule} |")
        lines.append("")
    for packed in schema.packed_structs:
        item = norm["packed_structs"][packed.name]
        lines.extend([f"## {packed.name} (packed)", "", f"Endianness: `{item['endian']}`; size: **{item['size']} bytes**.", "", "| Field | Type | Size |", "| --- | --- | ---: |"])
        for field in item["fields"]:
            size = PRIMITIVE_SIZE[field["type"]] * field["length"]
            suffix = f"[{field['length']}]" if field["length"] != 1 else ""
            lines.append(f"| `{field['name']}` | `{field['type']}{suffix}` | {size} |")
        lines.append("")
    lines.extend([
        "## Compatibility",
        "",
        "Published message shapes and packed layouts are immutable. Existing field IDs, types, and constants cannot change. "
        "Create a new message and message type when a shape must change. Existing enum values cannot change; new enum values and message types may be added. "
        "Unknown MessagePack fields are skipped, and duplicate field IDs are rejected.",
        "",
    ])
    return "\n".join(lines)
