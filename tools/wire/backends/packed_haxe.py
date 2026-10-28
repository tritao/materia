"""Haxe records for fixed packed layouts, limited to what Haxe `Int` holds exactly."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from ..model import Schema
from ..validate import ValidationError
from .common import lower_camel

SUPPORTED = {"u8": (1, False), "u16": (2, False), "i8": (1, True), "i16": (2, True), "i32": (4, True)}


def _check(field: dict[str, Any], struct: str, enums: dict[str, list[int]], indent: str) -> list[str]:
    name = field["name"]
    fail = f'throw "{struct}.{name} violates its constraint";'
    array = field["array"]
    def each(test: str) -> list[str]:
        if not array:
            return [f"{indent}if ({test.replace('@', name)}) {fail}"]
        return [f"{indent}for (v in {name}) if ({test.replace('@', 'v')}) {fail}"]
    if "range" in field:
        low, high = field["range"]
        return each(f"@ < {low} || @ > {high}")
    if "mask" in field:
        return each(f"(@ & ~{field['mask']}) != 0")
    if "zero" in field:
        return each("@ != 0")
    if "enum" in field:
        return each("!(" + " || ".join(f"@ == {v}" for v in enums[field["enum"]]) + ")")
    return [
        f"{indent}var {name}End = 0;",
        f"{indent}while ({name}End < {name}.length && {name}[{name}End] != 0) {name}End++;",
        f"{indent}for (i in 0...{name}.length) if (i < {name}End ? ({name}[i] < 32 || {name}[i] > 126) : {name}[i] != 0) {fail}",
    ]


def files(schema: Schema, norm: dict[str, Any], *, root: Path, package: str, label: str) -> dict[Path, str]:
    enums = {enum.name: [item.value for item in enum.values] for enum in schema.enums}
    output: dict[Path, str] = {}
    # Enum values as named Int constants, so code that must agree with the schema cannot drift from it.
    for enum in schema.enums:
        lines = [f"package {package};", "", f"/** Generated from {label}. Do not edit by hand. */", f"class {enum.name} {{"]
        lines += [f"  public static inline final {lower_camel(item.name)}:Int = {item.value};" for item in enum.values]
        lines += ["}", ""]
        output[root / f"{enum.name}.hx"] = "\n".join(lines)
    for packed in schema.packed_structs:
        item = norm["packed_structs"][packed.name]
        arrays = {f.name for f in packed.fields if f.type_ref.length is not None}
        fields = [dict(f, array=f["name"] in arrays) for f in item["fields"]]  # never mutate the lock's norm
        for field in fields:
            if field["type"] not in SUPPORTED:
                raise ValidationError(f"{packed.name}.{field['name']} uses {field['type']}, which the Haxe packed backend cannot hold")
        constrained = [f for f in fields if set(f) - {"name", "type", "length", "array"}]
        little = item["endian"] == "little"
        lines = [f"package {package};", "", "import haxe.io.Bytes;", "",
                 f"/** Generated from {label}. Do not edit by hand. */", f"class {packed.name} {{",
                 f"  public static inline final SIZE:Int = {item['size']};"]
        for field in fields:
            if field["array"]:
                lines.append(f"  public var {field['name']}:Array<Int> = [{', '.join(['0'] * field['length'])}];")
            else:
                lines.append(f"  public var {field['name']}:Int = 0;")
        lines.extend(["  public function new() {}", ""])
        lines.append("  /** Constraints declared in the schema; checked on encode and decode. */")
        lines.append("  public function check():Void {")
        for field in fields:
            if field["array"]:
                lines.append(f"    if ({field['name']}.length != {field['length']}) throw \"{packed.name}.{field['name']} has the wrong length\";")
        for field in constrained:
            lines.extend(_check(field, packed.name, enums, "    "))
        lines.extend(["  }", "", "  public function encode():Bytes {", "    check();", "    var out = Bytes.alloc(SIZE);", "    var offset = 0;"])
        for field in fields:
            size, _ = SUPPORTED[field["type"]]
            values = f"{field['name']}[i]" if field["array"] else field["name"]
            pad = "    "
            if field["array"]:
                lines.append(f"    for (i in 0...{field['length']}) {{")
                pad = "      "
            for byte in range(size):
                shift = byte * 8 if little else (size - 1 - byte) * 8
                lines.append(f"{pad}out.set(offset + {byte}, ({values} >> {shift}) & 0xFF);")
            lines.append(f"{pad}offset += {size};")
            if field["array"]:
                lines.append("    }")
        lines.extend(["    return out;", "  }", "", f"  public static function decode(input:Bytes):{packed.name} {{",
                      f"    if (input == null || input.length != SIZE) throw \"{packed.name} must be {item['size']} bytes\";",
                      f"    var value = new {packed.name}();", "    var offset = 0;"])
        for field in fields:
            size, signed = SUPPORTED[field["type"]]
            pad = "    "
            if field["array"]:
                lines.append(f"    for (i in 0...{field['length']}) {{")
                pad = "      "
            lines.append(f"{pad}var {field['name']}Raw = 0;")
            for byte in range(size):
                shift = byte * 8 if little else (size - 1 - byte) * 8
                lines.append(f"{pad}{field['name']}Raw |= input.get(offset + {byte}) << {shift};")
            if signed and size < 4:
                lines.append(f"{pad}if ({field['name']}Raw >= {1 << (size * 8 - 1)}) {field['name']}Raw -= {1 << (size * 8)};")
            target = f"value.{field['name']}[i]" if field["array"] else f"value.{field['name']}"
            lines.append(f"{pad}{target} = {field['name']}Raw;")
            lines.append(f"{pad}offset += {size};")
            if field["array"]:
                lines.append("    }")
        lines.extend(["    value.check();", "    return value;", "  }", "}", ""])
        output[root / f"{packed.name}.hx"] = "\n".join(lines)
    return output
