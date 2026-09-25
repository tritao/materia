"""Haxe records and strict MessagePack payload entry points."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from ..model import Schema
from .common import haxe_type, lower_camel

def haxe_files(schema: Schema, normalized_schema: dict[str, Any], *, root: Path, package: str, codec_name: str, constants_name: str, schema_label: str, codec_comment: str) -> dict[Path, str]:
    output: dict[Path, str] = {}
    haxe_root = root
    for enum in schema.enums:
        lines = [
            f"package {package};",
            "",
            f"/** Generated from {schema_label}. Do not edit by hand. */",
            f"class {enum.name} {{",
        ]
        for item in enum.values:
            lines.append(f"\tpublic static inline var {lower_camel(item.name)}:Int = {item.value};")
        lines.extend(["}", ""])
        output[haxe_root / f"{enum.name}.hx"] = "\n".join(lines)
    if normalized_schema["constants"]:
        lines = [
            f"package {package};",
            "",
            f"/** Generated from {schema_label}. Do not edit by hand. */",
            f"class {constants_name} {{",
        ]
        for name, constant in normalized_schema["constants"].items():
            lines.append(f"\tpublic static inline var {lower_camel(name)}:Int = {constant['value']};")
        lines.extend(["}", ""])
        output[haxe_root / f"{constants_name}.hx"] = "\n".join(lines)
    for message in schema.messages:
        fields = normalized_schema["messages"][message.name]["fields"]
        used_types = {f["type"] for f in fields}
        imports = []
        if used_types & {"u32", "u64", "i64"}:
            imports.append("haxe.Int64")
        if "bytes" in used_types:
            imports.append("haxe.io.Bytes")
        lines = [f"package {package};", ""]
        lines += [f"import {item};" for item in imports]
        if imports:
            lines.append("")
        lines += [f"/** Generated from {schema_label}. Do not edit by hand. */", "@:wire", f"class {message.name} {{"]
        for field in fields:
            name = lower_camel(field["name"])
            lines.append(f"\t@:id({field['id']})")
            lines.append(f"\tpublic var {name}:{haxe_type(field['type'])};")
        lines.extend(["}", ""])
        output[haxe_root / f"{message.name}.hx"] = "\n".join(lines)
    output[haxe_root / f"{codec_name}.hx"] = haxe_codec_file(normalized_schema, package=package, codec_name=codec_name, codec_comment=codec_comment)
    return output


def haxe_codec_file(norm: dict[str, Any], *, package: str, codec_name: str, codec_comment: str) -> str:
    messages = norm["messages"]
    enum_storage = {name: item["underlying"] for name, item in norm["enums"].items()}
    lines = [
        f"package {package};",
        "",
        "import haxe.Int64;",
        "import haxe.io.Bytes;",
        "import haxeon.wire.MessagePack;",
        "import haxeon.wire.MessagePackReader;",
    ]
    lines.extend(f"import {package}.{name};" for name in messages)
    lines.extend([
        "",
        codec_comment,
        f"class {codec_name} {{",
    ])

    for message_name, message in messages.items():
        fields = message["fields"]
        value_name = "decoded"
        validate_name = f"validate{message_name}"
        lines.extend([
            f"\tpublic static function encode{message_name}(value:{message_name}):Bytes {{",
            f"\t\t{validate_name}(value);",
            "\t\treturn MessagePack.encode(value);",
            "\t}",
            "",
            f"\tpublic static function decode{message_name}(bytes:Bytes):{message_name} {{",
            f"\t\tvalidateKeys(bytes, \"{message_name}\", {[field['id'] for field in fields]});",
            f"\t\tvar {value_name}:{message_name} = MessagePack.decode(bytes);",
            f"\t\t{validate_name}({value_name});",
            f"\t\treturn {value_name};",
            "\t}",
            "",
            f"\tstatic function {validate_name}(value:{message_name}):Void {{",
        ])
        for field in fields:
            name = lower_camel(field["name"])
            if field["constant"]:
                expected = field["value"]
                if field["type"] in {"i64", "u32"}:
                    compare = f"Int64.compare(value.{name}, Int64.parseString(\"{expected}\")) != 0"
                else:
                    compare = f"value.{name} != {expected}"
                lines.append(f"\t\tif ({compare}) throw \"invalid constant field {message_name}.{field['name']}\";")

            typ = enum_storage.get(field["type"], field["type"])
            condition = None
            if typ in {"u8", "u16"}:
                high = 2 ** (8 if typ == "u8" else 16) - 1
                condition = f"value.{name} < 0 || value.{name} > {high}"
            elif typ == "u32":
                condition = (f"Int64.compare(value.{name}, Int64.ofInt(0)) < 0 || "
                             f"Int64.compare(value.{name}, Int64.make(0, -1)) > 0")
            elif typ in {"i8", "i16", "i32"}:
                bits = int(typ[1:])
                condition = f"value.{name} < {-2 ** (bits - 1)} || value.{name} > {2 ** (bits - 1) - 1}"
            elif typ == "i64" and field["nonnegative"]:
                condition = f"Int64.compare(value.{name}, Int64.ofInt(0)) < 0"
            if field["nonnegative"] and typ != "i64":
                condition = f"({condition}) || value.{name} < 0" if condition else f"value.{name} < 0"
            if condition:
                lines.append(f"\t\tif ({condition}) throw \"{message_name}.{field['name']} is out of range\";")
        lines.extend(["\t}", ""])

    lines.extend([
        "\tstatic function validateKeys(bytes:Bytes, message:String, required:Array<Int>):Void {",
        "\t\tvar reader = new MessagePackReader(bytes);",
        "\t\tvar count = reader.readMapHeader();",
        "\t\tvar seen:Array<Int> = [];",
        "\t\tfor (_ in 0...count) {",
        "\t\t\tvar key = reader.readInt();",
        "\t\t\tif (key < 0) throw 'negative field ID $key in $message';",
        "\t\t\tif (seen.indexOf(key) >= 0) throw 'duplicate field ID $key in $message';",
        "\t\t\tseen.push(key);",
        "\t\t\treader.skip();",
        "\t\t}",
        "\t\tfor (id in required)",
        "\t\t\tif (seen.indexOf(id) < 0) throw 'missing required field ID $id in $message';",
        "\t\tif (!reader.atEnd()) throw 'trailing bytes after $message';",
        "\t}",
        "}",
        "",
    ])
    return "\n".join(lines)
