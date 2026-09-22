#!/usr/bin/env python3
"""Validate SensorKit's wire schema and generate its shared wire artifacts."""

from __future__ import annotations

import argparse
import json
import struct
import sys
from pathlib import Path
from typing import Any

from .model import Field, Message, Schema
from .parser import parse_file
from .validate import ValidationError, dump_normalized, normalized, validate_evolution


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = ROOT / "schema/sensor_wire.nkw"
EXAMPLES = ROOT / "schema/sensor_wire_examples.json"
LOCK = ROOT / "schema/sensor_wire.lock.json"
PRIMITIVE_SIZE = {"u8": 1, "u32": 4, "u64": 8, "f32": 4, "f64": 8}
STORAGE_TYPE = {"u8": "std::uint8_t", "u32": "std::uint32_t", "u64": "std::uint64_t",
                "f32": "std::uint32_t", "f64": "std::uint64_t"}


def lower_camel(name: str) -> str:
    parts = name.split("_")
    return parts[0] + "".join(part[:1].upper() + part[1:] for part in parts[1:])


def cpp_type(type_name: str) -> str:
    return {
        "u8": "std::uint8_t",
        "u32": "std::uint32_t",
        "u64": "std::uint64_t",
        "f32": "float",
        "f64": "double",
        "bytes": "std::span<const std::uint8_t>",
    }.get(type_name, type_name)


def haxe_type(type_name: str) -> str:
    return {"u8": "Int", "u32": "Int", "u64": "Int64", "f32": "Float", "f64": "Float", "bytes": "Bytes"}.get(type_name, "Int")


def resolved_field_value(field: dict[str, Any]) -> Any:
    return field["value"]


def haxe_files(schema: Schema, normalized_schema: dict[str, Any]) -> dict[Path, str]:
    output: dict[Path, str] = {}
    haxe_root = ROOT / "sensor_io/haxe/materia/sensor/wire"
    for enum in schema.enums:
        lines = [
            "package materia.sensor.wire;",
            "",
            "/** Generated from schema/sensor_wire.nkw. Do not edit by hand. */",
            f"class {enum.name} {{",
        ]
        for item in enum.values:
            lines.append(f"\tpublic static inline var {lower_camel(item.name)}:Int = {item.value};")
        lines.extend(["}", ""])
        output[haxe_root / f"{enum.name}.hx"] = "\n".join(lines)
    if normalized_schema["constants"]:
        lines = [
            "package materia.sensor.wire;",
            "",
            "/** Generated from schema/sensor_wire.nkw. Do not edit by hand. */",
            "class SensorWireConstants {",
        ]
        for name, value in normalized_schema["constants"].items():
            lines.append(f"\tpublic static inline var {lower_camel(name)}:Int = {value};")
        lines.extend(["}", ""])
        output[haxe_root / "SensorWireConstants.hx"] = "\n".join(lines)
    for message in schema.messages:
        fields = normalized_schema["messages"][message.name]["fields"]
        used_types = {f["type"] for f in fields}
        imports = []
        if "u64" in used_types:
            imports.append("haxe.Int64")
        if "bytes" in used_types:
            imports.append("haxe.io.Bytes")
        lines = ["package materia.sensor.wire;", ""]
        lines += [f"import {item};" for item in imports]
        if imports:
            lines.append("")
        lines += ["/** Generated from schema/sensor_wire.nkw. Do not edit by hand. */", "@:wire", f"class {message.name} {{"]
        for field in fields:
            name = lower_camel(field["name"])
            initializer = f" = {resolved_field_value(field)}" if field["constant"] else ""
            lines.append(f"\t@:id({field['id']})")
            lines.append(f"\tpublic var {name}:{haxe_type(field['type'])}{initializer};")
        lines.extend(["}", ""])
        output[ROOT / "sensor_io/haxe/materia/sensor/wire" / f"{message.name}.hx"] = "\n".join(lines)
    return output


def cpp_header(schema: Schema, norm: dict[str, Any]) -> str:
    lines = [
        "#pragma once",
        "",
        "#include <array>",
        "#include <cstddef>",
        "#include <cstdint>",
        "#include <span>",
        "",
        "namespace nksensor::wire::generated {",
        "",
    ]
    for enum in schema.enums:
        lines.append(f"enum class {enum.name} : {cpp_type(enum.underlying)} {{")
        for item in enum.values:
            lines.append(f"    {item.name} = {item.value},")
        lines.extend(["};", ""])
    for constant in norm["constants"].items():
        name, value = constant
        type_name = next(item.type_name for item in schema.constants if item.name == name)
        lines.append(f"inline constexpr {cpp_type(type_name)} {name.lower()} = {value};")
    if norm["constants"]:
        lines.append("")
    for packed in schema.packed_structs:
        packed_norm = norm["packed_structs"][packed.name]
        lines.append(f"inline constexpr std::size_t {packed.name.lower()}_size = {packed_norm['size']};")
    if schema.packed_structs:
        lines.append("")
    imu_data = next((p for p in norm["packed_structs"].values() if len(p["fields"]) == 4), None)
    if imu_data is not None:
        value_count = sum(field["length"] for field in imu_data["fields"])
        lines.append(f"inline constexpr std::size_t imu_packed_value_count = {value_count};")
        lines.append(f"inline constexpr std::size_t imu_packed_data_size = {imu_data['size']};")
    lidar = next((p for p in norm["packed_structs"].values() if any(f["name"] == "hit" for f in p["fields"])), None)
    if lidar is not None:
        lines.append(f"inline constexpr std::size_t lidar_packed_return_size = {lidar['size']};")
    lines.append("")
    for packed in schema.packed_structs:
        lines.append(f"struct {packed.name} {{")
        for field in norm["packed_structs"][packed.name]["fields"]:
            typ = cpp_type(field["type"])
            if field["length"] > 1:
                typ = f"std::array<{typ}, {field['length']}>"
            lines.append(f"    {typ} {field['name']}{{}};")
        lines.extend(["};", ""])
    for message in schema.messages:
        lines.append(f"struct {message.name} {{")
        for field in norm["messages"][message.name]["fields"]:
            typ = cpp_type(field["type"])
            value = field["value"]
            if field["type"] in {enum.name for enum in schema.enums}:
                value = f"static_cast<{typ}>({value})" if field["constant"] else "{}"
            elif field["type"] == "bytes":
                value = "{}"
            elif field["constant"]:
                value = str(value)
            else:
                value = "{}"
            lines.append(f"    {typ} {field['name']} = {value};")
        lines.extend(["};", ""])
    lines.extend(["} // namespace nksensor::wire::generated", ""])
    return "\n".join(lines)


def codec_header(schema: Schema) -> str:
    lines = [
        "#pragma once",
        "",
        '#include "nativekit_sensor_wire_generated.hpp"',
        "#include <string>",
        "",
        "namespace nksensor::wire::detail { class MessagePackReader; class MessagePackWriter; }",
        "namespace nksensor::wire::generated::detail {",
        "",
    ]
    for message in schema.messages:
        lines.append(f"void write(::nksensor::wire::detail::MessagePackWriter &, const {message.name} &);")
        lines.append(f"bool read(::nksensor::wire::detail::MessagePackReader &, {message.name} &, std::string *error);")
    for packed in schema.packed_structs:
        size_name = f"{packed.name.lower()}_size"
        lines.append(f"std::array<std::uint8_t, {size_name}> pack(const {packed.name} &);")
        lines.append(f"bool unpack(std::span<const std::uint8_t>, {packed.name} &);")
    lines.extend(["", "} // namespace nksensor::wire::generated::detail", ""])
    return "\n".join(lines)


def decoder_read(field: dict[str, Any], target: str, schema: Schema) -> list[str]:
    typ = field["type"]
    label = field["name"].replace("_", " ")
    if typ == "bytes":
        return [f"            if (!reader.read_binary_view({target})) {{", f'                set_error(error, "{label} is not MessagePack binary");', "                return false;", "            }"]
    if typ in {"f32", "f64"}:
        if typ == "f32":
            return [
                "            double raw = 0.0;",
                "            if (!reader.read_float(raw)) {",
                f'                set_error(error, "{label} is not a float");',
                "                return false;",
                "            }",
                f"            {target} = static_cast<float>(raw);",
            ]
        return [f"            if (!reader.read_float({target})) {{", f'                set_error(error, "{label} is not a float");', "                return false;", "            }"]
    if typ == "u64":
        return [f"            if (!reader.read_nonnegative_i64({target})) {{", f'                set_error(error, "{label} is not a supported integer");', "                return false;", "            }"]
    enum_storage = {enum.name: enum.underlying for enum in schema.enums}
    storage_type = enum_storage.get(typ, typ)
    maximum = "std::numeric_limits<std::uint8_t>::max()" if storage_type == "u8" else "std::numeric_limits<std::uint32_t>::max()"
    cast_type = typ if typ in enum_storage else cpp_type(typ)
    return [
        "            std::uint64_t raw = 0;",
        f"            if (!reader.read_nonnegative(raw) || raw > {maximum}) {{",
        f'                set_error(error, "{label} is out of range or not an integer");',
        "                return false;",
        "            }",
        f"            {target} = static_cast<{cast_type}>(raw);",
    ]


def codec_cpp(schema: Schema, norm: dict[str, Any]) -> str:
    lines = [
        '#include "sensor_wire_codec.hpp"',
        '#include "msgpack.hpp"',
        "",
        "#include <array>",
        "#include <bit>",
        "#include <limits>",
        "#include <string>",
        "",
        "namespace nksensor::wire::generated::detail {",
    ]
    for packed in schema.packed_structs:
        packed_norm = norm["packed_structs"][packed.name]
        size_name = f"{packed.name.lower()}_size"
        lines.extend([
            f"std::array<std::uint8_t, {size_name}> pack(const {packed.name} &value) {{",
            f"    std::array<std::uint8_t, {size_name}> bytes{{}};",
            "    std::size_t offset = 0;",
        ])
        for field in packed_norm["fields"]:
            scalar_size = PRIMITIVE_SIZE[field["type"]]
            count = field["length"]
            index = "index" if count > 1 else None
            if count > 1:
                lines.append(f"    for (std::size_t index = 0; index < {count}; ++index) {{")
            indent = "        " if count > 1 else "    "
            member = f"value.{field['name']}[{index}]" if index else f"value.{field['name']}"
            storage = STORAGE_TYPE[field["type"]]
            bits_name = f"bits_{field['name']}"
            if field["type"].startswith("f"):
                lines.append(f"{indent}const auto {bits_name} = std::bit_cast<{storage}>({member});")
            else:
                lines.append(f"{indent}const auto {bits_name} = static_cast<{storage}>({member});")
            for byte_index in range(scalar_size):
                shift = byte_index * 8 if packed.endian == "little" else (scalar_size - byte_index - 1) * 8
                lines.append(f"{indent}bytes[offset++] = static_cast<std::uint8_t>({bits_name} >> {shift});")
            if count > 1:
                lines.append("    }")
        lines.extend(["    return bytes;", "}", "", f"bool unpack(std::span<const std::uint8_t> bytes, {packed.name} &value) {{", f"    if (bytes.size() != {size_name}) return false;", "    std::size_t offset = 0;"])
        for field in packed_norm["fields"]:
            scalar_size = PRIMITIVE_SIZE[field["type"]]
            count = field["length"]
            index = "index" if count > 1 else None
            if count > 1:
                lines.append(f"    for (std::size_t index = 0; index < {count}; ++index) {{")
            indent = "        " if count > 1 else "    "
            storage = STORAGE_TYPE[field["type"]]
            bits_name = f"bits_{field['name']}"
            if scalar_size > 1:
                lines.append(f"{indent}{storage} {bits_name} = 0;")
                for byte_index in range(scalar_size):
                    shift = byte_index * 8 if packed.endian == "little" else (scalar_size - byte_index - 1) * 8
                    lines.append(f"{indent}{bits_name} |= static_cast<{storage}>(bytes[offset++]) << {shift};")
            else:
                lines.append(f"{indent}const {storage} {bits_name} = bytes[offset++];")
            member = f"value.{field['name']}[{index}]" if index else f"value.{field['name']}"
            if field["type"].startswith("f"):
                lines.append(f"{indent}{member} = std::bit_cast<{cpp_type(field['type'])}>({bits_name});")
            else:
                lines.append(f"{indent}{member} = static_cast<{cpp_type(field['type'])}>({bits_name});")
            if count > 1:
                lines.append("    }")
        lines.extend(["    return true;", "}", ""])
    lines.extend([
        "namespace {",
        "void set_error(std::string *error, const char *message) { if (error) *error = message; }",
        "}",
        "",
    ])
    enum_names = {enum.name for enum in schema.enums}
    enum_storage = {enum.name: enum.underlying for enum in schema.enums}
    for message in schema.messages:
        fields = norm["messages"][message.name]["fields"]
        lines.append(f"void write(::nksensor::wire::detail::MessagePackWriter &writer, const {message.name} &value) {{")
        lines.append(f"    writer.write_map_header({len(fields)});")
        for field in fields:
            lines.append(f"    writer.write_integer({field['id']});")
            value = f"value.{field['name']}"
            if field["constant"]:
                value = (
                    f"static_cast<std::uint64_t>(static_cast<{field['type']}>({field['value']}))"
                    if field["type"] in enum_names
                    else str(field["value"])
                )
                lines.append(f"    writer.write_integer({value});")
            elif field["type"] == "bytes":
                lines.append(f"    writer.write_binary({value});")
            elif field["type"] in {"f32", "f64"}:
                lines.append(f"    writer.write_float64(static_cast<double>({value}));")
            else:
                expr = f"static_cast<std::uint64_t>({value})" if field["type"] in enum_names else value
                lines.append(f"    writer.write_integer({expr});")
        lines.extend(["}", "", f"bool read(::nksensor::wire::detail::MessagePackReader &reader, {message.name} &value, std::string *error) {{", "    std::uint32_t field_count = 0;", "    if (!reader.read_map_size(field_count)) {", f'        set_error(error, "{message.name} is not a MessagePack map");', "        return false;", "    }"])
        for field in fields:
            lines.append(f"    bool has_{field['name']} = false;")
        lines.extend(["    for (std::uint32_t index = 0; index < field_count; ++index) {", "        std::uint64_t key = 0;", "        if (!reader.read_nonnegative(key)) {", '            set_error(error, "wire field key is not a non-negative integer");', "            return false;", "        }", "        switch (key) {"])
        for field in fields:
            lines.extend([f"        case {field['id']}: {{"])
            if field["constant"]:
                temp_field = dict(field)
                temp_field["constant"] = False
                lines.append("            std::uint64_t raw = 0;")
                typ = field["type"]
                if typ == "u64":
                    lines.append("            if (!reader.read_nonnegative_i64(raw)) {")
                elif typ == "f64":
                    lines.append("            double raw_float = 0.0;")
                    lines.append("            if (!reader.read_float(raw_float)) {")
                elif typ == "bytes":
                    lines.append("            std::span<const std::uint8_t> raw_bytes;")
                    lines.append("            if (!reader.read_binary_view(raw_bytes)) {")
                else:
                    storage_type = enum_storage.get(typ, typ)
                    maximum = "std::numeric_limits<std::uint8_t>::max()" if storage_type == "u8" else "std::numeric_limits<std::uint32_t>::max()"
                    lines.append(f"            if (!reader.read_nonnegative(raw) || raw > {maximum}) {{")
                lines.extend(["                set_error(error, \"invalid constant wire field\");", "                return false;", "            }"])
                expected = field["value"]
                if typ == "f64":
                    lines.append(f"            if (raw_float != {expected}) {{")
                elif typ == "bytes":
                    lines.append("                set_error(error, \"invalid constant wire field\");")
                    lines.append("                return false;")
                    lines.append("            }")
                    lines.append("            value." + field["name"] + " = {};")
                    lines.append(f"            has_{field['name']} = true;")
                    lines.append("            break;")
                    lines.append("        }")
                    continue
                else:
                    lines.append(f"            if (raw != {expected}) {{")
                constant_message = f"invalid constant field {message.name}.{field['name']}"
                if field["name"] == "schema_version":
                    constant_message = f"unsupported {message.name} schema version"
                elif field["name"] == "message_type":
                    constant_message = f"MessagePack value is not a {message.name}"
                elif field["name"] == "return_stride":
                    constant_message = "LiDAR return stride is not the supported packed format"
                lines.extend([f'                set_error(error, "{constant_message}");', "                return false;", "            }"])
                lines.append(f"            value.{field['name']} = {expected};" if typ not in enum_names else f"            value.{field['name']} = static_cast<{typ}>({expected});")
            else:
                lines.extend(decoder_read(field, f"value.{field['name']}", schema))
            lines.append(f"            has_{field['name']} = true;")
            lines.extend(["            break;", "        }"])
        lines.extend(["        default:", "            if (!reader.skip()) {", '                set_error(error, "wire message contains an invalid unknown field");', "                return false;", "            }", "            break;", "        }", "    }", "    if (!reader.at_end()) {", f'        set_error(error, "{message.name} MessagePack value has trailing bytes");', "        return false;", "    }"])
        missing = " || ".join(f"!has_{field['name']}" for field in fields)
        lines.extend([f"    if ({missing}) {{", f'        set_error(error, "{message.name} is missing a required field");', "        return false;", "    }", "    return true;", "}", ""])
    lines.extend(["} // namespace nksensor::wire::generated::detail", ""])
    return "\n".join(lines)


def _mp_uint(value: int) -> bytes:
    if value < 0:
        raise ValueError("wire examples only support non-negative values")
    if value < 128:
        return bytes([value])
    if value <= 0xFF:
        return b"\xcc" + struct.pack(">B", value)
    if value <= 0xFFFF:
        return b"\xcd" + struct.pack(">H", value)
    if value <= 0xFFFFFFFF:
        return b"\xce" + struct.pack(">I", value)
    return b"\xcf" + struct.pack(">Q", value)


def _mp_binary(value: bytes) -> bytes:
    size = len(value)
    if size <= 0xFF:
        return b"\xc4" + struct.pack(">B", size) + value
    if size <= 0xFFFF:
        return b"\xc5" + struct.pack(">H", size) + value
    return b"\xc6" + struct.pack(">I", size) + value


def _mp_float(value: float) -> bytes:
    return b"\xcb" + struct.pack(">d", value)


def hmpk_message(fields: list[dict[str, Any]], example: dict[str, Any]) -> bytes:
    count = len(fields)
    if count <= 15:
        payload = bytes([0x80 | count])
    else:
        payload = b"\xde" + struct.pack(">H", count)
    for field in fields:
        payload += _mp_uint(field["id"])
        name = field["name"]
        if name == "data" and "data_f64" in example:
            value = b"".join(struct.pack("<d", float(item)) for item in example["data_f64"])
            if len(value) != 24 * 8:
                raise ValueError("IMU vector must contain exactly 24 values")
            payload += _mp_binary(value)
            continue
        elif name == "data" and "data_hex" in example:
            value = bytes.fromhex(example["data_hex"])
            payload += _mp_binary(value)
            continue
        value = example[name]
        if field["type"] == "f64":
            payload += _mp_float(float(value))
        else:
            payload += _mp_uint(int(value))
    return b"HMPK\x01\x00" + struct.pack(">I", len(payload)) + payload


def vector_fixture(norm: dict[str, Any]) -> str:
    examples = json.loads(EXAMPLES.read_text(encoding="utf-8"))
    lines = ["# Generated from schema/sensor_wire.nkw and sensor_wire_examples.json; complete HMPK frames."]
    for message_name, example in examples.items():
        fields = norm["messages"][message_name]["fields"]
        lines.append(f"{message_name}\t{hmpk_message(fields, example).hex()}")
    return "\n".join(lines) + "\n"


def haxe_vector_check(norm: dict[str, Any]) -> str:
    examples = json.loads(EXAMPLES.read_text(encoding="utf-8"))
    fields_by_message = norm["messages"]
    enum_values = norm["enums"]
    used_enums = sorted({
        field["type"]
        for message in fields_by_message.values()
        for field in message["fields"]
        if field["type"] in enum_values
    })
    lines = [
        "import haxe.Int64;",
        "import haxe.io.Bytes;",
        "import haxeon.wire.MessagePack;",
        "import haxeon.wire.MessagePackFrame;",
        "import materia.sensor.wire.ImuSampleMessage;",
        "import materia.sensor.wire.LidarScanMessage;",
        "import materia.sensor.wire.PackedFrameMessage;",
    ]
    lines.extend(f"import materia.sensor.wire.{enum_name};" for enum_name in used_enums)
    lines.extend([
        "import sys.io.File;",
        "",
        "/** Generated interop check. Compile with Haxeon and pass the generated TSV path. */",
        "class SensorWireVectorCheck {",
        "\tstatic function main():Int {",
        "\t\tvar expected = readVectors(Sys.args()[0]);",
    ])
    for message_name, example in examples.items():
        var_name = {"PackedFrameMessage": "packed", "ImuSampleMessage": "imu", "LidarScanMessage": "lidar"}[message_name]
        lines.append(f"\t\tvar {var_name} = new {message_name}();")
        for field in fields_by_message[message_name]["fields"]:
            name = field["name"]
            if name == "data":
                if "data_f64" in example:
                    raw = b"".join(struct.pack("<d", float(value)) for value in example["data_f64"])
                    literal = raw.hex()
                else:
                    literal = example["data_hex"]
                value_expr = f'bytesFromHex("{literal}")'
            elif field["type"] == "u64":
                value_expr = f'Int64.parseString("{example[name]}")'
            elif field["type"] in enum_values:
                item_name = next(key for key, value in enum_values[field["type"]]["values"].items()
                                 if value == example[name])
                value_expr = f"{field['type']}.{lower_camel(item_name)}"
            else:
                value_expr = str(example[name])
            lines.append(f"\t\t{var_name}.{lower_camel(name)} = {value_expr};")
        vector_key = message_name
        lines.append(f'\t\tverify(expected, "{vector_key}", MessagePackFrame.pack(MessagePack.encode({var_name})));')
    lines.extend([
        "\t\treturn 42;",
        "\t}",
        "",
        "\tstatic function readVectors(path:String):Map<String, Bytes> {",
        "\t\tvar result:Map<String, Bytes> = [];",
        "\t\tfor (line in File.getContent(path).split(\"\\n\")) {",
        "\t\t\tvar text = StringTools.trim(line);",
        "\t\t\tif (text == \"\" || StringTools.startsWith(text, \"#\")) continue;",
        "\t\t\tvar pair = text.split(\"\\t\");",
        "\t\t\tresult.set(pair[0], bytesFromHex(pair[1]));",
        "\t\t}",
        "\t\treturn result;",
        "\t}",
        "",
        "\tstatic function verify(expected:Map<String, Bytes>, name:String, actual:Bytes):Void {",
        "\t\tvar vector = expected.get(name);",
        "\t\tif (vector == null || vector.compare(actual) != 0) throw 'Sensor wire vector differs: $name';",
        "\t}",
        "",
        "\tstatic function bytesFromHex(text:String):Bytes {",
        "\t\tvar bytes = Bytes.alloc(text.length >> 1);",
        "\t\tfor (index in 0...bytes.length)",
        "\t\t\tbytes.set(index, (hexDigit(text.charCodeAt(index * 2)) << 4) | hexDigit(text.charCodeAt(index * 2 + 1)));",
        "\t\treturn bytes;",
        "\t}",
        "",
        "\tstatic function hexDigit(code:Int):Int {",
        "\t\tif (code >= 48 && code <= 57) return code - 48;",
        "\t\tif (code >= 65 && code <= 70) return code - 55;",
        "\t\treturn code - 87;",
        "\t}",
        "}",
        "",
    ])
    return "\n".join(lines)


def docs(schema: Schema, norm: dict[str, Any]) -> str:
    lines = ["# SensorKit wire schema", "", "Generated from `schema/sensor_wire.nkw`. MessagePack fields use integer map keys; the HMPK envelope is version 1.", ""]
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
            lines.append(f"| {field['id']} | `{field['name']}` | `{field['type']}` | {rule} |")
        reserved = ", ".join(f"{lo}..{hi}" if lo != hi else str(lo) for lo, hi in norm["messages"][message.name]["reserved"])
        lines.extend(["", f"Reserved IDs: `{reserved}`.", ""])
    for packed in schema.packed_structs:
        item = norm["packed_structs"][packed.name]
        lines.extend([f"## {packed.name} (packed)", "", f"Endianness: `{item['endian']}`; size: **{item['size']} bytes**.", "", "| Field | Type | Size |", "| --- | --- | ---: |"])
        for field in item["fields"]:
            size = {"u8": 1, "u32": 4, "u64": 8, "f32": 4, "f64": 8}[field["type"]] * field["length"]
            suffix = f"[{field['length']}]" if field["length"] != 1 else ""
            lines.append(f"| `{field['name']}` | `{field['type']}{suffix}` | {size} |")
        lines.append("")
    lines.extend(["## Compatibility", "", "Existing fields, IDs, types, constants, and packed layouts are immutable. New enum values and new message types can be added. Adding a field to an existing message is rejected; define a new message when the wire shape changes.", ""])
    return "\n".join(lines)


def outputs(schema: Schema, norm: dict[str, Any]) -> dict[Path, str]:
    result = haxe_files(schema, norm)
    result.update({
        ROOT / "sensor_io/include/nativekit_sensor_wire_generated.hpp": cpp_header(schema, norm),
        ROOT / "sensor_io/generated/cpp/sensor_wire_codec.hpp": codec_header(schema),
        ROOT / "sensor_io/generated/cpp/sensor_wire_generated.cpp": codec_cpp(schema, norm),
        ROOT / "sensor_io/generated/sensor_wire.md": docs(schema, norm),
        ROOT / "sensor_io/tests/fixtures/sensor_wire_vectors.tsv": vector_fixture(norm),
        ROOT / "sensor_io/tests/haxe/SensorWireVectorCheck.hx": haxe_vector_check(norm),
        LOCK: dump_normalized(norm),
    })
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail when checked-in generated output is stale")
    parser.add_argument("--schema", type=Path, default=SCHEMA)
    args = parser.parse_args(argv)
    try:
        schema = parse_file(args.schema)
        norm = normalized(schema)
        if LOCK.exists():
            old = json.loads(LOCK.read_text(encoding="utf-8"))
            validate_evolution(norm, old)
        elif args.check:
            raise ValidationError("schema compatibility lock is missing")
        generated = outputs(schema, norm)
        stale: list[Path] = []
        for path, content in generated.items():
            if args.check:
                if not path.exists() or path.read_text(encoding="utf-8") != content:
                    stale.append(path)
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8")
        if stale:
            for path in stale:
                print(f"stale generated file: {path.relative_to(ROOT)}", file=sys.stderr)
            return 1
        print("SensorKit wire schema is valid" if args.check else "Generated SensorKit wire artifacts")
        return 0
    except (OSError, ValueError, ValidationError) as exc:
        print(f"wire schema error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
