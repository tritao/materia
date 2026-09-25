"""C++ generated types, MessagePack codecs, and packed layouts."""

from __future__ import annotations

from typing import Any

from ..model import Schema
from .common import PRIMITIVE_SIZE, STORAGE_TYPE, cpp_type

def cpp_header(schema: Schema, norm: dict[str, Any], *, namespace: str, aliases: dict[str, str]) -> str:
    lines = [
        "#pragma once",
        "",
        "#include <array>",
        "#include <cstddef>",
        "#include <cstdint>",
        "#include <span>",
        "",
        f"namespace {namespace} {{",
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
        lines.append(f"inline constexpr {cpp_type(type_name)} {name.lower()} = {value['value']};")
    if norm["constants"]:
        lines.append("")
    for packed in schema.packed_structs:
        packed_norm = norm["packed_structs"][packed.name]
        lines.append(f"inline constexpr std::size_t {packed.name.lower()}_size = {packed_norm['size']};")
    if schema.packed_structs:
        lines.append("")
    for alias, expression in aliases.items():
        lines.append(f"inline constexpr std::size_t {alias} = {expression};")
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
    lines.extend([f"}} // namespace {namespace}", ""])
    return "\n".join(lines)


def codec_header(schema: Schema, *, namespace: str, reader_namespace: str, generated_header: str) -> str:
    lines = [
        "#pragma once",
        "",
        f'#include "{generated_header}"',
        "#include <string>",
        "",
        f"namespace {reader_namespace} {{ class MessagePackReader; class MessagePackWriter; }}",
        f"namespace {namespace}::detail {{",
        "",
    ]
    for message in schema.messages:
        lines.append(f"bool write(::{reader_namespace}::MessagePackWriter &, const {message.name} &, std::string *error);")
        lines.append(f"bool read(::{reader_namespace}::MessagePackReader &, {message.name} &, std::string *error);")
    for packed in schema.packed_structs:
        size_name = f"{packed.name.lower()}_size"
        lines.append(f"std::array<std::uint8_t, {size_name}> pack(const {packed.name} &);")
        lines.append(f"bool unpack(std::span<const std::uint8_t>, {packed.name} &);")
    lines.extend(["", f"}} // namespace {namespace}::detail", ""])
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
    enum_storage = {enum.name: enum.underlying for enum in schema.enums}
    storage_type = enum_storage.get(typ, typ)
    cast_type = typ if typ in enum_storage else cpp_type(typ)
    if storage_type.startswith("i"):
        if field["nonnegative"]:
            range_condition = "raw < 0"
        else:
            range_condition = "false"
        if storage_type != "i64":
            range_condition += (
                f" || raw < std::numeric_limits<{cpp_type(storage_type)}>::min()"
                f" || raw > std::numeric_limits<{cpp_type(storage_type)}>::max()"
            )
        return [
            "            std::int64_t raw = 0;",
            f"            if (!reader.read_signed_integer(raw) || {range_condition}) {{",
            f'                set_error(error, "{label} is out of range or not a signed integer");',
            "                return false;",
            "            }",
            f"            {target} = static_cast<{cast_type}>(raw);",
        ]
    maximum = f"std::numeric_limits<{cpp_type(storage_type)}>::max()"
    return [
        "            std::uint64_t raw = 0;",
        f"            if (!reader.read_nonnegative(raw) || raw > {maximum}) {{",
        f'                set_error(error, "{label} is out of range or not an integer");',
        "                return false;",
        "            }",
        f"            {target} = static_cast<{cast_type}>(raw);",
    ]


def codec_cpp(schema: Schema, norm: dict[str, Any], *, namespace: str, reader_namespace: str, codec_header_name: str, msgpack_header: str) -> str:
    lines = [
        f'#include "{codec_header_name}"',
        f'#include "{msgpack_header}"',
        "",
        "#include <array>",
        "#include <bit>",
        "#include <limits>",
        "#include <string>",
        "#include <unordered_set>",
        "",
        f"namespace {namespace}::detail {{",
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
            elif field["type"].startswith("i"):
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
        lines.extend([
            f"bool write(::{reader_namespace}::MessagePackWriter &writer, const {message.name} &value, std::string *error) {{",
        ])
        for field in fields:
            if field["nonnegative"]:
                lines.extend([
                    f"    if (value.{field['name']} < 0) {{",
                    f'        set_error(error, "{message.name}.{field["name"]} must be non-negative");',
                    "        return false;",
                    "    }",
                ])
        lines.append(f"    writer.write_map_header({len(fields)});")
        for field in fields:
            lines.append(f"    writer.write_integer({field['id']});")
            value = f"value.{field['name']}"
            if field["constant"]:
                if field["type"] in {"f32", "f64"}:
                    lines.append(f"    writer.write_float64(static_cast<double>({field['value']}));")
                elif field["type"].startswith("i"):
                    lines.append(f"    writer.write_signed_integer({field['value']});")
                elif field["type"] in enum_names and enum_storage[field["type"]].startswith("i"):
                    lines.append(f"    writer.write_signed_integer({field['value']});")
                else:
                    constant = (f"static_cast<std::uint64_t>(static_cast<{field['type']}>({field['value']}))"
                                if field["type"] in enum_names else str(field["value"]))
                    lines.append(f"    writer.write_integer({constant});")
            elif field["type"] == "bytes":
                lines.append(f"    writer.write_binary({value});")
            elif field["type"] in {"f32", "f64"}:
                lines.append(f"    writer.write_float64(static_cast<double>({value}));")
            elif field["type"].startswith("i"):
                lines.append(f"    writer.write_signed_integer({value});")
            elif field["type"] in enum_names and enum_storage[field["type"]].startswith("i"):
                lines.append(f"    writer.write_signed_integer(static_cast<std::int64_t>({value}));")
            else:
                expr = f"static_cast<std::uint64_t>({value})" if field["type"] in enum_names else value
                lines.append(f"    writer.write_integer({expr});")
        lines.extend(["    return true;", "}", "", f"bool read(::{reader_namespace}::MessagePackReader &reader, {message.name} &value, std::string *error) {{", "    std::uint32_t field_count = 0;", "    if (!reader.read_map_size(field_count)) {", f'        set_error(error, "{message.name} is not a MessagePack map");', "        return false;", "    }"])
        for field in fields:
            lines.append(f"    bool has_{field['name']} = false;")
        lines.extend(["    std::unordered_set<std::uint64_t> seen_keys;", "    for (std::uint32_t index = 0; index < field_count; ++index) {", "        std::uint64_t key = 0;", "        if (!reader.read_nonnegative(key)) {", '            set_error(error, "wire field key is not a non-negative integer");', "            return false;", "        }", "        if (!seen_keys.insert(key).second) {", f'            set_error(error, "duplicate field ID in {message.name}");', "            return false;", "        }", "        switch (key) {"])
        for field in fields:
            lines.extend([f"        case {field['id']}: {{"])
            if field["constant"]:
                typ = field["type"]
                temp = f"constant_{field['name']}"
                lines.append(f"            {cpp_type(typ)} {temp}{{}};")
                lines.extend(decoder_read(field, temp, schema))
                expected = field["value"]
                expected_expr = f"static_cast<{typ}>({expected})" if typ in enum_names else str(expected)
                lines.append(f"            if ({temp} != {expected_expr}) {{")
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
        lines.append("        default:")
        lines.extend(["            if (!reader.skip()) {", '                set_error(error, "wire message contains an invalid unknown field");', "                return false;", "            }", "            break;", "        }", "    }", "    if (!reader.at_end()) {", f'        set_error(error, "{message.name} MessagePack value has trailing bytes");', "        return false;", "    }"])
        missing = " || ".join(f"!has_{field['name']}" for field in fields)
        lines.extend([f"    if ({missing}) {{", f'        set_error(error, "{message.name} is missing a required field");', "        return false;", "    }"])
        for field in fields:
            if field["nonnegative"]:
                lines.extend([
                    f"    if (value.{field['name']} < 0) {{",
                    f'        set_error(error, "{message.name}.{field["name"]} must be non-negative");',
                    "        return false;",
                    "    }",
                ])
        lines.extend(["    return true;", "}", ""])
    lines.extend([f"}} // namespace {namespace}::detail", ""])
    return "\n".join(lines)
