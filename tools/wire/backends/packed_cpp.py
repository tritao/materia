"""Self-contained C++20 codecs for fixed packed layouts."""

from __future__ import annotations

from typing import Any

from ..model import Schema
from .common import cpp_type


def render(schema: Schema, norm: dict[str, Any], namespace: str) -> str:
    lines = [
        "#pragma once", "", "#include <array>", "#include <bit>",
        "#include <cstddef>", "#include <cstdint>", "#include <span>", "",
        f"namespace {namespace} {{", "",
    ]
    for constant in schema.constants:
        value = norm["constants"][constant.name]["value"]
        lines.append(f"inline constexpr {cpp_type(constant.type_name)} {constant.name} = {value};")
    if schema.constants:
        lines.append("")
    for enum in schema.enums:
        lines.append(f"enum class {enum.name} : {cpp_type(enum.underlying)} {{")
        lines.extend(f"    {item.name} = {item.value}," for item in enum.values)
        lines.extend(["};", ""])
    for packed in schema.packed_structs:
        item = norm["packed_structs"][packed.name]
        array_fields = {field.name for field in packed.fields if field.type_ref.length is not None}
        lines.append(f"inline constexpr std::size_t {packed.name}_SIZE = {item['size']};")
        lines.append(f"struct {packed.name} {{")
        lines.append(f"    static constexpr std::size_t SIZE = {packed.name}_SIZE;")
        for field in item["fields"]:
            typ = cpp_type(field["type"])
            if field["name"] in array_fields:
                typ = f"std::array<{typ}, {field['length']}>"
            lines.append(f"    {typ} {field['name']}{{}};")
        lines.extend(["};", ""])
        lines.extend([
            f"inline bool encode(const {packed.name} &value, std::span<std::uint8_t> out) {{",
            f"    if (out.size() < {packed.name}_SIZE) return false;",
            "    std::size_t offset = 0;",
        ])
        for field in item["fields"]:
            typ = field["type"]
            bits = int(typ[1:])
            storage = f"std::uint{bits}_t"
            count = field["length"]
            if field["name"] in array_fields:
                lines.append(f"    for (std::size_t i = 0; i < {count}; ++i) {{")
            indent = "        " if field["name"] in array_fields else "    "
            member = f"value.{field['name']}[i]" if field["name"] in array_fields else f"value.{field['name']}"
            bits_name = f"bits_{field['name']}"
            expr = f"std::bit_cast<{storage}>({member})" if typ.startswith("f") or typ.startswith("i") else f"static_cast<{storage}>({member})"
            lines.append(f"{indent}const {storage} {bits_name} = {expr};")
            for byte in range(bits // 8):
                shift = byte * 8 if packed.endian == "little" else bits - 8 - byte * 8
                lines.append(f"{indent}out[offset++] = static_cast<std::uint8_t>({bits_name} >> {shift});")
            if field["name"] in array_fields:
                lines.append("    }")
        lines.extend(["    return true;", "}", ""])
        lines.extend([
            f"inline bool decode(std::span<const std::uint8_t> input, {packed.name} &value) {{",
            f"    if (input.size() != {packed.name}_SIZE) return false;",
            "    std::size_t offset = 0;",
        ])
        for field in item["fields"]:
            typ = field["type"]
            bits = int(typ[1:])
            storage = f"std::uint{bits}_t"
            count = field["length"]
            if field["name"] in array_fields:
                lines.append(f"    for (std::size_t i = 0; i < {count}; ++i) {{")
            indent = "        " if field["name"] in array_fields else "    "
            bits_name = f"bits_{field['name']}"
            lines.append(f"{indent}{storage} {bits_name} = 0;")
            for byte in range(bits // 8):
                shift = byte * 8 if packed.endian == "little" else bits - 8 - byte * 8
                lines.append(f"{indent}{bits_name} |= static_cast<{storage}>(input[offset++]) << {shift};")
            member = f"value.{field['name']}[i]" if field["name"] in array_fields else f"value.{field['name']}"
            expr = f"std::bit_cast<{cpp_type(typ)}>({bits_name})" if typ.startswith("f") or typ.startswith("i") else bits_name
            lines.append(f"{indent}{member} = {expr};")
            if field["name"] in array_fields:
                lines.append("    }")
        lines.extend(["    return true;", "}", ""])
    lines.extend([f"}} // namespace {namespace}", ""])
    return "\n".join(lines)
