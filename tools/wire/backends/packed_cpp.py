"""Self-contained C++20 codecs for fixed packed layouts."""

from __future__ import annotations

from typing import Any

from ..model import Schema
from .common import cpp_type


def _constrained(item: dict[str, Any]) -> list[dict[str, Any]]:
    return [f for f in item["fields"] if set(f) - {"name", "type", "length"}]


def _violation(field: dict[str, Any], enums: dict[str, list[int]], arrays: set[str], target: str) -> str:
    """C++ condition that is true when the field breaks its constraint."""
    name = field["name"]
    def each(body: str) -> str:
        if name not in arrays:
            return body.replace("@", f"{target}.{name}")
        return f"[&] {{ for (const auto v : {target}.{name}) if ({body.replace('@', 'v')}) return true; return false; }}()"
    if "range" in field:
        low, high = field["range"]
        bits = int(field["type"][1:])
        unsigned = field["type"].startswith("u")
        natural = (0, 2**bits - 1) if unsigned else (-(2**(bits - 1)), 2**(bits - 1) - 1)
        # Omit a bound the type already guarantees, which would trip -Wtype-limits.
        suffix, cast = ("ULL", "") if field["type"] == "u64" else ("LL", "static_cast<std::int64_t>")
        tests = []
        if low > natural[0]:
            tests.append(f"{cast}(@) < {low}{suffix}")
        if high < natural[1]:
            tests.append(f"{cast}(@) > {high}{suffix}")
        return each(" || ".join(tests)) if tests else "false"
    if "mask" in field:
        return each(f"(static_cast<std::uint64_t>(@) & ~static_cast<std::uint64_t>({field['mask']}ULL)) != 0")
    if "zero" in field:
        return each("@ != 0")
    if "enum" in field:
        return each("!(" + " || ".join(f"@ == {v}" for v in enums[field["enum"]]) + ")")
    f = f"{target}.{name}"
    return (f"[&] {{ std::size_t end = 0; while (end < {f}.size() && {f}[end] != 0) ++end; "
            f"for (std::size_t i = 0; i < end; ++i) if ({f}[i] < 32 || {f}[i] > 126) return true; "
            f"for (std::size_t i = end; i < {f}.size(); ++i) if ({f}[i] != 0) return true; return false; }}()")


def render(schema: Schema, norm: dict[str, Any], namespace: str) -> str:
    enums = {enum.name: [item.value for item in enum.values] for enum in schema.enums}
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
        constrained = _constrained(item)
        if constrained:
            lines.append(f"/// Constraints declared in the schema; checked on encode and decode.")
            lines.append(f"inline bool valid(const {packed.name} &value) {{")
            for field in constrained:
                lines.append(f"    if ({_violation(field, enums, array_fields, 'value')}) return false;")
            lines.extend(["    return true;", "}", ""])
        lines.extend([
            f"inline bool encode(const {packed.name} &value, std::span<std::uint8_t> out) {{",
            f"    if (out.size() < {packed.name}_SIZE) return false;",
        ] + (["    if (!valid(value)) return false;"] if constrained else []) + [
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
        lines.extend([f"    return {'valid(value)' if constrained else 'true'};", "}", ""])
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
        lines.extend([f"    return {'valid(value)' if constrained else 'true'};", "}", ""])
    lines.extend([f"}} // namespace {namespace}", ""])
    return "\n".join(lines)
