"""Shared type mappings for generated language backends."""

from __future__ import annotations

PRIMITIVE_SIZE = {"u8": 1, "u16": 2, "u32": 4, "u64": 8,
                  "i8": 1, "i16": 2, "i32": 4, "i64": 8, "f32": 4, "f64": 8}
STORAGE_TYPE = {
    "u8": "std::uint8_t", "u16": "std::uint16_t", "u32": "std::uint32_t", "u64": "std::uint64_t",
    "i8": "std::uint8_t", "i16": "std::uint16_t", "i32": "std::uint32_t", "i64": "std::uint64_t",
    "f32": "std::uint32_t", "f64": "std::uint64_t",
}

def lower_camel(name: str) -> str:
    parts = name.split("_")
    return parts[0] + "".join(part[:1].upper() + part[1:] for part in parts[1:])


def cpp_type(type_name: str) -> str:
    return {
        "u8": "std::uint8_t",
        "u16": "std::uint16_t",
        "u32": "std::uint32_t",
        "u64": "std::uint64_t",
        "i8": "std::int8_t",
        "i16": "std::int16_t",
        "i32": "std::int32_t",
        "i64": "std::int64_t",
        "f32": "float",
        "f64": "double",
        "bytes": "std::span<const std::uint8_t>",
    }.get(type_name, type_name)


def haxe_type(type_name: str) -> str:
    return {"u32": "Int64", "u64": "Int64", "i64": "Int64", "f32": "Float", "f64": "Float", "bytes": "Bytes"}.get(type_name, "Int")
