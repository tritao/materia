"""Allocation-free Rust codecs for fixed packed layouts only."""

from __future__ import annotations

from typing import Any

from ..model import Schema


def _variant(name: str) -> str:
    return "".join(part[:1].upper() + part[1:] for part in name.split("_"))


def render(schema: Schema, norm: dict[str, Any]) -> str:
    lines = [
        "// Generated from a .wire.idl schema. Do not edit.",
        "#![no_std]", "", "#[derive(Clone, Copy, Debug, PartialEq, Eq)]",
        "pub enum Error { ShortBuffer, WrongLength }", "",
    ]
    for constant in schema.constants:
        value = norm["constants"][constant.name]["value"]
        lines.append(f"pub const {constant.name}: {constant.type_name} = {value};")
    if schema.constants:
        lines.append("")
    for enum in schema.enums:
        lines.extend([f"#[repr({enum.underlying})]", "#[derive(Clone, Copy, Debug, PartialEq, Eq)]", f"pub enum {enum.name} {{"])
        lines.extend(f"    {_variant(item.name)} = {item.value}," for item in enum.values)
        lines.extend(["}", ""])
    for packed in schema.packed_structs:
        item = norm["packed_structs"][packed.name]
        array_fields = {field.name for field in packed.fields if field.type_ref.length is not None}
        lines.extend(["#[derive(Clone, Copy, Debug, PartialEq)]", f"pub struct {packed.name} {{"])
        for field in item["fields"]:
            typ = field["type"]
            if field["name"] in array_fields:
                typ = f"[{typ}; {field['length']}]"
            lines.append(f"    pub {field['name']}: {typ},")
        lines.extend(["}", "", f"impl {packed.name} {{", f"    pub const SIZE: usize = {item['size']};", ""])
        lines.extend(["    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {", "        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }", "        let mut offset = 0usize;"])
        for field in item["fields"]:
            typ = field["type"]
            size = int(typ[1:]) // 8
            expr = f"self.{field['name']}"
            if field["name"] in array_fields:
                lines.append(f"        for value in self.{field['name']} {{")
                expr = "value"
            indent = "            " if field["name"] in array_fields else "        "
            suffix = "le" if packed.endian == "little" else "be"
            lines.extend([f"{indent}out[offset..offset + {size}].copy_from_slice(&{expr}.to_{suffix}_bytes());", f"{indent}offset += {size};"])
            if field["name"] in array_fields:
                lines.append("        }")
        lines.extend(["        Ok(offset)", "    }", "", "    pub fn decode(input: &[u8]) -> Result<Self, Error> {", "        if input.len() != Self::SIZE { return Err(Error::WrongLength); }", "        let mut offset = 0usize;"])
        for field in item["fields"]:
            typ = field["type"]
            size = int(typ[1:]) // 8
            name = field["name"]
            if field["name"] in array_fields:
                lines.append(f"        let mut {name} = [0 as {typ}; {field['length']}];")
                lines.append(f"        for item in &mut {name} {{")
            indent = "            " if field["name"] in array_fields else "        "
            lines.extend([f"{indent}let mut bytes = [0u8; {size}];", f"{indent}bytes.copy_from_slice(&input[offset..offset + {size}]);"])
            suffix = "le" if packed.endian == "little" else "be"
            target = "*item" if field["name"] in array_fields else f"let {name}"
            lines.extend([f"{indent}{target} = {typ}::from_{suffix}_bytes(bytes);", f"{indent}offset += {size};"])
            if field["name"] in array_fields:
                lines.append("        }")
        lines.append("        let _ = offset;")
        members = ", ".join(field["name"] for field in item["fields"])
        lines.extend([f"        Ok(Self {{ {members} }})", "    }", "}", ""])
    return "\n".join(lines)
