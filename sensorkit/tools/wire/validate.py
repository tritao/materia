"""Semantic checks and normalized schema compatibility rules."""

from __future__ import annotations

import json
from typing import Any

from .model import Schema

PRIMITIVE_RANGES = {
    "u8": (0, 2**8 - 1, 1),
    "u32": (0, 2**32 - 1, 4),
    "u64": (0, 2**64 - 1, 8),
    "f32": (None, None, 4),
    "f64": (None, None, 8),
}


class ValidationError(ValueError):
    pass


def _resolve(value: Any, constants: dict[str, Any], enums: dict[str, dict[str, int]]) -> Any:
    if isinstance(value, tuple) and len(value) == 2:
        enum_name, item_name = value
        if enum_name not in enums or item_name not in enums[enum_name]:
            raise ValidationError(f"unknown enum literal {enum_name}.{item_name}")
        return enums[enum_name][item_name]
    if isinstance(value, str):
        if value in constants:
            return constants[value]
        enum_matches = [items[value] for items in enums.values() if value in items]
        if len(enum_matches) == 1:
            return enum_matches[0]
        raise ValidationError(f"unknown or ambiguous constant {value}")
    return value


def normalized(schema: Schema) -> dict[str, Any]:
    enum_values = {enum.name: {value.name: value.value for value in enum.values} for enum in schema.enums}
    constants = {constant.name: constant.value for constant in schema.constants}
    packed_sizes: dict[str, int] = {}
    for packed in schema.packed_structs:
        size = 0
        for field in packed.fields:
            base_size = PRIMITIVE_RANGES.get(field.type_ref.name, (None, None, None))[2]
            if base_size is None or field.type_ref.length is not None and field.type_ref.length <= 0:
                raise ValidationError(f"{packed.name}.{field.name} has an invalid packed type")
            size += base_size * (field.type_ref.length or 1)
        packed_sizes[packed.name] = size

    def resolved(value: Any) -> Any:
        if isinstance(value, str) and value.startswith("sizeof(") and value.endswith(")"):
            struct_name = value[7:-1]
            if struct_name not in packed_sizes:
                raise ValidationError(f"unknown packed struct {struct_name} in sizeof")
            return packed_sizes[struct_name]
        return _resolve(value, constants, enum_values)

    for constant in schema.constants:
        constants[constant.name] = resolved(constant.value)
    result = {
        "version": 1,
        "constants": {constant.name: resolved(constant.value) for constant in schema.constants},
        "enums": {
            enum.name: {"underlying": enum.underlying, "values": {v.name: v.value for v in enum.values}}
            for enum in schema.enums
        },
        "messages": {},
        "packed_structs": {},
    }
    if len(enum_values) != len(schema.enums):
        raise ValidationError("duplicate enum name")
    for enum in schema.enums:
        if enum.underlying not in PRIMITIVE_RANGES or not enum.underlying.startswith("u"):
            raise ValidationError(f"enum {enum.name} must use an unsigned integer type")
        low, high, _ = PRIMITIVE_RANGES[enum.underlying]
        names: set[str] = set()
        values: set[int] = set()
        for item in enum.values:
            if item.name in names or item.value in values:
                raise ValidationError(f"enum {enum.name} has a duplicate member or value")
            if not isinstance(item.value, int) or not low <= item.value <= high:
                raise ValidationError(f"enum value {enum.name}.{item.name} is out of range")
            names.add(item.name)
            values.add(item.value)
    constant_names = [constant.name for constant in schema.constants]
    if len(constant_names) != len(set(constant_names)):
        raise ValidationError("duplicate constant name")
    for constant in schema.constants:
        if constant.type_name not in PRIMITIVE_RANGES:
            raise ValidationError(f"constant {constant.name} uses unsupported type {constant.type_name}")
        low, high, _ = PRIMITIVE_RANGES[constant.type_name]
        value = resolved(constant.value)
        if constant.type_name.startswith("u") and (not isinstance(value, int) or not low <= value <= high):
            raise ValidationError(f"constant {constant.name} is out of range")
        if constant.type_name.startswith("f") and not isinstance(value, (int, float)):
            raise ValidationError(f"constant {constant.name} is not numeric")

    message_names: set[str] = set()
    message_ids: dict[int, str] = {}
    enum_by_name = {enum.name: enum for enum in schema.enums}
    for message in schema.messages:
        if message.name in message_names:
            raise ValidationError(f"duplicate message {message.name}")
        message_names.add(message.name)
        field_ids: set[int] = set()
        field_names: set[str] = set()
        reserved_ranges = sorted(message.reserved)
        for index, (start, end) in enumerate(reserved_ranges):
            if start <= 0 or end < start or end > 0xFFFFFFFF:
                raise ValidationError(f"{message.name} has an invalid reserved range")
            if index and start <= reserved_ranges[index - 1][1]:
                raise ValidationError(f"{message.name} has overlapping reserved ranges")
        message_fields: list[dict[str, Any]] = []
        for field in message.fields:
            if field.id <= 0 or field.id > 0xFFFFFFFF:
                raise ValidationError(f"{message.name}.{field.name} has an invalid field ID")
            if field.id in field_ids or any(start <= field.id <= end for start, end in reserved_ranges):
                raise ValidationError(f"{message.name} has a duplicate or reserved field ID {field.id}")
            if field.name in field_names:
                raise ValidationError(f"{message.name} has a duplicate field name {field.name}")
            field_ids.add(field.id)
            field_names.add(field.name)
            typ = field.type_ref.name
            if field.type_ref.length is not None:
                raise ValidationError(f"message field {message.name}.{field.name} cannot be a fixed array")
            if typ != "bytes" and typ not in PRIMITIVE_RANGES and typ not in enum_by_name:
                raise ValidationError(f"{message.name}.{field.name} uses unsupported type {typ}")
            if field.constant and field.value is None:
                raise ValidationError(f"constant field {message.name}.{field.name} needs a value")
            value = None
            if field.constant:
                value = resolved(field.value)
                if typ in PRIMITIVE_RANGES and typ.startswith("u"):
                    low, high, _ = PRIMITIVE_RANGES[typ]
                    if not isinstance(value, int) or not low <= value <= high:
                        raise ValidationError(f"constant field {message.name}.{field.name} is out of range")
                elif typ in enum_by_name:
                    valid_values = {item.value for item in enum_by_name[typ].values}
                    if value not in valid_values:
                        raise ValidationError(f"constant field {message.name}.{field.name} is not an enum value")
                else:
                    raise ValidationError(f"constant field {message.name}.{field.name} has an invalid type")
            message_fields.append({"id": field.id, "name": field.name, "type": typ, "constant": field.constant, "value": value})
        message_fields.sort(key=lambda item: item["id"])
        type_field = next((f for f in message.fields if f.name == "message_type" and f.constant), None)
        if type_field is not None:
            if type_field.type_ref.name != "MessageType":
                raise ValidationError(f"{message.name}.message_type must use MessageType")
            resolved_id = resolved(type_field.value)
            if not isinstance(resolved_id, int) or resolved_id in message_ids:
                raise ValidationError(f"duplicate or invalid message ID {resolved_id}")
            message_ids[resolved_id] = message.name
        result["messages"][message.name] = {
            "fields": message_fields,
            "reserved": [[start, end] for start, end in reserved_ranges],
        }

    for packed in schema.packed_structs:
        if packed.endian not in {"little", "big"}:
            raise ValidationError(f"packed struct {packed.name} needs explicit little or big endian")
        if packed.name in result["packed_structs"]:
            raise ValidationError(f"duplicate packed struct {packed.name}")
        names: set[str] = set()
        fields: list[dict[str, Any]] = []
        for field in packed.fields:
            if field.name in names:
                raise ValidationError(f"{packed.name} has duplicate field {field.name}")
            names.add(field.name)
            if field.type_ref.name not in PRIMITIVE_RANGES:
                raise ValidationError(f"{packed.name}.{field.name} uses unsupported packed type")
            if field.type_ref.length is not None and not 0 < field.type_ref.length <= 1_000_000:
                raise ValidationError(f"{packed.name}.{field.name} has an invalid fixed-array length")
            fields.append({"name": field.name, "type": field.type_ref.name, "length": field.type_ref.length or 1})
        size = packed_sizes[packed.name]
        if size <= 0:
            raise ValidationError(f"packed struct {packed.name} has zero size")
        result["packed_structs"][packed.name] = {"endian": packed.endian, "fields": fields, "size": size}

    if not message_names or len(message_names) != len(schema.messages):
        raise ValidationError("schema must declare uniquely named messages")
    return result


def validate_evolution(current: dict[str, Any], old: dict[str, Any]) -> None:
    """Reject wire-breaking edits while permitting new enums and messages."""
    if current.get("version") != old.get("version"):
        raise ValidationError("schema evolution changed the normalized format version")
    for group in ("constants", "enums", "messages", "packed_structs"):
        previous = old.get(group, {})
        now = current.get(group, {})
        for name, previous_value in previous.items():
            if name not in now:
                raise ValidationError(f"schema evolution removed {group[:-1]} {name}")
            new_value = now[name]
            if group == "enums":
                if new_value.get("underlying") != previous_value.get("underlying"):
                    raise ValidationError(f"schema evolution changed enum storage for {name}")
                for member, value in previous_value.get("values", {}).items():
                    if new_value.get("values", {}).get(member) != value:
                        raise ValidationError(f"schema evolution changed enum value {name}.{member}")
            elif group == "messages":
                for old_field in previous_value.get("fields", []):
                    if old_field not in new_value.get("fields", []):
                        raise ValidationError(f"schema evolution changed field {name}.{old_field['name']}")
                if any(not any(new_lo <= old_lo and old_hi <= new_hi
                               for new_lo, new_hi in new_value.get("reserved", []))
                       for old_lo, old_hi in previous_value.get("reserved", [])):
                    raise ValidationError(f"schema evolution unreserved field IDs in {name}")
                if len(new_value.get("fields", [])) > len(previous_value.get("fields", [])):
                    raise ValidationError(f"schema evolution added required fields to {name}; define a new message")
            elif new_value != previous_value:
                raise ValidationError(f"schema evolution changed {group[:-1]} {name}")


def dump_normalized(value: dict[str, Any]) -> str:
    return json.dumps(value, indent=2, sort_keys=True) + "\n"
