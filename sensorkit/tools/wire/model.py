"""Data model for the small SensorKit wire IDL."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class TypeRef:
    name: str
    length: int | None = None


@dataclass(frozen=True)
class Constant:
    name: str
    type_name: str
    value: Any


@dataclass(frozen=True)
class EnumValue:
    name: str
    value: int


@dataclass
class Enum:
    name: str
    underlying: str
    values: list[EnumValue] = field(default_factory=list)


@dataclass(frozen=True)
class Field:
    id: int
    name: str
    type_ref: TypeRef
    constant: bool = False
    value: Any = None
    nonnegative: bool = False


@dataclass
class Message:
    name: str
    fields: list[Field] = field(default_factory=list)


@dataclass(frozen=True)
class PackedField:
    name: str
    type_ref: TypeRef


@dataclass
class PackedStruct:
    name: str
    endian: str
    fields: list[PackedField] = field(default_factory=list)


@dataclass
class Schema:
    constants: list[Constant] = field(default_factory=list)
    enums: list[Enum] = field(default_factory=list)
    messages: list[Message] = field(default_factory=list)
    packed_structs: list[PackedStruct] = field(default_factory=list)
