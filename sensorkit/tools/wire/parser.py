"""Lexer and parser for SensorKit's intentionally small wire IDL."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

from .model import Constant, Enum, EnumValue, Field, Message, PackedField, PackedStruct, Schema, TypeRef


TOKEN = re.compile(r"\s+|//[^\n]*|\#[^\n]*|(?:\d+\.\d+)|(?:\d+)|(?:[A-Za-z_][A-Za-z_0-9]*)|\.\.|[^\s]")


class ParseError(ValueError):
    pass


class Parser:
    def __init__(self, source: str, path: str = "<schema>") -> None:
        self.path = path
        self.tokens = [t for t in TOKEN.findall(source) if not t.isspace() and not t.startswith(("//", "#"))]
        self.index = 0

    def fail(self, message: str) -> None:
        token = self.peek() or "<eof>"
        raise ParseError(f"{self.path}:{self.index + 1}: {message}; got {token!r}")

    def peek(self) -> str | None:
        return self.tokens[self.index] if self.index < len(self.tokens) else None

    def take(self) -> str:
        token = self.peek()
        if token is None:
            self.fail("unexpected end of file")
        self.index += 1
        return token

    def expect(self, value: str) -> None:
        actual = self.take()
        if actual != value:
            self.fail(f"expected {value!r}")

    def identifier(self) -> str:
        token = self.take()
        if not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", token):
            self.fail("expected identifier")
        return token

    def integer(self) -> int:
        token = self.take()
        if not token.isdigit():
            self.fail("expected non-negative integer")
        return int(token)

    def literal(self) -> Any:
        if self.peek() == "-":
            self.take()
            token = self.take()
            if token.isdigit():
                return -int(token)
            try:
                return -float(token)
            except ValueError:
                self.fail("expected numeric literal after '-' ")
        token = self.take()
        if token.isdigit():
            return int(token)
        if re.fullmatch(r"\d+\.\d+", token):
            return float(token)
        if re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", token):
            if token == "sizeof" and self.peek() == "(":
                self.take()
                name = self.identifier()
                self.expect(")")
                return f"sizeof({name})"
            if self.peek() == ".":
                self.take()
                return (token, self.identifier())
            return token
        self.fail("expected literal")

    def type_ref(self) -> TypeRef:
        name = self.identifier()
        length = None
        if self.peek() == "[":
            self.take()
            length = self.integer()
            self.expect("]")
        return TypeRef(name, length)

    def parse(self) -> Schema:
        schema = Schema()
        while self.peek() is not None:
            kind = self.take()
            if kind == "const":
                name = self.identifier()
                self.expect(":")
                type_name = self.identifier()
                self.expect("=")
                value = self.literal()
                schema.constants.append(Constant(name, type_name, value))
                self.optional_semicolon()
            elif kind == "enum":
                name = self.identifier()
                self.expect(":")
                underlying = self.identifier()
                self.expect("{")
                values: list[EnumValue] = []
                while self.peek() != "}":
                    item = self.identifier()
                    self.expect("=")
                    values.append(EnumValue(item, self.integer()))
                    self.optional_semicolon()
                self.expect("}")
                schema.enums.append(Enum(name, underlying, values))
            elif kind == "message":
                name = self.identifier()
                self.expect("{")
                fields: list[Field] = []
                reserved: list[tuple[int, int]] = []
                while self.peek() != "}":
                    if self.peek() == "reserved":
                        self.take()
                        start = self.integer()
                        end = start
                        if self.peek() == "..":
                            self.take()
                            end = self.integer()
                        reserved.append((start, end))
                        self.expect(";")
                        continue
                    field_id = self.integer()
                    field_name = self.identifier()
                    self.expect(":")
                    field_type = self.type_ref()
                    constant = self.peek() == "constant"
                    value = None
                    if constant:
                        self.take()
                        self.expect("=")
                        value = self.literal()
                    fields.append(Field(field_id, field_name, field_type, constant, value))
                    self.optional_semicolon()
                self.expect("}")
                schema.messages.append(Message(name, fields, reserved))
            elif kind == "packed":
                name = self.identifier()
                self.expect("endian")
                endian = self.identifier()
                self.expect("{")
                fields: list[PackedField] = []
                while self.peek() != "}":
                    field_name = self.identifier()
                    self.expect(":")
                    fields.append(PackedField(field_name, self.type_ref()))
                    self.optional_semicolon()
                self.expect("}")
                schema.packed_structs.append(PackedStruct(name, endian, fields))
            else:
                self.fail(f"unknown declaration {kind!r}")
        return schema

    def optional_semicolon(self) -> None:
        if self.peek() == ";":
            self.take()


def parse_file(path: Path) -> Schema:
    return Parser(path.read_text(encoding="utf-8"), str(path)).parse()
