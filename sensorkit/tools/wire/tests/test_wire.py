from __future__ import annotations

import copy
import unittest
from pathlib import Path

from tools.wire.parser import ParseError, Parser, parse_file
from tools.wire.validate import ValidationError, normalized, validate_evolution


ROOT = Path(__file__).resolve().parents[3]


class SensorWireSchemaTests(unittest.TestCase):
    def setUp(self) -> None:
        self.schema = parse_file(ROOT / "schema/sensor_wire.nkw")

    def test_canonical_schema_has_expected_message_and_packed_sizes(self) -> None:
        result = normalized(self.schema)
        self.assertEqual(result["packed_structs"]["ImuSampleData"]["size"], 192)
        self.assertEqual(result["packed_structs"]["LidarReturn"]["size"], 12)
        lidar_fields = result["messages"]["LidarScanMessage"]["fields"]
        self.assertEqual(lidar_fields[9]["value"], 12)
        self.assertEqual(len(result["messages"]["PackedFrameMessage"]["fields"]), 12)

    def test_duplicate_field_ids_are_rejected(self) -> None:
        schema = Parser("message Sample { 1 first : u8 1 second : u8 }").parse()
        with self.assertRaisesRegex(ValidationError, "duplicate or reserved field ID"):
            normalized(schema)

    def test_duplicate_message_ids_are_rejected(self) -> None:
        schema = Parser(
            "enum MessageType : u8 { first = 1 second = 2 } "
            "message First { 1 message_type : MessageType constant = first } "
            "message Second { 1 message_type : MessageType constant = first }"
        ).parse()
        with self.assertRaisesRegex(ValidationError, "duplicate or invalid message ID"):
            normalized(schema)

    def test_reserved_ids_cannot_be_reused(self) -> None:
        schema = Parser("message Sample { 1 first : u8 reserved 2..4; 3 third : u8 }").parse()
        with self.assertRaisesRegex(ValidationError, "duplicate or reserved field ID"):
            normalized(schema)
        overlapping = Parser("message Sample { 1 first : u8 reserved 3..6; reserved 6..8; }").parse()
        with self.assertRaisesRegex(ValidationError, "overlapping reserved ranges"):
            normalized(overlapping)

    def test_invalid_constant_default_and_unknown_type_are_rejected(self) -> None:
        bad_default = Parser("message Sample { 1 value : u8 constant = 256 }").parse()
        with self.assertRaisesRegex(ValidationError, "out of range"):
            normalized(bad_default)
        bad_type = Parser("message Sample { 1 value : decimal128 }").parse()
        with self.assertRaisesRegex(ValidationError, "unsupported type"):
            normalized(bad_type)
        bad_message_type = Parser("message Sample { 1 message_type : u8 constant = 1 }").parse()
        with self.assertRaisesRegex(ValidationError, "must use MessageType"):
            normalized(bad_message_type)

    def test_packed_struct_requires_endianness_and_fixed_size_is_computed(self) -> None:
        with self.assertRaises(ParseError):
            Parser("packed Value { part : u32 }").parse()
        schema = Parser("message Sample { 1 value : u8 } packed Value endian little { a : u32 b : u8[3] }").parse()
        self.assertEqual(normalized(schema)["packed_structs"]["Value"]["size"], 7)

    def test_schema_evolution_rejects_changes_to_existing_fields(self) -> None:
        before = normalized(self.schema)
        after = copy.deepcopy(before)
        after["messages"]["ImuSampleMessage"]["fields"][2]["type"] = "u32"
        with self.assertRaisesRegex(ValidationError, "changed field"):
            validate_evolution(after, before)

    def test_schema_evolution_rejects_added_required_fields(self) -> None:
        before = normalized(self.schema)
        after = copy.deepcopy(before)
        after["messages"]["ImuSampleMessage"]["fields"].append(
            {"id": 9, "name": "new_value", "type": "u32", "constant": False, "value": None}
        )
        with self.assertRaisesRegex(ValidationError, "added required fields"):
            validate_evolution(after, before)

    def test_schema_evolution_allows_new_enum_values_and_messages(self) -> None:
        before = normalized(self.schema)
        after = copy.deepcopy(before)
        after["enums"]["MessageType"]["values"]["custom"] = 6
        after["messages"]["CustomMessage"] = {"fields": [], "reserved": []}
        validate_evolution(after, before)


if __name__ == "__main__":
    unittest.main()
