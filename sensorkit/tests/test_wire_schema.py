from __future__ import annotations

import copy
import unittest
from pathlib import Path

from tools.wire.parser import ParseError, Parser, parse_file
from tools.wire.validate import ValidationError, normalized, validate_evolution


ROOT = Path(__file__).resolve().parents[2]


class SensorWireSchemaTests(unittest.TestCase):
    def setUp(self) -> None:
        self.schema = parse_file(ROOT / "sensorkit/schema/sensor_wire.wire.idl")

    def test_canonical_schema_has_expected_message_and_packed_sizes(self) -> None:
        result = normalized(self.schema)
        self.assertEqual(result["packed_structs"]["ImuSampleData"]["size"], 192)
        self.assertEqual(result["packed_structs"]["LidarReturn"]["size"], 12)
        lidar_fields = result["messages"]["LidarScanMessage"]["fields"]
        self.assertEqual(lidar_fields[9]["value"], 12)
        self.assertEqual(len(result["messages"]["PackedFrameMessage"]["fields"]), 12)

    def test_duplicate_field_ids_are_rejected(self) -> None:
        schema = Parser("message Sample { 1 first : u8 1 second : u8 }").parse()
        with self.assertRaisesRegex(ValidationError, "duplicate field ID"):
            normalized(schema)

    def test_duplicate_message_ids_are_rejected(self) -> None:
        schema = Parser(
            "enum MessageType : u8 { first = 1 second = 2 } "
            "message First { 1 message_type : MessageType constant = first } "
            "message Second { 1 message_type : MessageType constant = first }"
        ).parse()
        with self.assertRaisesRegex(ValidationError, "duplicate or invalid message ID"):
            normalized(schema)

    def test_removed_range_and_sizeof_syntax_is_rejected(self) -> None:
        for declaration in (
            "message Sample { 1 first : u8 reserved 2..4; }",
            "message Sample { 1 first : u8 extension 8..10; }",
            "const VALUE : u32 = sizeof(Sample)",
        ):
            with self.subTest(declaration=declaration), self.assertRaises(ParseError):
                Parser(declaration).parse()

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

    def test_integer_ranges_match_haxe_and_nonnegative_constraints(self) -> None:
        bad_unsigned_64 = Parser("message Sample { 1 value : u64 }").parse()
        with self.assertRaisesRegex(ValidationError, "use i64 nonnegative"):
            normalized(bad_unsigned_64)
        negative = Parser("message Sample { 1 value : i64 nonnegative constant = -1 }").parse()
        with self.assertRaisesRegex(ValidationError, "must be non-negative"):
            normalized(negative)
        valid = Parser("message Sample { 1 value : i64 nonnegative 2 count : u32 }").parse()
        fields = normalized(valid)["messages"]["Sample"]["fields"]
        self.assertTrue(fields[0]["nonnegative"])
        self.assertEqual(fields[1]["type"], "u32")

    def test_packed_struct_requires_endianness_and_fixed_size_is_computed(self) -> None:
        with self.assertRaises(ParseError):
            Parser("packed Value { part : u32 }").parse()
        schema = Parser("message Sample { 1 value : u8 } packed Value endian little { a : u32 b : u8[3] }").parse()
        self.assertEqual(normalized(schema)["packed_structs"]["Value"]["size"], 7)

    def test_schema_evolution_rejects_changes_to_existing_fields(self) -> None:
        before = normalized(self.schema)
        after = copy.deepcopy(before)
        after["messages"]["ImuSampleMessage"]["fields"][2]["type"] = "u32"
        with self.assertRaisesRegex(ValidationError, "changed message"):
            validate_evolution(after, before)

    def test_schema_evolution_rejects_any_existing_message_shape_change(self) -> None:
        before = normalized(self.schema)
        after = copy.deepcopy(before)
        after["messages"]["ImuSampleMessage"]["fields"].append(
            {"id": 32, "name": "new_value", "type": "u32", "constant": False, "value": None,
             "nonnegative": False}
        )
        with self.assertRaisesRegex(ValidationError, "changed message"):
            validate_evolution(after, before)

        after = copy.deepcopy(before)
        after["messages"]["ImuSampleMessage"]["fields"][1]["value"] = 5
        with self.assertRaisesRegex(ValidationError, "changed message"):
            validate_evolution(after, before)

    def test_schema_evolution_accepts_legacy_allocation_metadata_in_lock(self) -> None:
        current = normalized(self.schema)
        old = copy.deepcopy(current)
        old["constants"]["CURRENT_FRAME_VERSION"] = 1
        old["messages"]["ImuSampleMessage"]["reserved"] = [[9, 31]]
        old["messages"]["ImuSampleMessage"]["extension"] = [[32, 63]]
        validate_evolution(current, old)

    def test_schema_evolution_keeps_published_constants_and_packed_layouts(self) -> None:
        before = normalized(self.schema)
        after = copy.deepcopy(before)
        after["constants"]["CURRENT_FRAME_VERSION"]["type"] = "u16"
        with self.assertRaisesRegex(ValidationError, "changed constant"):
            validate_evolution(after, before)
        after = copy.deepcopy(before)
        after["packed_structs"]["LidarReturn"]["size"] += 1
        with self.assertRaisesRegex(ValidationError, "changed packed_struct"):
            validate_evolution(after, before)

    def test_schema_evolution_allows_new_enum_values_and_messages(self) -> None:
        before = normalized(self.schema)
        after = copy.deepcopy(before)
        after["enums"]["MessageType"]["values"]["custom"] = 6
        after["messages"]["CustomMessage"] = {"fields": []}
        validate_evolution(after, before)


if __name__ == "__main__":
    unittest.main()
