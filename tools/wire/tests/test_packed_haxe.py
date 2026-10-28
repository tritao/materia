from __future__ import annotations

import copy
import unittest
from pathlib import Path

from tools.wire.backends import packed_haxe
from tools.wire.parser import Parser
from tools.wire.validate import ValidationError, normalized


class PackedHaxeTests(unittest.TestCase):
    def test_renders_records_without_touching_the_lock_data(self) -> None:
        schema = Parser("packed P endian big { a : u16 range 1..9 b : i32[2] }").parse()
        norm = normalized(schema)
        before = copy.deepcopy(norm)
        files = packed_haxe.files(schema, norm, root=Path("out"), package="x.wire", label="p.wire.idl")
        self.assertEqual(norm, before)
        source = files[Path("out/P.hx")]
        self.assertIn("public static inline final SIZE:Int = 10;", source)
        self.assertIn("if (a < 1 || a > 9) throw", source)
        self.assertIn("out.set(offset + 0, (a >> 8) & 0xFF);", source)  # big endian: high byte first

    def test_enums_become_named_int_constants(self) -> None:
        schema = Parser("enum Mode : u8 { idle_now = 0 run = 2 } packed P endian little { m : Mode }").parse()
        files = packed_haxe.files(schema, normalized(schema), root=Path("out"), package="x", label="p")
        source = files[Path("out/Mode.hx")]
        self.assertIn("public static inline final idleNow:Int = 0;", source)
        self.assertIn("public static inline final run:Int = 2;", source)

    def test_rejects_types_haxe_int_cannot_hold(self) -> None:
        for type_name in ("u32", "u64", "i64", "f32"):
            schema = Parser(f"packed P endian little {{ a : {type_name} }}").parse()
            with self.assertRaises(ValidationError, msg=type_name):
                packed_haxe.files(schema, normalized(schema), root=Path("out"), package="x", label="p")


if __name__ == "__main__":
    unittest.main()
