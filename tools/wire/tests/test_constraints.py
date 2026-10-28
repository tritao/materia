from __future__ import annotations

import shutil
import subprocess
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tools.wire.backends.packed_cpp import render as cpp_render
from tools.wire.backends.rust import render as rust_render
from tools.wire.parser import ParseError, Parser
from tools.wire.validate import ValidationError, normalized

PROBE = """
enum Mode : u8 { idle = 0 run = 2 }
packed Probe endian little {
    level : u8 range 0..11
    delta : i16 range -5..5
    flags : u8 mask 0x03
    mode : Mode
    text : u8[4] text
    reserved : u8[2] zero
    plain : u16
}
"""


def norm(source: str):
    schema = Parser(source).parse()
    return schema, normalized(schema)


class ConstraintTests(unittest.TestCase):
    def test_fields_named_like_modifiers_parse(self) -> None:
        _, n = norm("packed P endian little { index : u8 range 1..2 text : u8[2] text mask : u8 mask 0x0f zero : u8 zero range : u8 }")
        fields = {f["name"]: f for f in n["packed_structs"]["P"]["fields"]}
        self.assertEqual(fields["index"]["range"], [1, 2])
        self.assertTrue(fields["text"]["text"])
        self.assertEqual(fields["mask"]["mask"], 15)
        self.assertTrue(fields["zero"]["zero"])
        self.assertNotIn("range", fields["range"])

    def test_lock_is_unchanged_without_constraints(self) -> None:
        _, n = norm("packed P endian little { a : u8 b : u16[2] }")
        self.assertEqual(n["packed_structs"]["P"]["fields"], [
            {"name": "a", "type": "u8", "length": 1}, {"name": "b", "type": "u16", "length": 2}])

    def test_enum_field_stores_its_underlying_type(self) -> None:
        _, n = norm(PROBE)
        mode = next(f for f in n["packed_structs"]["Probe"]["fields"] if f["name"] == "mode")
        self.assertEqual((mode["type"], mode["enum"]), ("u8", "Mode"))
        self.assertEqual(n["packed_structs"]["Probe"]["size"], 1 + 2 + 1 + 1 + 4 + 2 + 2)

    def test_rejects_invalid_constraints(self) -> None:
        for source in (
            "packed P endian little { a : u8 range 0..300 }",
            "packed P endian little { a : u8 range 5..1 }",
            "packed P endian little { a : f32 range 0..1 }",
            "packed P endian little { a : i8 mask 0x01 }",
            "packed P endian little { a : u8 mask 0x1ff }",
            "packed P endian little { a : f32 zero }",
            "packed P endian little { a : u16 text }",
            "packed P endian little { a : u8 text }",
            "enum E : u8 { x = 0 } packed P endian little { a : E[2] }",
        ):
            with self.assertRaises(ValidationError, msg=source):
                norm(source)
        with self.assertRaises(ParseError):
            Parser("packed P endian little { a : u8 range 0..1 zero }").parse()

    @unittest.skipUnless(shutil.which("c++") and shutil.which("rustc"), "C++ and Rust compilers required")
    def test_generated_codecs_enforce_constraints(self) -> None:
        schema, n = norm(PROBE)
        with TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "probe.hpp").write_text(cpp_render(schema, n, "probe"))
            (root / "main.cpp").write_text('''
#include "probe.hpp"
#include <array>
#include <cassert>
int main() {
    probe::Probe v{}; v.level = 11; v.delta = -5; v.flags = 3; v.mode = 2; v.text = {'a','b',0,0};
    std::array<unsigned char, probe::Probe::SIZE> b{};
    assert(probe::encode(v, b));
    probe::Probe d{}; assert(probe::decode(b, d) && d.delta == -5);
    auto bad = [&](auto mutate) { auto c = b; mutate(c); probe::Probe x{}; return !probe::decode(c, x); };
    assert(bad([](auto &c) { c[0] = 12; }));          // range
    assert(bad([](auto &c) { c[1] = 6; c[2] = 0; })); // signed range
    assert(bad([](auto &c) { c[3] = 4; }));           // mask
    assert(bad([](auto &c) { c[4] = 1; }));           // enum
    assert(bad([](auto &c) { c[5] = 1; }));           // non-printable character in text
    return 0;
}
''')
            subprocess.run(["c++", "-std=c++20", "-Wall", "-Wextra", "-Werror", str(root / "main.cpp"), "-o", str(root / "cpp")], check=True)
            subprocess.run([str(root / "cpp")], check=True)
            rust = rust_render(schema, n) + '''
#[cfg(test)] mod tests {
    use super::*;
    #[test] fn constraints() {
        let v = Probe { level: 11, delta: -5, flags: 3, mode: 2, text: [b'a', b'b', 0, 0], reserved: [0; 2], plain: 7 };
        let mut b = [0u8; Probe::SIZE];
        assert_eq!(v.encode(&mut b), Ok(Probe::SIZE));
        assert_eq!(Probe::decode(&b), Ok(v));
        let bad = |f: &dyn Fn(&mut [u8; Probe::SIZE])| { let mut c = b; f(&mut c); Probe::decode(&c) };
        assert_eq!(bad(&|c| c[0] = 12), Err(Error::Constraint("Probe.level")));
        assert_eq!(bad(&|c| { c[1] = 6; c[2] = 0; }), Err(Error::Constraint("Probe.delta")));
        assert_eq!(bad(&|c| c[3] = 4), Err(Error::Constraint("Probe.flags")));
        assert_eq!(bad(&|c| c[4] = 1), Err(Error::Constraint("Probe.mode")));
        assert_eq!(bad(&|c| c[5] = 1), Err(Error::Constraint("Probe.text")));
        assert_eq!(bad(&|c| c[8] = b'x'), Err(Error::Constraint("Probe.text")));
        assert_eq!(bad(&|c| c[9] = 1), Err(Error::Constraint("Probe.reserved")));
        let mut wrong = v; wrong.level = 12;
        assert_eq!(wrong.encode(&mut b), Err(Error::Constraint("Probe.level")));
    }
}
'''
            (root / "probe.rs").write_text(rust)
            subprocess.run(["rustc", "--test", "--cfg", "x", str(root / "probe.rs"), "-o", str(root / "rust")], check=True)
            subprocess.run([str(root / "rust")], check=True)


if __name__ == "__main__":
    unittest.main()
