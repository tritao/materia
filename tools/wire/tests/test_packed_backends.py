from __future__ import annotations

import shutil
import subprocess
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tools.wire.backends.packed_cpp import render as cpp_render
from tools.wire.backends.rust import render as rust_render
from tools.wire.parser import Parser
from tools.wire.validate import normalized


class PackedBackendTests(unittest.TestCase):
    def test_one_element_array_keeps_array_type(self) -> None:
        schema = Parser("packed One endian little { values : u8[1] }").parse()
        norm = normalized(schema)
        self.assertIn("std::array<std::uint8_t, 1> values", cpp_render(schema, norm, "probe"))
        self.assertIn("pub values: [u8; 1]", rust_render(schema, norm))

    @unittest.skipUnless(shutil.which("c++") and shutil.which("rustc"), "C++ and Rust compilers required")
    def test_big_endian_signed_array_and_float_vectors(self) -> None:
        schema = Parser("packed Probe endian big { signed_value : i16 samples : u16[2] value : f32 }").parse()
        norm = normalized(schema)
        self.assertEqual(norm["packed_structs"]["Probe"]["size"], 10)
        with TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "probe.hpp").write_text(cpp_render(schema, norm, "probe"))
            (root / "main.cpp").write_text(f'''
#include "probe.hpp"
#include <array>
#include <cassert>
int main() {{
    probe::Probe value{{-2, {{0x1234, 0xabcd}}, 1.5f}};
    std::array<unsigned char, probe::Probe::SIZE> bytes{{}};
    assert(probe::encode(value, bytes));
    const std::array<unsigned char, 10> expected{{0xff,0xfe,0x12,0x34,0xab,0xcd,0x3f,0xc0,0,0}};
    assert(bytes == expected);
    probe::Probe decoded{{}};
    assert(probe::decode(bytes, decoded));
    assert(decoded.signed_value == -2 && decoded.samples[1] == 0xabcd && decoded.value == 1.5f);
}}
''')
            subprocess.run(["c++", "-std=c++20", "-Wall", "-Wextra", "-Werror", str(root / "main.cpp"), "-o", str(root / "cpp")], check=True)
            subprocess.run([str(root / "cpp")], check=True)
            rust = rust_render(schema, norm)
            rust += f'''
#[cfg(test)] mod tests {{
    use super::*;
    #[test] fn canonical() {{
        let value = Probe {{ signed_value: -2, samples: [0x1234, 0xabcd], value: 1.5 }};
        let mut bytes = [0u8; Probe::SIZE];
        assert_eq!(value.encode(&mut bytes), Ok(10));
        assert_eq!(bytes, [0xff,0xfe,0x12,0x34,0xab,0xcd,0x3f,0xc0,0,0]);
        assert_eq!(Probe::decode(&bytes), Ok(value));
    }}
}}
'''
            (root / "probe.rs").write_text(rust)
            subprocess.run(["rustc", "--test", str(root / "probe.rs"), "-o", str(root / "rust")], check=True)
            subprocess.run([str(root / "rust")], check=True)


if __name__ == "__main__":
    unittest.main()
