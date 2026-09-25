from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tools.wire.backends.cpp import cpp_header
from tools.wire.backends.haxe import haxe_files
from tools.wire.backends.markdown import docs
from tools.wire.parser import Parser
from tools.wire.render import plan
from tools.wire.validate import ValidationError, normalized


class BackendConfigurationTests(unittest.TestCase):
    def test_packed_only_schema_and_lock_output(self) -> None:
        schema = Parser("const VERSION : u8 = 5 packed Target endian little { joint : u16 value : f32 }").parse()
        norm = normalized(schema)
        self.assertEqual(norm["messages"], {})
        self.assertEqual(norm["packed_structs"]["Target"]["size"], 6)
        with TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            outputs = plan(schema, norm, {"lock": "schema/target.lock.json"}, root)
            self.assertEqual(list(outputs), [root / "schema/target.lock.json"])

    def test_output_cannot_escape_manifest_directory(self) -> None:
        schema = Parser("packed Target endian little { value : u8 }").parse()
        with TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValidationError, "escapes manifest directory"):
                plan(schema, normalized(schema), {"lock": "../outside.json"}, Path(directory).resolve())

    def test_independent_schema_uses_supplied_names(self) -> None:
        schema = Parser("message Probe { 1 value : u16 }").parse()
        norm = normalized(schema)
        generated = haxe_files(
            schema, norm, root=Path("probe/generated"), package="probe.wire",
            codec_name="ProbeCodec", constants_name="ProbeConstants",
            schema_label="probe.wire.idl", codec_comment="/** Probe payload codec. */",
        )
        self.assertIn("package probe.wire;", generated[Path("probe/generated/Probe.hx")])
        self.assertIn("class ProbeCodec", generated[Path("probe/generated/ProbeCodec.hx")])
        self.assertIn("namespace probe::wire", cpp_header(schema, norm, namespace="probe::wire", aliases={}))
        self.assertIn("# Probe wire", docs(schema, norm, title="Probe wire", intro="Probe payloads.", codec_name="ProbeCodec"))


if __name__ == "__main__":
    unittest.main()
