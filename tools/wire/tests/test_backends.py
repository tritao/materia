from __future__ import annotations

import unittest
from pathlib import Path

from tools.wire.backends.cpp import cpp_header
from tools.wire.backends.haxe import haxe_files
from tools.wire.backends.markdown import docs
from tools.wire.parser import Parser
from tools.wire.validate import normalized


class BackendConfigurationTests(unittest.TestCase):
    def test_independent_schema_uses_supplied_names(self) -> None:
        schema = Parser("message Probe { 1 value : u16 }").parse()
        norm = normalized(schema)
        generated = haxe_files(
            schema, norm, root=Path("probe/generated"), package="probe.wire",
            codec_name="ProbeCodec", constants_name="ProbeConstants",
            schema_label="probe.nkw", codec_comment="/** Probe payload codec. */",
        )
        self.assertIn("package probe.wire;", generated[Path("probe/generated/Probe.hx")])
        self.assertIn("class ProbeCodec", generated[Path("probe/generated/ProbeCodec.hx")])
        self.assertIn("namespace probe::wire", cpp_header(schema, norm, namespace="probe::wire", aliases={}))
        self.assertIn("# Probe wire", docs(schema, norm, title="Probe wire", intro="Probe payloads.", codec_name="ProbeCodec"))


if __name__ == "__main__":
    unittest.main()
