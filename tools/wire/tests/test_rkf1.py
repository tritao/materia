import copy
import tempfile
import unittest
from pathlib import Path

from tools.wire.rkf1 import check, scan
from tools.wire.validate import ValidationError

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "robotkit/remote/haxe/robotkit/protocol"


class Rkf1LockTests(unittest.TestCase):
    def test_new_id_allowed_but_changed_id_rejected(self):
        old = {"version": 1, "declarations": {"Hello": {"kind": "class", "fields": [
            {"id": 1, "name": "version", "type": "Int"}]}}, "messageTypes": {"Hello": 1}}
        added = copy.deepcopy(old)
        added["declarations"]["Hello"]["fields"].append(
            {"id": 2, "name": "subscriptions", "type": "Array<String>"})
        check(added, old)
        changed = copy.deepcopy(added)
        changed["declarations"]["Hello"]["fields"][0]["id"] = 3
        with self.assertRaisesRegex(ValidationError, "Hello.@id\\(1\\)"):
            check(changed, old)

    def test_scan_real_source_mutations(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            for source in SOURCE.glob("*.hx"):
                (directory / source.name).write_bytes(source.read_bytes())
            baseline = scan(directory)
            hello = directory / "Hello.hx"
            original = hello.read_text()
            self.assertIn("@:id(1)", original)
            hello.write_text(original.replace("@:id(1) public var protocolVersion",
                                              "@:id(99) public var protocolVersion", 1))
            with self.assertRaisesRegex(ValidationError, "Hello.@id"):
                check(scan(directory), baseline)
            hello.write_text(original)
            first_type = baseline["declarations"]["Hello"]["fields"][0]["type"]
            hello.write_text(original.replace("protocolVersion:" + first_type + ";",
                                              "protocolVersion:String;", 1))
            with self.assertRaisesRegex(ValidationError, "Hello.@id"):
                check(scan(directory), baseline)
            hello.write_text(original)
            hello.unlink()
            with self.assertRaisesRegex(ValidationError, "declaration Hello"):
                check(scan(directory), baseline)
            hello.write_text(original)
            message = directory / "RobotMessageType.hx"
            original_message = message.read_text()
            name, value = next(iter(baseline["messageTypes"].items()))
            import re
            message.write_text(re.sub(r"(var\s+" + name + r"\s*=\s*)(?:0x[\da-fA-F]+|\d+)",
                                      r"\g<1>0x7fff", original_message, count=1))
            with self.assertRaisesRegex(ValidationError, "message type"):
                check(scan(directory), baseline)
            message.write_text(original_message)
            hello.write_text("// @:id(800) public var fake:Int;\n" + original +
                             "\n/* @:wire class Ghost { @:id(1) public var x:Int; } */\n")
            check(scan(directory), baseline)
            hello.write_text(original + "\n@:wire class Extra { @:id(1) public var x:haxe.Int64; }\n")
            value = scan(directory)
            self.assertEqual(value["declarations"]["Extra"]["fields"][0]["type"], "Int64")
            self.assertIn("Hello", value["declarations"])


if __name__ == "__main__":
    unittest.main()
