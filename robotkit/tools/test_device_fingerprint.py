import importlib.util
import json
from pathlib import Path
import unittest

MODULE_PATH = Path(__file__).with_name("device_fingerprint.py")
SPEC = importlib.util.spec_from_file_location("device_fingerprint", MODULE_PATH)
assert SPEC and SPEC.loader
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


class FingerprintTests(unittest.TestCase):
    def test_layout_and_schema_changes_change_both_language_constants(self):
        lock = module.DEFAULT_LOCK.read_bytes()
        layout = b"ordered-joints: left-wheel,right-wheel; channels: 0,1\n"
        value = module.fingerprint(layout, lock)
        self.assertEqual(len(value), 16)
        self.assertEqual(value.hex(), "179b3f58ee7869463d1ffe5d0b8eb143")
        self.assertNotEqual(value, module.fingerprint(layout + b"calibration=2\n", lock))
        edited = json.loads(lock)
        edited["fingerprint_test_extension"] = 1
        self.assertNotEqual(value, module.fingerprint(layout, json.dumps(edited).encode()))
        self.assertEqual(value, module.fingerprint(layout, json.dumps(json.loads(lock),
            sort_keys=True, indent=4).encode()))
        self.assertEqual(module.cpp_constant(value).count("0x"), 16)
        self.assertEqual(module.rust_constant(value).count("0x"), 16)
        with self.assertRaises(ValueError):
            module.fingerprint(b"", lock)


if __name__ == "__main__":
    unittest.main()
