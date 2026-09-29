import copy
import unittest

from tools.wire.rkf1 import check
from tools.wire.validate import ValidationError


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


if __name__ == "__main__":
    unittest.main()
