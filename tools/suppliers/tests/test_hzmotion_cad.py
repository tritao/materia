import hashlib
import json
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import hzmotion_cad


class HzMotionCadTests(unittest.TestCase):
    def test_only_reviewed_supports_are_enabled(self):
        records = json.loads(hzmotion_cad.MANIFEST.read_text())['assets']
        enabled = [a for a in records if a['interfaceReview']['status'] == 'verified-reference']
        self.assertEqual({a['model'] for a in enabled}, {'BF12', 'BF15'})
        self.assertTrue(all('normalization' in a for a in enabled))
        self.assertTrue(all('normalization' not in a for a in records if a not in enabled))

    def test_verified_step(self):
        content = b'ISO-10303-21;\nEND-ISO-10303-21;'
        record = {'bytes': len(content), 'sha256': hashlib.sha256(content).hexdigest()}
        self.assertEqual(hzmotion_cad.verified(content, record), content)
        with self.assertRaisesRegex(ValueError, 'changed'):
            hzmotion_cad.verified(content + b'changed', record)

    def test_login_response_is_not_a_cad_asset(self):
        with self.assertRaisesRegex(ValueError, 'not STEP'):
            hzmotion_cad.verified(b'<html>Sign in</html>', {})


if __name__ == '__main__':
    unittest.main()
