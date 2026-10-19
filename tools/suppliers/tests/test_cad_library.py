import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

sys.path.insert(0, str(Path(__file__).parents[1]))
import cad_library

STEP = b"ISO-10303-21;\nDATA;\n#1=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));\nENDSEC;END-ISO-10303-21;"


def archive(files):
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as out:
        for name, data in files.items():
            out.writestr(name, data)
    return buffer.getvalue()


class CadTests(unittest.TestCase):
    def test_archive_paths_cannot_escape_content_addressed_storage(self):
        with tempfile.TemporaryDirectory() as directory:
            assets = Path(directory) / "assets"
            result = cad_library.inventory(archive({"../../escape.STP": STEP, "run.exe": b"ignored"}), "https://example.test/a.zip", assets)
            self.assertEqual(len(result["assets"]), 1)
            asset = result["assets"][0]
            self.assertEqual((assets / (asset["sha256"] + ".step")).read_bytes(), STEP)
            self.assertEqual(asset["unit_status"], "declared_mm")
            self.assertFalse((Path(directory) / "escape.STP").exists())
            self.assertFalse((assets / "run.exe").exists())

    def test_html_is_not_accepted_as_step(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                cad_library.inventory(archive({"part.step": b"<html>denied</html>"}), "https://example.test/a.zip", Path(directory))

    def test_expanded_limit_and_large_members(self):
        with tempfile.TemporaryDirectory() as directory:
            payload = archive({"large.step": STEP})
            with patch.object(cad_library, "MAX_EXPANDED", 1):
                with self.assertRaises(ValueError):
                    cad_library.inventory(payload, "https://example.test/a.zip", Path(directory))
            with patch.object(cad_library, "MAX_MEMBER", 1):
                result = cad_library.inventory(payload, "https://example.test/a.zip", Path(directory))
                self.assertEqual(result["members"][0]["status"], "too_large")
                self.assertEqual(result["assets"], [])

    def test_table_links_preserve_exact_sku_identity(self):
        product = {"url": "https://example.test/product/", "description_html":
                   '<table><tr><td>BK 12</td><td><a href="/bk12.zip">CAD</a></td><td>1002</td></tr></table>'}
        contexts = cad_library.link_contexts(product, [{"sku": "1002", "supplier_product_id": 2},
                                                     {"sku": "002", "supplier_product_id": 3}])
        self.assertEqual(contexts["https://example.test/bk12.zip"][0]["variant_ids"], [2])

    def test_offline_download_requires_cache(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(cad_library, "urlopen") as fetch:
            with self.assertRaises(ValueError):
                cad_library.download("https://example.test/a.zip", Path(directory), offline=True)
            fetch.assert_not_called()


if __name__ == "__main__":
    unittest.main()
