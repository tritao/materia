import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("vallder", Path(__file__).parents[1] / "vallder.py")
vallder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vallder)


class ImportTests(unittest.TestCase):
    def test_price_and_variant_options(self):
        product = {"id": 2, "parent": 1, "type": "variation", "name": "Bolt", "sku": "001",
                   "permalink": "https://vallder.com/product/bolt/", "prices": {
                       "price": "26", "regular_price": "39", "currency_minor_unit": 2,
                       "currency_code": "EUR"}, "description": "<table><tr><th>Size</th></tr>"
                       "<tr><td>M6 &amp; M8<br>bolt</td></tr></table><a href='drawing.pdf'>Drawing</a>"}
        result = vallder.normalize(product, [{"name": "Variant", "value": "m6"}])
        self.assertEqual(result["price"]["amount"], "0.26")
        self.assertEqual(result["price"]["regular_amount"], "0.39")
        self.assertEqual(result["sku"], "001")
        self.assertEqual(result["option_attributes"][0]["value"], "m6")
        self.assertEqual(result["specification_tables"], [[["Size"], ["M6 & M8 bolt"]]])
        self.assertEqual(result["description_links"], ["drawing.pdf"])
        self.assertIsNone(result["component_id"])

    def test_inconsistent_pagination_rejected(self):
        class FakeClient:
            def get(self, url):
                return {"url": url, "fetched_at": "test", "headers": {
                    "x-wp-total": "2", "x-wp-totalpages": "2"}, "body": '[{"id":1}]'}
        with self.assertRaisesRegex(ValueError, "duplicate"):
            vallder.collection(FakeClient())

    def test_offline_never_fetches(self):
        with tempfile.TemporaryDirectory() as directory:
            client = vallder.Client(Path(directory), offline=True)
            with patch.object(vallder, "urlopen") as fetch:
                with self.assertRaisesRegex(ValueError, "Missing cached"):
                    client.get("https://vallder.com/example")
                fetch.assert_not_called()

    def test_variant_coverage_rejected(self):
        with patch.object(vallder, "collection", side_effect=[
            ([{"id": 1, "variations": [{"id": 2}]}], []), ([], [])
        ]):
            with self.assertRaisesRegex(ValueError, "parent variant references"):
                vallder.import_catalog(None)

    def test_failed_write_keeps_previous_catalog(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "catalog.json"
            vallder.write_json(path, {"previous": True})
            with self.assertRaises(TypeError):
                vallder.write_json(path, {"invalid": object()})
            self.assertEqual(json.loads(path.read_text()), {"previous": True})


if __name__ == "__main__":
    unittest.main()
