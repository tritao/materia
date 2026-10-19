import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import audit


def catalog():
    return {"products": [{"supplier_product_id": 1, "sku": "1131500", "name": "Socket Bolts INOX A2",
        "url": "https://vallder.com/product/socket-bolts-inox-a2/", "attributes": [{"terms": [
            {"name": "Socket-Bolt-M6x20-INOX-A2", "slug": "m6x20"}]}],
        "specification_tables": [[["Product", "d", "L", "dk", "k", "Material / Class", "Standard", "SKU"],
            ["Socket-Bolt-M6x20-INOX-A2", "M6", "20", "9.8", "6", "A2 (304) / A2-70", "EN ISO 4762", "001"]]]}],
        "variants": [{"supplier_product_id": 2, "parent_id": 1, "sku": "001", "url": "https://vallder.com/example/",
            "price": {}, "availability": {}, "source": {"fetched_at": "test"},
            "option_attributes": [{"name": "Variant", "value": "m6x20"}]}]}


class AuditTests(unittest.TestCase):
    def test_mapping_keeps_evidence_and_review(self):
        data = catalog()
        before = copy.deepcopy(data)
        mapping = audit.fastener_mappings(data)[0]
        self.assertEqual(mapping["values"], {"size": "M6", "length": 20.0, "propertyClass": "A2-70"})
        self.assertEqual(mapping["evidence"]["head_diameter_mm"], 9.8)
        self.assertEqual(mapping["compatibility"], "review_required")
        self.assertEqual(data, before)

    def test_rejects_changed_table_sku_or_options(self):
        for field, value in [("sku", "different"), ("option_attributes", [{"value": "m8x20"}])]:
            data = catalog()
            data["variants"][0][field] = value
            with self.assertRaises(ValueError):
                audit.fastener_mappings(data)

    def test_carbon_class_comes_from_supplier_evidence(self):
        data = catalog()
        parent = data["products"][0]
        parent.update({"sku": "1131000", "name": "Socket Bolts",
                       "short_description_html": "<p>Low cost, carbon steel 8.8 grade Socket Bolts</p>"})
        parent["specification_tables"] = [[["Product", "dk", "k", "b", "L", "d", "SKU", "Drawing"],
            ["Socket-Bolt-M6x20", "9.8", "6", "", "20", "M6", "001"]]]
        parent["attributes"][0]["terms"][0]["name"] = "Socket-Bolt-M6x20"
        result = audit.fastener_mappings(data, "1131000")[0]
        self.assertEqual(result["values"]["propertyClass"], "8.8")
        self.assertFalse(result["evidence"]["standard_claimed_by_supplier"])
        parent["short_description_html"] = "carbon steel, class unknown"
        with self.assertRaises(ValueError):
            audit.fastener_mappings(data, "1131000")

    def test_m8_head_conflict_is_reported_without_changing_recipe(self):
        data = catalog()
        parent = data["products"][0]
        row = parent["specification_tables"][0][1]
        row[0:5] = ["Socket-Bolt-M8x20-INOX-A2", "M8", "20", "12.3", "8"]
        parent["attributes"][0]["terms"][0].update({"name": row[0], "slug": "m8x20"})
        data["variants"][0]["option_attributes"][0]["value"] = "m8x20"
        result = audit.fastener_mappings(data)[0]
        self.assertFalse(result["head_dimension_check"]["diameter_within_reference"])
        self.assertEqual(result["values"], {"size": "M8", "length": 20.0, "propertyClass": "A2-70"})
        self.assertEqual(result["compatibility"], "review_required")

    def test_archive_is_candidate_not_verified_step(self):
        data = {"name": "Profile", "supplier_product_id": 1, "url": "https://vallder.com/product/profile/",
                "categories": [{"slug": "aluminum-profiles"}], "description_links": ["/cad.zip?version=1"]}
        result = audit.audit_product(data, 3)
        self.assertEqual(result["status"], "inspect_linked_cad")
        self.assertFalse(result["cad_download_verified"])
        self.assertEqual(result["links"]["archive"], ["https://vallder.com/cad.zip?version=1"])
        self.assertIn("machinekit.structural.TSlotExtrusion", result["candidate_families"])


if __name__ == "__main__":
    unittest.main()
