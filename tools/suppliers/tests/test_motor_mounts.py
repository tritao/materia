import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).parents[1]))
import motor_mounts


def fixture():
    base = 'https://example.test/'
    table = [['Model', 'RBK12-57-motor-bracket', 'RBK15-86-motor-bracket'],
             ['SKU', 'first', 'second'], ['Download 3D Model', '12-57-3D', '15-86-3D'],
             ['Bearing', '6001ZZ', '6002ZZ'], ['Angular Bearing (optional)', '7001AC', '7002AC'],
             ['Ballscrew Size', 'SFU16xx', 'SFU20xx']]
    catalog = {'products': [{'supplier_product_id': 6718, 'url': base,
                            'specification_tables': [table],
                            'description_links': [base+'12-57-3D.zip', base+'15-86-3D.zip']}],
               'variants': [{'parent_id': 6718, 'sku': sku, 'supplier_product_id': idx,
                             'option_attributes': [{'name': 'Variant', 'value': model.lower()}],
                             'source': {'fetched_at': 'snapshot'}}
                            for sku, idx, model in [('first', 1, table[0][1]), ('second', 2, table[0][2])]]}
    assets = [{'product_id': 6718, 'source_url': base+'12-57-3D.zip', 'variant_ids': [],
               'table_evidence': [], 'usable_geometry': True, 'file': 'cad/assets/one.step', 'sha256': 'one'}]
    return catalog, assets


class MotorMountTests(unittest.TestCase):
    def test_transposed_columns_bind_only_exact_model_sku_and_file(self):
        catalog, assets = fixture()
        bindings = motor_mounts.bind_rbk(catalog, assets)
        self.assertEqual(assets[0]['variant_ids'], [1])
        self.assertEqual(bindings[0]['sku'], 'first')
        self.assertEqual(bindings[0]['cad'][0]['sha256'], 'one')
        self.assertFalse(bindings[0]['compatibility_verified'])
        self.assertEqual(bindings[1]['status'], 'geometry_gap')
        self.assertEqual(bindings[1]['cad'], [])
        motor_mounts.bind_rbk(catalog, assets)
        self.assertEqual(len(assets[0]['table_evidence']), 1)

    def test_changed_or_ambiguous_identity_fails_closed(self):
        original, assets = fixture()
        variants = []
        c = copy.deepcopy(original);c['products'][0]['specification_tables'][0][2][1] = '15-86-3D';variants.append(c)
        c = copy.deepcopy(original);c['variants'][0]['option_attributes'][0]['value'] = 'rbk15-86-motor-bracket';variants.append(c)
        c = copy.deepcopy(original);c['products'][0]['specification_tables'][0][1].pop();variants.append(c)
        for catalog in variants:
            with self.assertRaises(ValueError):
                motor_mounts.bind_rbk(catalog, copy.deepcopy(assets))

    def test_unusable_cad_does_not_become_a_usable_binding(self):
        catalog, assets = fixture()
        assets[0]['usable_geometry'] = False
        self.assertEqual(motor_mounts.bind_rbk(catalog, assets)[0]['cad'], [])
        catalog, assets = fixture()
        duplicate = copy.deepcopy(assets[0]);duplicate['sha256'] = 'other'
        with self.assertRaises(ValueError):
            motor_mounts.bind_rbk(catalog, assets+[duplicate])


if __name__ == '__main__':
    unittest.main()
