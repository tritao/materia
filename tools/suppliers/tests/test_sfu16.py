import copy
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).parents[1]))
import sfu16


def fixture():
    nut = ['SFU 1605'] + [''] * 18
    nut[6:16] = ['28', '27.8', '48', '38', '5.5', '46', '10', '10', 'M6', '40']
    nut[-1] = '1035001'
    bf = ['BF 12', '', '10', '', '', '', '', '25']
    catalog = {'products': [
        {'supplier_product_id': 3028, 'url': 'screw', 'specification_tables': [[['SFU1605', '', '5', '16']]]},
        {'supplier_product_id': 3061, 'url': 'nut', 'specification_tables': [[nut]]},
        {'supplier_product_id': 2378, 'url': 'support', 'specification_tables': [[bf]]}],
        'variants': [
            {'parent_id': 3028, 'sku': 'screw400', 'supplier_product_id': 1,
             'option_attributes': [{'value': 'sfu-1605-400mm'}]},
            {'parent_id': 3061, 'sku': '1035001', 'supplier_product_id': 3062},
            {'parent_id': 2378, 'sku': '1013002', 'supplier_product_id': 2380}]}
    assets = [{'sha256': sfu16.BF12_HASH, 'usable_geometry': True, 'variant_ids': [2380]}]
    return catalog, assets


class Sfu16Tests(unittest.TestCase):
    def test_exact_variants_keep_standard_machining(self):
        result = sfu16.build(*fixture())
        self.assertEqual(len(result['recipes']), 3)
        self.assertEqual(result['recipes'][0]['values'], {'length': 400})
        self.assertFalse(result['custom_end_machining'])
        self.assertEqual(result['nut_mm']['overall_length'], 46)

    def test_changed_cad_cannot_inherit_datums(self):
        catalog, assets = fixture()
        assets[0]['sha256'] = 'changed'
        self.assertEqual(len(sfu16.build(catalog, assets)['recipes']), 2)

    def test_changed_drawing_dimensions_fail_closed(self):
        for product, index in [(0, 3), (1, 11), (2, 2)]:
            catalog, assets = fixture()
            catalog['products'][product]['specification_tables'][0][0][index] = 'changed'
            with self.assertRaises(ValueError):
                sfu16.build(catalog, assets)

    def test_ambiguous_sku_fails_closed(self):
        catalog, assets = fixture()
        catalog['variants'].append(copy.deepcopy(catalog['variants'][-1]))
        with self.assertRaises(ValueError):
            sfu16.build(catalog, assets)
