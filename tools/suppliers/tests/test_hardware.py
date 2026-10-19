from pathlib import Path
import copy
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import hardware


def catalogue():
    parents = [{'supplier_product_id': pid, 'name': str(pid), 'url': 'https://example.test/' + str(pid),
                'specification_tables': []} for pid in (5323, 5502, 5554, 5857, 6027)]
    parents[0]['specification_tables'] = [[
        ['Type', 'Slot size', 'C', 'E', 'A', 'B', 'F', 'D'],
        ['T-Nut-30', '30', '5.9', 'M5 / M6 / M8', '15.5', '15.9', '8.0', '1.9'],
        ['T-Nut-40', '40', '8.9', 'M4 / M5 / M6 / M8', '19.4', '19.0', '7.8', '2.7']]]
    return {'products': parents, 'variants': [
        {'parent_id': 5323, 'sku': 'actual-sku', 'supplier_product_id': 123,
         'option_attributes': [{'name': 'Variant', 'value': 't-nut-30-m6'}]},
        {'parent_id': 5323, 'sku': 'unsupported-20', 'supplier_product_id': 124,
         'option_attributes': [{'name': 'Variant', 'value': 't-nut-20-m5'}]}]}


class HardwareTests(unittest.TestCase):
    def test_exact_option_join_keeps_incomplete_series_unbound(self):
        report = hardware.build(catalogue())
        self.assertEqual([r['sku'] for r in report['recipes']], ['actual-sku'])
        recipe = report['recipes'][0]
        self.assertEqual(recipe['values']['width'], 15.9)
        self.assertEqual(recipe['values']['length'], 15.5)
        self.assertEqual(recipe['values']['size'], 'M6')
        self.assertEqual(recipe['compatibility'], 'review_required')
        self.assertTrue(all(not gap['compatibility_verified'] for gap in report['evidence_gaps']))

    def test_ambiguous_options_or_missing_table_thread_fail_closed(self):
        original = catalogue()
        ambiguous = copy.deepcopy(original)
        ambiguous['variants'][0]['option_attributes'].append({'name': 'Size', 'value': 'M5'})
        missing_thread = copy.deepcopy(original)
        missing_thread['products'][0]['specification_tables'][0][1][3] = 'M5 / M8'
        for catalog in (ambiguous, missing_thread):
            with self.assertRaises(ValueError):
                hardware.build(catalog)

    def test_mixed_bracket_headers_do_not_imply_compatibility(self):
        catalog = catalogue()
        bracket = next(p for p in catalog['products'] if p['supplier_product_id'] == 5857)
        bracket['specification_tables'] = [[
            ['Product', 'H', 'A', 'G', 'E', 'F', 'D', 'C', 'Weight [kg]', 'SKU'],
            ['RA-BRACKET-3030', '35', '28', '7', '12.5', '9', '3', '4', '.025', '3030-sku'],
            ['Product', 'A', 'B', 'C', 'D', 'E', 'F', 'Weight [kg]', 'SKU'],
            ['RA-BRACKET-8080', '78', '78', '78.5', '8.5', '6', '40', '.170', '8080-sku']]]
        report = hardware.build(catalog)
        interfaces = report['bracket_interfaces']
        self.assertEqual(interfaces[0]['slot_width_mm'], 7)
        self.assertEqual(interfaces[1]['hole_diameter_mm'], 8.5)
        self.assertTrue(interfaces[1]['pitch_matches_grid'])
        self.assertFalse(interfaces[1]['compatibility_verified'])
        bracket['specification_tables'][0][-1][6] = '39'
        self.assertFalse(hardware.build(catalog)['bracket_interfaces'][1]['pitch_matches_grid'])


if __name__ == '__main__':
    unittest.main()
