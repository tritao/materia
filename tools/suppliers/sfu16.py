"""Vallder drawing-based SFU1605/BF12 bindings and explicit supplier-intended fit basis."""
import re

BF12_HASH = '76ccf59477cad0fa0a33462842c34c0b3ff0eed99c2c60e70666bcad40a23c86'


def build(catalog, assets):
    parents = {p['supplier_product_id']: p for p in catalog['products']}
    screw = parents[3028]
    rows = [r for t in screw['specification_tables'] for r in t if r and r[0] == 'SFU1605']
    if len(rows) != 1 or rows[0][2:4] != ['5', '16']:
        raise ValueError('SFU1605 diameter/lead evidence changed')
    recipes = []
    for v in catalog['variants']:
        if v['parent_id'] != 3028:
            continue
        options = v['option_attributes']
        if len(options) != 1:
            raise ValueError('Ambiguous machined screw option')
        match = re.fullmatch(r'sfu-1605-(\d+)mm', options[0]['value'])
        if not match:
            continue
        length = int(match[1])
        if not 111 < length <= 4000:
            raise ValueError('Invalid SFU1605 length')
        recipes.append({'supplier_id': 'vallder', 'sku': v['sku'], 'supplier_product_id': v['supplier_product_id'],
            'parent_id': 3028, 'type_id': 'machinekit.motion.vallder-sfu1605-screw', 'values': {'length': length},
            'required_inputs': [], 'geometry_source': 'generator', 'compatibility': 'supplier_assumed',
            'source_url': screw['url'], 'drawing_url': 'https://vallder.com/wp-content/uploads/2017/08/products-sfu1605.png',
            'custom_end_machining': False})
    nut = parents[3061]
    rows = [r for t in nut['specification_tables'] for r in t if r and r[0] == 'SFU 1605']
    if len(rows) != 1 or len(rows[0]) != 19 or rows[0][6:16] != ['28', '27.8', '48', '38', '5.5', '46', '10', '10', 'M6', '40']:
        raise ValueError('SFU1605 nut drawing/table dimensions changed')
    for parent_id, sku, type_id, source in [(3061, rows[0][-1], 'machinekit.motion.vallder-sfu1605-nut', 'generator'),
                                           (2378, '1013002', 'machinekit.motion.vallder-bf12-support', 'supplier_cad')]:
        variants = [v for v in catalog['variants'] if v['parent_id'] == parent_id and v['sku'] == sku]
        if len(variants) != 1:
            raise ValueError('Ambiguous SFU nut/BF12 SKU')
        v = variants[0]
        if parent_id == 2378:
            if not any(a['sha256'] == BF12_HASH and a['usable_geometry'] and v['supplier_product_id'] in a['variant_ids'] for a in assets):
                continue  # Changed CAD does not inherit old datums.
            table_rows = [r for t in parents[parent_id]['specification_tables'] for r in t if r and r[0] == 'BF 12']
            if len(table_rows) != 1 or table_rows[0][2] != '10' or table_rows[0][7] != '25':
                raise ValueError('BF12 bore/shaft height evidence changed')
        recipes.append({'supplier_id': 'vallder', 'sku': sku, 'supplier_product_id': v['supplier_product_id'],
            'parent_id': parent_id, 'type_id': type_id, 'values': {}, 'required_inputs': [],
            'geometry_source': source, 'compatibility': 'supplier_assumed', 'source_url': parents[parent_id]['url']})
    return {'schema_version': 1, 'recipes': recipes, 'compatibility_basis': 'supplier_intended_SFU16_RBK12_BF12',
        'custom_end_machining': False,
        'screw_end_machining_mm': {'coupling_diameter': 10, 'coupling_length': 15, 'lock_thread': 'M12x1',
            'thread_and_relief_length': 14, 'fixed_journal_diameter': 12, 'fixed_journal_length': 25,
            'fixed_end_total': 54, 'floating_journal_diameter': 10, 'floating_journal_length': 11},
        'nut_mm': {'overall_length': 46, 'flange_offset': 10, 'flange_thickness': 10,
                   'pilot_diameter': 28, 'barrel_diameter': 27.8, 'flange_diameter': 48, 'bolt_circle': 38,
                   'bolt_hole': 5.5, 'bolt_angles_deg': [45,90,135,225,270,315], 'flange_flats': 40},
        'discrepancies': [{'feature': 'nut_length', 'selected_sku_table_mm': 46, 'calculator_mm': 42,
                            'decision': 'Use selected SKU1035001 table/drawing; do not resize purchased nut'},
                          {'feature': 'fixed_axial_stack', 'plain_journal_mm': 25, 'cad_bearings_plus_outer_sleeve_mm': 25.5,
                            'decision': 'Retain standard shaft; reference-model seating discrepancy, not a custom machining request'}],
        'limits': ['Supplier-intended compatibility is an assumption, distinct from a measured assembly fit',
                   'Race root, friction, backlash and load ratings are inherited reference assumptions',
                   'Screw threads semantic; small end chamfers/fillets and nut grease port not modeled']}
