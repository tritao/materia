#!/usr/bin/env python3
"""Separate custom motor plates from exact supplier CAD and interface evidence."""
from pathlib import PurePosixPath
from pathlib import Path
from urllib.parse import urlsplit
import re
import json


def attach_interfaces(bindings, references):
    """A changed STEP stays usable as raw CAD, but never inherits old mounting frames."""
    recipes = []
    for binding in bindings:
        reference = next((r for r in references if r['sku'] == binding['sku']), None)
        if reference is None or binding['geometry_kind'] != 'supplier_cad':
            continue
        if binding['name'] != reference['model'] + '-motor-bracket':
            raise ValueError('Pinned RBK interface model/SKU mismatch')
        cad = binding['cad']
        if len(cad) != 1 or cad[0]['sha256'] != reference['asset_sha256']:
            binding['interface_status'] = 'asset_revision_needs_measurement'
            continue
        binding['interface_status'] = 'pinned_cad_and_drawing_datums'
        binding['mounting_frame_status'] = 'pinned_named_datums'
        binding['interfaces'] = reference
        binding['type_id'] = 'machinekit.assembly.rbk-motor-mount'
        binding['recipe_values'] = {'model': reference['model'], 'revision': reference['revision']}
        recipes.append({'supplier_id': 'vallder', 'sku': binding['sku'],
            'supplier_product_id': binding['supplier_product_id'], 'parent_id': 6718,
            'type_id': binding['type_id'], 'values': binding['recipe_values'], 'required_inputs': [],
            'geometry_source': 'supplier_cad', 'compatibility': 'review_required',
            'asset_sha256': reference['asset_sha256'], 'source_url': binding['source_url']})
    return recipes


def bind_rbk(catalog, assets):
    """RBK table is transposed: Model/SKU/Download rows join by column, not row."""
    parent = next(p for p in catalog['products'] if p['supplier_product_id'] == 6718)
    tables = [t for t in parent['specification_tables'] if t and t[0] and t[0][0] == 'Model']
    if len(tables) != 1:
        raise ValueError('Ambiguous RBK model table')
    table = tables[0]
    def unique(label):
        rows = [r for r in table if r and r[0] == label]
        if len(rows) != 1 or len(rows[0]) != len(table[0]):
            raise ValueError('RBK column identity changed: ' + label)
        return rows[0]
    models, skus, downloads = unique('Model'), unique('SKU'), unique('Download 3D Model')
    result = []
    for col in range(1, len(models)):
        model = re.fullmatch(r'RBK(\d+)-(57|86)-motor-bracket', models[col])
        if not model or downloads[col] != f'{model[1]}-{model[2]}-3D':
            raise ValueError('RBK model/download column mismatch')
        variants = [v for v in catalog['variants'] if v['parent_id'] == 6718 and v['sku'] == skus[col]]
        if len(variants) != 1 or variants[0]['option_attributes'] != [{'name': 'Variant', 'value': models[col].lower()}]:
            raise ValueError('RBK model/SKU/variant column mismatch')
        variant = variants[0]
        urls = [u for u in parent['description_links'] if PurePosixPath(urlsplit(u).path).name == downloads[col] + '.zip']
        if len(urls) != 1:
            raise ValueError('RBK download column has no unique exact filename link')
        matching = [a for a in assets if a['product_id'] == 6718 and a['source_url'] == urls[0]]
        if len(matching) > 1:
            raise ValueError('RBK archive contains ambiguous STEP choices')
        for asset in matching:
            if asset['variant_ids'] and asset['variant_ids'] != [variant['supplier_product_id']]:
                raise ValueError('RBK CAD variant conflicts with column evidence')
            asset['variant_ids'] = [variant['supplier_product_id']]
            asset['identity_status'] = 'table_linked'
            evidence = {'table_column': col, 'model': models[col], 'supplier_skus': [skus[col]],
                        'variant_ids': [variant['supplier_product_id']], 'download_label': downloads[col]}
            if evidence not in asset['table_evidence']:
                asset['table_evidence'].append(evidence)
        usable = [a for a in matching if a['usable_geometry']]
        result.append({'supplier_id': 'vallder', 'sku': variant['sku'], 'supplier_product_id': variant['supplier_product_id'],
            'name': models[col], 'source_url': parent['url'], 'observed_at': variant['source']['fetched_at'],
            'geometry_kind': 'supplier_cad', 'generic_plate_equivalent': False,
            'cad': [{'file': a['file'], 'sha256': a['sha256'], 'source_url': a['source_url']} for a in usable],
            'status': 'supplier_geometry_available' if usable else 'geometry_gap',
            'mounting_frame_status': 'unverified', 'compatibility_verified': False,
            'bearing': unique('Bearing')[col], 'optional_bearing': unique('Angular Bearing (optional)')[col],
            'ballscrew_size': unique('Ballscrew Size')[col]})
    return result


def build(catalog, assets):
    bindings = bind_rbk(catalog, assets)
    cube = next(p for p in catalog['products'] if p['supplier_product_id'] == 6685)
    expected_header = ['Model', 'D', 'W', 'L', 'H', 'H1', 'W1', 'D1', 'D2', 'D3', 'D4', 'D5', 'D6', 'X', 'X1', 'SKU', 'Weight', 'Price €']
    tables = [t for t in cube['specification_tables'] if t and t[0] == expected_header]
    if len(tables) != 1:
        raise ValueError('Motor cube dimension table changed')
    for row in tables[0][1:]:
        if not row or row[0] not in ('DB57-motor-bracket', 'DB86-motor-bracket'):
            continue
        if len(row) != len(expected_header):
            raise ValueError('Motor cube dimension row changed')
        frame, pilot, spacing = (23, 38.1, 47.14) if row[0].startswith('DB57') else (34, 73.0, 69.6)
        variants = [v for v in catalog['variants'] if v['parent_id'] == 6685 and v['sku'] == row[15]]
        if len(variants) != 1 or variants[0]['option_attributes'] != [{'name': 'Variant', 'value': row[0].lower()}]:
            raise ValueError('Motor cube model/SKU mismatch')
        bindings.append({'supplier_id': 'vallder', 'sku': row[15], 'name': row[0], 'source_url': cube['url'],
            'geometry_kind': 'cube_bracket', 'generic_plate_equivalent': False, 'status': 'geometry_gap',
            'candidate_nema_frame': frame, 'opening_mm': float(row[1]), 'reference_pilot_mm': pilot,
            'nominal_diametral_clearance_mm': round(float(row[1])-pilot, 6),
            'bolt_pitch_mm': float(row[7]), 'reference_bolt_pitch_mm': spacing,
            'bolt_pitch_matches': abs(float(row[7])-spacing) < 0.001,
            'pilot_review': 'nominal_interference' if float(row[1]) < pilot else 'tolerance_review_required',
            'compatibility_verified': False, 'table_row': row,
            'limits': ['D5 missing; fillets and manufacturing tolerances unspecified',
                       'Cube geometry cannot be supplied by the flat NEMA plate generator']})
    for pid in (4182, 26376, 26380):
        p = next(p for p in catalog['products'] if p['supplier_product_id'] == pid)
        bindings.append({'supplier_id': 'vallder', 'sku': p['sku'], 'name': p['name'], 'source_url': p['url'],
            'geometry_kind': 'drawing_only', 'generic_plate_equivalent': False, 'status': 'drawing_review_required',
            'drawing_urls': p['description_links'], 'compatibility_verified': False,
            'limits': ['Asymmetric plate/bent bracket geometry differs from the generic square plate',
                       'Profile-side features need full dimensional and clearance qualification']})
    references = json.loads(Path(__file__).with_name('rbk-interfaces.json').read_text())['interfaces']
    recipes = attach_interfaces(bindings, references)
    return {'schema_version': 1, 'generic_recipe_type': 'machinekit.assembly.nema-mount-plate',
            'generic_plate_offer_policy': 'Custom made part; no Vallder SKU inferred', 'bindings': bindings,
            'recipes': recipes}
