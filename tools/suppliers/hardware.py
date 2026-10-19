#!/usr/bin/env python3
"""Evidence-bound slot hardware recipes; no series-name compatibility inference."""
import re
import math

NUT_DRAWING = 'https://vallder.com/wp-content/uploads/2020/03/products-t-nut-dimension.png'


def build(catalog):
    parents = {p['supplier_product_id']: p for p in catalog['products']}
    parent = parents[5323]
    tables = [t for t in parent['specification_tables'] if t and
              t[0] == ['Type', 'Slot size', 'C', 'E', 'A', 'B', 'F', 'D']]
    if len(tables) != 1:
        raise ValueError('Slide-in nut dimension table changed')
    rows = {}
    for row in tables[0][1:]:
        if row and row[0] in ('T-Nut-30', 'T-Nut-40'):
            if row[0] in rows:
                raise ValueError('Ambiguous slide-in nut dimension row')
            rows[row[0]] = row
    recipes = []
    for variant in catalog['variants']:
        if variant['parent_id'] != 5323:
            continue
        options = variant['option_attributes']
        if len(options) != 1:
            raise ValueError('Ambiguous slide-in nut option')
        match = re.fullmatch(r't-nut-(30|40)-(m[34568])', options[0]['value'])
        if not match:
            if re.fullmatch(r't-nut-(20|45)-m[34568]', options[0]['value']):
                continue
            raise ValueError('Unrecognized slide-in nut option')
        series, size = match.groups()
        row = rows['T-Nut-' + series]
        if len(row) != 8 or size.upper() not in row[3].split(' / '):
            raise ValueError('Nut thread/table mismatch')
        c, a, b, f, d = (float(row[i]) for i in (2, 4, 5, 6, 7))
        if row[1] != series or not all(math.isfinite(v) and v > 0 for v in (c, a, b, f, d)) or d >= c or f >= b:
            raise ValueError('Invalid stepped nut dimensions')
        recipes.append({'supplier_id': 'vallder', 'sku': variant['sku'],
            'supplier_product_id': variant['supplier_product_id'], 'parent_id': 5323,
            'type_id': 'machinekit.structural.slide-in-slot-nut',
            'values': {'width': b, 'length': a, 'height': c, 'ridgeWidth': f,
                       'ridgeHeight': d, 'size': size.upper()},
            'required_inputs': [], 'geometry_source': 'generator', 'compatibility': 'review_required',
            'source_url': parent['url'], 'drawing_url': NUT_DRAWING, 'table_row': row,
            'candidate_system_id': 'TSLOT-' + series + '-SLOT8',
            'limits': ['Sharp conservative envelope: unspecified chamfers omitted',
                       'Thread is semantic; nominal bore is not a helical manufacturing model',
                       'No dimensional tolerances, strength rating or screw engagement qualification',
                       'Fit results apply only to the exact drawing sections tested']})
    gaps = []
    for pid, missing in [(5323, ['20/45 series ridge height D']),
                         (5502, ['B overall height', 'W ridge width', 'C top feature extent', 'rotation clearance']),
                         (5554, ['A', 'a1', 'b', 'spring envelope']),
                         (5857, ['3030/4040 B, K, L and locating tab dimensions',
                                 '8080 slot length/end positions and gusset details', 'screw/washer/tool access']),
                         (6027, ['leg thickness and inside corner', 'cross-hole position', 'set-screw engagement'])]:
        p = parents[pid]
        gaps.append({'parent_id': pid, 'name': p['name'], 'source_url': p['url'],
                     'status': 'review_required', 'missing_evidence': missing,
                     'tables': p['specification_tables'], 'compatibility_verified': False})
    bracket_interfaces = []
    for table in parents[5857]['specification_tables']:
        for row in table:
            if not row:
                continue
            if row[0] in ('RA-BRACKET-3030', 'RA-BRACKET-4040'):
                if len(row) != 10:
                    raise ValueError('Right-angle bracket table changed')
                bracket_interfaces.append({'sku': row[9], 'name': row[0],
                    'slot_width_mm': float(row[3]), 'slot_centre_travel_mm': float(row[5]),
                    'candidate_thread': 'M6' if row[0].endswith('3030') else 'M8',
                    'source_url': parents[5857]['url'], 'compatibility_verified': False,
                    'status': 'partial_dimensions_only'})
            elif row[0] == 'RA-BRACKET-8080':
                if len(row) != 9:
                    raise ValueError('8080 bracket table changed')
                bracket_interfaces.append({'sku': row[8], 'name': row[0],
                    'hole_diameter_mm': float(row[4]), 'hole_pitch_mm': float(row[6]),
                    'candidate_thread': 'M8', 'candidate_system_id': 'TSLOT-40-SLOT8',
                    'pitch_matches_grid': float(row[6]) == 40,
                    'source_url': parents[5857]['url'], 'compatibility_verified': False,
                    'status': 'pitch_only_not_full_mount_qualification'})
    return {'schema_version': 1, 'recipes': recipes, 'evidence_gaps': gaps,
            'bracket_interfaces': bracket_interfaces,
            'fit_report': 'hardware-fit.json (native check; never implies a load rating)'}


if __name__ == '__main__':
    import argparse
    import json
    from vallder import DEFAULT_OUTPUT, write_json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=__import__('pathlib').Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    result = build(json.loads((args.output / 'catalog.json').read_text()))
    write_json(args.output / 'hardware.json', result)
    print(f"{len(result['recipes'])} slide-in nut bindings; {len(result['evidence_gaps'])} evidence reviews")
