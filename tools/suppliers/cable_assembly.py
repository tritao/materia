#!/usr/bin/env python3
"""Validate evidence-bearing cable recipes and emit BOM, netlist and CAD inputs (mm)."""
import argparse
import hashlib
from collections import Counter
from html import escape
import json
import math
from pathlib import Path
from urllib.request import Request, urlopen

from lcsccable import write_json

ROOT = Path(__file__).resolve().parents[2]
REFERENCE = Path(__file__).with_name('lcsccable-reference.json')
EXAMPLE = ROOT / 'machinekit/examples/cable-harness/xh-two-wire.json'


def positive(value, name, zero=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0 or (value == 0 and not zero):
        raise ValueError(name + ' must be a finite ' + ('nonnegative' if zero else 'positive') + ' number')
    return value


def vector(value, name):
    if not isinstance(value, list) or len(value) != 3 or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise ValueError(name + ' must be a finite 3-vector')
    return value


def sub(a, b): return [a[i] - b[i] for i in range(3)]
def dot(a, b): return sum(x * y for x, y in zip(a, b))
def norm(a): return math.sqrt(dot(a, a))
def cross(a, b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def indexed(rows, name):
    result = {}
    if not isinstance(rows, list) or not rows:
        raise ValueError(name + ' must be a nonempty array')
    for row in rows:
        identity = row.get('id')
        if not isinstance(identity, str) or not identity or identity in result:
            raise ValueError('Missing or duplicate ' + name + ' ID')
        result[identity] = row
    return result


def property_value(prop, name, issues):
    if prop is None:
        issues.add(name + ': unknown')
        return None
    if prop.get('status') not in ('documented', 'design_assumption') or not prop.get('evidence'):
        raise ValueError(name + ' needs evidence and a documented/design_assumption status')
    if prop['status'] != 'documented':
        issues.add(name + ': design assumption')
    return prop['value']


def validate(recipe, reference, catalog=None):
    if recipe.get('schema_version') != 1 or recipe.get('units') != 'mm':
        raise ValueError('Expected cable recipe v1 in mm')
    if reference.get('schema_version') != 1:
        raise ValueError('Unsupported cable reference')
    ends = indexed(recipe.get('ends'), 'end')
    wire_types = indexed(recipe.get('wire_types'), 'wire type')
    wires = indexed(recipe.get('wires'), 'wire')
    if set(ends) & set(wires):
        raise ValueError('End and wire geometry IDs must be distinct')
    reference_parts = indexed(reference['parts'], 'reference part')
    issues = {'Supplier drawing approval, crimp process and assembly tolerances are not qualified'}
    bom = Counter()
    wire_lengths = Counter()
    connections, geometry_wires, geometry_ends, used = [], [], [], set()
    selected_parts = {p['id']: p for p in catalog['parts']} if catalog else None
    if catalog is None:
        issues.add('Live supplier catalogue membership was not checked')
    for end in ends.values():
        housing = reference_parts.get(end['housing_id'])
        if housing is None or housing['kind'] != 'housing':
            raise ValueError('Unknown housing')
        if selected_parts is not None and end['housing_id'] not in selected_parts:
            raise ValueError('Housing missing from captured supplier selectors')
        origin, direction, up = [vector(end[k], k) for k in ('origin', 'exit_direction', 'up')]
        if abs(norm(direction)-1) > 1e-8 or abs(norm(up)-1) > 1e-8 or abs(dot(direction, up)) > 1e-8:
            raise ValueError('End frame must have orthonormal exit_direction and up')
        vacant = end.get('unpopulated_cavities', [])
        if len(set(vacant)) != len(vacant) or any(type(c) is not int or not 1 <= c <= housing['positions'] for c in vacant):
            raise ValueError('Invalid unpopulated cavity')
        bom[housing['id']] += 1
        geometry_ends.append({'id': end['id'], 'housing_id': housing['id'], 'origin': origin,
                              'exit_direction': direction, 'up': up, 'dimensions_mm': housing['envelope_mm'],
                              'geometry_status': 'nominal_envelope', 'source': housing['source'],
                              'placement_status': 'design_assumption'})
        issues.add('Connector cavity centre and rear-face placement are nominal design datums')
    for wire_type in wire_types.values():
        positive(wire_type['awg'], 'AWG')
        if type(wire_type['awg']) is not int:
            raise ValueError('AWG must be an integer')
        if catalog is not None:
            if 'AWG' + str(wire_type['awg']) not in catalog['gauge_options']:
                raise ValueError('Wire gauge missing from supplier selectors')
            matches = [s for s in catalog['wire_styles'] if s['raw'] == wire_type['supplier_style']]
            if len(matches) != 1:
                raise ValueError('Wire style must match one exact supplier record including voltage/display name')
        if wire_type.get('manufacturer_mpn') is None:
            issues.add('Wire ' + wire_type['id'] + ': exact manufacturer MPN unknown')
        for key in ('outer_diameter_mm', 'minimum_bend_radius_mm', 'voltage_v', 'temperature_range_c', 'strand_construction', 'length_tolerance_mm'):
            value = property_value(wire_type.get(key), wire_type['id'] + '.' + key, issues)
            if key in ('outer_diameter_mm', 'minimum_bend_radius_mm', 'voltage_v') and value is not None:
                positive(value, key)
            elif key == 'temperature_range_c' and value is not None:
                if not isinstance(value, list) or len(value) != 2 or any(type(x) not in (int, float) or not math.isfinite(x) for x in value) or value[0] >= value[1]:
                    raise ValueError('Invalid wire temperature range')
            elif key == 'length_tolerance_mm' and value is not None:
                positive(value, key, zero=True)
            elif key == 'strand_construction' and value is not None:
                if not isinstance(value, str) or not value.strip():
                    raise ValueError('Invalid strand construction')
    for wire in wires.values():
        wt = wire_types.get(wire['wire_type_id'])
        if wt is None:
            raise ValueError('Unknown wire type')
        od = property_value(wt.get('outer_diameter_mm'), wt['id'] + '.outer_diameter_mm', issues)
        if od is None:
            raise ValueError('Preview requires an explicit wire outer diameter')
        cut = positive(wire['cut_length_mm'], 'cut length')
        if not isinstance(wire.get('color'), str) or not wire['color']:
            raise ValueError('Wire needs an explicit color')
        route = wire['route']
        if not isinstance(route, list) or len(route) < 2:
            raise ValueError('Wire route needs at least two points')
        for p in route:
            vector(p, 'route point')
        directions = [sub(b, a) for a, b in zip(route, route[1:])]
        if any(norm(d) < 1e-8 for d in directions):
            raise ValueError('Zero-length route segment')
        length = sum(norm(d) for d in directions)
        if any(norm(cross(a, b)) > 1e-8 or dot(a, b) <= 0 for a, b in zip(directions, directions[1:])):
            raise ValueError('Sharp route corners are unsupported; provide smooth routes in a future geometry version')
        allowances, stripped_length = 0, 0
        for endpoint, route_point, tangent in [(wire['from'], route[0], directions[0]), (wire['to'], route[-1], [-v for v in directions[-1]])]:
            end = ends.get(endpoint['end_id'])
            if end is None:
                raise ValueError('Unknown end')
            housing = reference_parts[end['housing_id']]
            cavity = endpoint['cavity']
            if type(cavity) is not int or not 1 <= cavity <= housing['positions']:
                raise ValueError('Cavity outside housing')
            key = (end['id'], cavity)
            if key in used or cavity in end.get('unpopulated_cavities', []):
                raise ValueError('Duplicate or unpopulated cavity assignment')
            used.add(key)
            contact = reference_parts.get(endpoint['contact_id'])
            if contact is None or contact['kind'] != 'contact' or contact['mpn'] not in housing['compatible_contacts']:
                raise ValueError('Contact incompatible with housing')
            if selected_parts is not None and contact['id'] not in selected_parts:
                raise ValueError('Contact missing from supplier selectors')
            if not contact['awg_range'][0] <= wt['awg'] <= contact['awg_range'][1]:
                raise ValueError('Contact incompatible with wire gauge')
            if not contact['insulation_od_mm'][0] <= od <= contact['insulation_od_mm'][1]:
                raise ValueError('Contact incompatible with insulation diameter')
            right = cross(end['exit_direction'], end['up'])
            offset = (cavity - (housing['positions']+1)/2)*housing['pitch_mm']
            expected = [end['origin'][i]+right[i]*offset for i in range(3)]
            if norm(sub(route_point, expected)) > 1e-6:
                raise ValueError('Route endpoint does not meet its connector cavity datum')
            if dot(tangent, end['exit_direction'])/norm(tangent) < 1-1e-6:
                raise ValueError('Wire tangent disagrees with connector exit direction')
            allowance = property_value(endpoint.get('termination_allowance_mm'), wire['id'] + '.termination_allowance_mm', issues)
            if allowance is None:
                raise ValueError('Explicit termination allowance required')
            allowances += positive(allowance, 'termination allowance', zero=True)
            strip = property_value(endpoint.get('strip_length_mm'), wire['id'] + '.strip_length_mm', issues)
            if strip is not None:
                positive(strip, 'strip length')
                stripped_length += strip
                if strip > cut:
                    raise ValueError('Strip length exceeds cut length')
            if endpoint.get('tinning_length_mm') is not None:
                positive(endpoint['tinning_length_mm'], 'tinning length', zero=True)
                if strip is None or endpoint['tinning_length_mm'] > strip:
                    raise ValueError('Tinning exceeds stripped length')
                issues.add('Tinning a crimped end requires manufacturer process approval')
            bom[contact['id']] += 1
        if stripped_length > cut + 1e-6:
            raise ValueError('Combined stripping exceeds cut length')
        if length + allowances > cut + 1e-6:
            raise ValueError('Wire cut length shorter than route plus termination allowances')
        if cut - length - allowances > 1e-6:
            issues.add(wire['id'] + ': slack is specified but not represented by routed geometry')
        wire_key = (wt['id'], wire['color'])
        wire_lengths[wire_key] += cut
        connections.append({'wire_id': wire['id'], 'from': wire['from'], 'to': wire['to'],
                            'color': wire['color'], 'awg': wt['awg'], 'cut_length_mm': cut,
                            'routed_length_mm': length, 'termination_allowances_mm': allowances,
                            'unrouted_slack_mm': max(0, cut-length-allowances)})
        geometry_wires.append({'id': wire['id'], 'diameter_mm': od, 'route': route,
                               'color': wire['color'], 'geometry_status': 'nominal_route_envelope',
                               'diameter_status': wt['outer_diameter_mm']['status']})
    for end in ends.values():
        housing = reference_parts[end['housing_id']]
        assigned = {c for e, c in used if e == end['id']}
        if assigned | set(end.get('unpopulated_cavities', [])) != set(range(1, housing['positions']+1)):
            raise ValueError('Every cavity must be assigned or explicitly unpopulated')
    # Volumes of adjacent cylindrical wire envelopes must not overlap.
    # v1 supports straight parallel runs; arbitrary segment distance is intentionally not guessed.
    for i, a in enumerate(geometry_wires):
        for b in geometry_wires[i+1:]:
            da, db = sub(a['route'][-1], a['route'][0]), sub(b['route'][-1], b['route'][0])
            if norm(cross(da, db)) > 1e-8:
                raise ValueError('Nonparallel multiwire collision checking is unsupported in v1')
            delta = sub(b['route'][0], a['route'][0])
            gap = norm(cross(delta, da))/norm(da)
            ta = sorted([dot(sub(p, a['route'][0]), da)/dot(da, da) for p in (b['route'][0], b['route'][-1])])
            if max(0, ta[0]) < min(1, ta[1]) and gap < (a['diameter_mm']+b['diameter_mm'])/2 - 1e-8:
                raise ValueError('Wire envelopes overlap')
    rows = [{'kind': reference_parts[k]['kind'], 'supplier_id': 'lcsccable', 'supplier_part_id': k,
             'manufacturer': reference_parts[k]['manufacturer'], 'mpn': reference_parts[k]['mpn'],
             'quantity': count, 'unit': 'piece'} for k, count in sorted(bom.items())]
    rows.extend({'kind': 'wire', 'wire_type_id': k[0], 'color': k[1], 'quantity': value, 'unit': 'mm',
                 'manufacturer_mpn': wire_types[k[0]].get('manufacturer_mpn')} for k, value in sorted(wire_lengths.items()))
    def fingerprint(data):
        return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return {'schema_version': 1, 'id': recipe['id'], 'units': 'mm',
            'recipe': recipe,
            'input_fingerprints': {'recipe_sha256': fingerprint(recipe), 'reference_sha256': fingerprint(reference),
                                   'catalog_sha256': fingerprint(catalog) if catalog else None},
            'part_evidence': {k: {'drawing': reference_parts[k]['source'],
                                  'supplier': reference_parts[k]['supplier_source']} for k in sorted(bom)},
            'status': 'reference_design', 'manufacturing_qualified': False,
            'open_requirements': sorted(issues), 'bom': rows, 'connections': connections,
            'geometry': {'schema_version': 1, 'units': 'mm', 'manufacturing_qualified': False,
                         'id': recipe['id'], 'wires': geometry_wires, 'ends': geometry_ends}}


def drawing(report):
    """Dimensioned wire schedule; no implied connector manufacturing detail."""
    height = 150 + 70*len(report['connections'])
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="{height}" viewBox="0 0 1000 {height}">',
             '<rect width="100%" height="100%" fill="white"/>',
             '<g font-family="sans-serif" font-size="14" fill="#222">',
             f'<text x="25" y="30">{escape(report["id"])} — wire schedule (mm; schematic, not to scale)</text>',
             '<text x="25" y="55">REFERENCE DESIGN — nominal envelopes; see report.json for open requirements.</text>']
    for i, row in enumerate(report['connections']):
        y = 110+i*70
        a, b = row['from'], row['to']
        lines.extend([f'<path d="M220 {y} H720" stroke="#555" stroke-width="3"/>',
                      f'<text x="25" y="{y+5}">{escape(a["end_id"])} cavity {a["cavity"]}</text>',
                      f'<text x="750" y="{y+5}">{escape(b["end_id"])} cavity {b["cavity"]}</text>',
                      f'<text x="230" y="{y-12}">{escape(row["wire_id"])} / AWG{row["awg"]} / {escape(row["color"])}</text>',
                      f'<text x="230" y="{y+24}">route {row["routed_length_mm"]:g}; cut {row["cut_length_mm"]:g}; termination allowances {row["termination_allowances_mm"]:g}</text>'])
    return '\n'.join(lines + ['</g></svg>'])


def fetch_documents(reference, output):
    """Fetch only public, pinned source PDFs. Never follow manufacturer form workflows."""
    sources = {p['source']['sha256']: p['source'] for p in reference['parts']}
    for digest, source in sources.items():
        path = output / (digest + '.pdf')
        if path.exists():
            body = path.read_bytes()
        else:
            request = Request(source['url'], headers={'User-Agent': 'Mozilla/5.0 (compatible; MateriaSupplierImporter/1.0)',
                                                     'Referer': 'https://www.lcsc.com/'})
            with urlopen(request, timeout=30) as response:
                body = response.read()
        if not body.startswith(b'%PDF') or hashlib.sha256(body).hexdigest() != digest:
            raise ValueError('Source document differs from reviewed PDF; review before updating reference')
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(body)
        write_json(path.with_suffix('.source.json'), source)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recipe', type=Path, default=EXAMPLE)
    parser.add_argument('--reference', type=Path, default=REFERENCE)
    parser.add_argument('--catalog', type=Path)
    parser.add_argument('--fetch-documents', action='store_true', help='Fetch and checksum public source PDFs before generation')
    parser.add_argument('--output', type=Path, default=ROOT / 'build/suppliers/lcsccable/example')
    args = parser.parse_args()
    reference = json.loads(args.reference.read_text())
    if args.fetch_documents:
        fetch_documents(reference, args.output.parent / 'documents')
    report = validate(json.loads(args.recipe.read_text()), reference,
                      json.loads(args.catalog.read_text()) if args.catalog else None)
    for name, value in [('report', report), ('bom', report['bom']), ('connections', report['connections']), ('geometry', report['geometry'])]:
        write_json(args.output / (name + '.json'), value)
    (args.output / 'drawing.svg').write_text(drawing(report))
    print(f'{report["id"]}: {len(report["connections"])} wires; {len(report["open_requirements"])} open requirements; manufacturing_qualified=false')


if __name__ == '__main__':
    main()
