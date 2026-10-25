#!/usr/bin/env python3
"""Read-only LCSC Cable capability capture. No cart/order/account requests."""
import argparse
import csv
from collections import Counter
from datetime import datetime, timezone
import hashlib
from html.parser import HTMLParser
import json
from pathlib import Path
import re
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin, urlsplit
from urllib.request import Request, urlopen

BASE = 'https://lcsccable.com'
DEFAULT_OUTPUT = Path(__file__).resolve().parents[2] / 'build/suppliers/lcsccable'
ENDPOINTS = {
    'gauges': '/api/cable/ecp/get/cable/select?returnMpnType=cable',
    'wire_styles': '/api/cable/ecp/get/cable/specification-model?returnMpnType=specification_model',
    'housings': '/api/cable/ecp/get/cable/select?returnMpnType=rubber_shell',
    'terminals': '/api/cable/ecp/get/cable/select?returnMpnType=terminal',
}


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n')
    temporary.replace(path)


class Client:
    """Cache failures as evidence too; offline replay never makes network requests."""
    def __init__(self, directory, refresh=False, offline=False):
        self.directory, self.refresh, self.offline = directory, refresh, offline
        self.last_request = 0

    def get(self, url):
        host = urlsplit(url).hostname
        if host not in ('lcsccable.com', 'static.lcsccable.com'):
            raise ValueError('Unexpected supplier host: ' + str(host))
        path = self.directory / (hashlib.sha256(url.encode()).hexdigest() + '.json')
        if path.exists() and not self.refresh:
            data = json.loads(path.read_text())
            if data['url'] != url or hashlib.sha256(data['body'].encode()).hexdigest() != data['sha256']:
                raise ValueError('Corrupt supplier cache')
            return data
        if self.offline:
            raise ValueError('Missing cached response: ' + url)
        for attempt in range(3):
            time.sleep(max(0, .5 - (time.monotonic() - self.last_request)))
            self.last_request = time.monotonic()
            request = Request(url, headers={
                'User-Agent': 'Mozilla/5.0 (compatible; MateriaSupplierImporter/1.0)',
                'Referer': BASE + '/customize', 'Origin': BASE,
                'Accept': 'application/json, text/plain, */*',
            })
            try:
                response = urlopen(request, timeout=30)
            except HTTPError as error:
                response = error
            except (URLError, TimeoutError):
                if attempt == 2:
                    raise
                time.sleep(2 ** attempt)
                continue
            with response:
                body = response.read().decode('utf-8')
                data = {'url': url, 'status': response.code,
                        'fetched_at': datetime.now(timezone.utc).isoformat(),
                        'headers': {k.lower(): v for k, v in response.headers.items()},
                        'body': body, 'sha256': hashlib.sha256(body.encode()).hexdigest()}
            if data['status'] == 429 or data['status'] >= 500:
                if attempt < 2:
                    time.sleep(2 ** attempt)
                    continue
            write_json(path, data)
            return data


class Scripts(HTMLParser):
    def __init__(self):
        super().__init__()
        self.urls = []

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == 'script' and attrs.get('src'):
            url = urljoin(BASE, attrs['src'])
            if urlsplit(url).hostname == 'static.lcsccable.com':
                self.urls.append(url)


def literal_array(text, marker):
    """Decode a data-only JS array; never evaluate downloaded JavaScript."""
    start = text.index(marker) + len(marker)
    if text[start] != '[':
        raise ValueError('Expected array after ' + marker)
    depth, quote, escaped = 0, None, False
    for end in range(start, len(text)):
        char = text[end]
        if quote:
            if escaped:
                escaped = False
            elif char == '\\':
                escaped = True
            elif char == quote:
                quote = None
            continue
        if char in ('"', "'"):
            quote = char
        elif char in '[{':
            depth += 1
        elif char in ']}':
            depth -= 1
            if depth == 0:
                fragment = text[start:end + 1]
                # Only transform tokens outside strings; JSON rejects expressions/functions.
                tokens = re.split(r'("(?:\\.|[^"\\])*")', fragment)
                for i in range(0, len(tokens), 2):
                    tokens[i] = re.sub(r'([A-Za-z_$][\w$]*)\s*:', r'"\1":', tokens[i])
                    tokens[i] = tokens[i].replace('!0', 'true').replace('!1', 'false')
                return json.loads(''.join(tokens))
    raise ValueError('Unterminated supplier array')


def source(record):
    return {k: record[k] for k in ('url', 'status', 'fetched_at', 'sha256')}


def part(token, kind, evidence):
    # Keep exact token and MPN: brands and MPNs may themselves contain underscores.
    match = re.fullmatch(r'(.+)_([^_]+)_(C\d+)', token)
    if not match:
        raise ValueError('Unrecognized supplier part identity: ' + token)
    return {'id': match[3], 'kind': kind, 'supplier_token': token,
            'manufacturer': match[1], 'mpn': match[2], 'source': evidence,
            'engineering_verification': 'unverified', 'component_id': None}


def capture(client):
    page = client.get(BASE + '/customize')
    if page['status'] != 200:
        raise ValueError('Customizer unavailable')
    scripts = Scripts()
    scripts.feed(page['body'])
    bundles = [client.get(url) for url in dict.fromkeys(scripts.urls)]
    bundles = [b for b in bundles if b['status'] == 200]
    def bundle(marker):
        found = [b for b in bundles if marker in b['body']]
        if len(found) != 1:
            raise ValueError('Frontend changed: expected one bundle containing ' + marker)
        return found[0]
    quote = bundle('tabOptions:[{label:"Consumer Electronics"')
    shared = bundle('value:"multiColor",label:"MultiColor"')
    rules = bundle('Minimum length is 30mm')
    services = literal_array(quote['body'], 'tabOptions:')
    shapes = literal_array(quote['body'], 'shapeOptions:')
    shared_data = shared['body'][shared['body'].index('i=[{value:"black"'):]
    colors = literal_array(shared_data, 'i=')
    treatments = literal_array(shared_data, 'a=')
    units = literal_array(shared_data, 'c=')
    pinout_match = re.search(r'he=\[(.*?)\],me=', quote['body'])
    if pinout_match is None:
        raise ValueError('Frontend pinout encoding changed')
    pinouts = [{'value': int(value), 'label': label} for value, label in
               re.findall(r'value:(\d+),label:"([^"]+)"', pinout_match[1])]
    if len(pinouts) != 4:
        raise ValueError('Frontend pinout coverage changed')
    records, api = {}, {}
    for name, path in ENDPOINTS.items():
        response = client.get(BASE + path)
        records[name] = response
        try:
            payload = json.loads(response['body'])
        except json.JSONDecodeError:
            payload = {}
        ok = response['status'] == 200 and payload.get('code') == 200 and isinstance(payload.get('result'), list)
        api[name] = {'status': 'captured' if ok else 'unavailable', 'source': source(response),
                     'items': payload['result'] if ok else []}
    styles = []
    for index, row in enumerate(api['wire_styles']['items']):
        if not isinstance(row, dict) or not isinstance(row.get('productMpn'), str):
            raise ValueError('Malformed wire style')
        styles.append({'id': 'style-' + hashlib.sha256(json.dumps(row, sort_keys=True).encode()).hexdigest()[:16], 'raw': row,
                       'source': api['wire_styles']['source'],
                       'engineering_verification': 'unverified',
                       'missing': ['exact_wire_mpn', 'conductor_construction', 'insulation_od_mm',
                                   'temperature_range', 'minimum_bend_radius_mm', 'dimensional_tolerances']})
    parts = [part(token, kind, api[key]['source']) for key, kind in
             [('housings', 'housing'), ('terminals', 'contact')] for token in api[key]['items']]
    if len({p['id'] for p in parts}) != len(parts):
        raise ValueError('Duplicate supplier part IDs')
    families = []
    for service in services:
        options = service['options'] if service.get('quote') else [None]
        for option in options:
            terminal = service['value'] == 'consumer_electronics' and option['value'] == 'terminal_wire'
            families.append({'service': service['value'], 'segment': option['value'] if option else None,
                             'label': option['label'] if option else service['label'],
                             'configuration': 'online_terminal_harness' if terminal else 'rfq',
                             'source': source(quote), 'engineering_verification': 'unverified'})
    return {'schema_version': 1, 'supplier_id': 'lcsccable', 'page_source': source(page),
            'coverage': {'selector_lists_complete': all(a['status'] == 'captured' for a in api.values()),
                         'scope': 'Public selector lists only; connector search may expose additional parts. No Cartesian compatibility inferred.',
                         'api': {k: {x: v[x] for x in ('status', 'source')} for k, v in api.items()},
                         'wire_style_count': len(styles), 'part_count': len(parts),
                         'parts_by_kind': dict(Counter(p['kind'] for p in parts))},
            'families': families, 'wire_styles': styles, 'gauge_options': api['gauges']['items'],
            'parts': parts,
            'frontend': {'source': source(quote), 'shared_source': source(shared),
                         'shapes': shapes, 'colors': colors, 'treatments': treatments,
                         'length_units': units, 'pinouts': pinouts,
                         'length_basis': 'wire_only_excluding_housing_and_terminals',
                         'minimum_length_mm': 30, 'length_input': 'positive_integer_in_selected_unit',
                         'rule_source': source(rules),
                         'pinout_interpretation': 'Supplier labels only; recipes require explicit cavity maps.'}}


def matrix(catalog, reference=None):
    rows = [{'id': s['id'], 'kind': 'wire_style', 'designation': s['raw'].get('displayMpn'),
             'status': 'selection_only', 'missing': s['missing']} for s in catalog['wire_styles']]
    rows.extend({'id': p['id'], 'kind': p['kind'], 'designation': p['mpn'],
                 'status': 'identity_only', 'missing': ['part_drawing', 'compatibility', 'geometry_datums']}
                for p in catalog['parts'])
    reviewed = {p['id']: p for p in (reference or {}).get('parts', [])}
    for row in rows:
        evidence = reviewed.get(row['id'])
        if evidence:
            row['status'] = 'documented_nominal_envelope' if row['kind'] == 'housing' else 'documented_wire_compatibility'
            row['source'] = evidence['source']
            row['missing'] = evidence['unknown']
    groups = {}
    for style in catalog['wire_styles']:
        groups.setdefault(style['raw']['productMpn'], []).append(style['raw'])
    return {'schema_version': 1, 'coverage': catalog['coverage'], 'items': rows,
            'repeated_style_designations': {k: v for k, v in groups.items() if len(v)>1},
            'documented_part_count': sum(r['status'].startswith('documented_') for r in rows),
            'manufacturing_qualified_count': 0, 'verified_manufacturer_cad_count': 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--refresh', action='store_true')
    mode.add_argument('--offline', action='store_true')
    args = parser.parse_args()
    catalog = capture(Client(args.output / 'raw', args.refresh, args.offline))
    write_json(args.output / 'catalog.json', catalog)
    reference_path = Path(__file__).with_name('lcsccable-reference.json')
    reference = json.loads(reference_path.read_text()) if reference_path.exists() else None
    coverage = matrix(catalog, reference)
    write_json(args.output / 'coverage.json', coverage)
    with (args.output / 'coverage.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['id', 'kind', 'designation', 'status', 'missing'])
        for row in coverage['items']:
            writer.writerow([row['id'], row['kind'], row['designation'], row['status'], '; '.join(row['missing'])])
    print(json.dumps(catalog['coverage'], indent=2))


if __name__ == '__main__':
    main()
