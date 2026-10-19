#!/usr/bin/env python3
"""Download pinned HZMotion STEP assets; account credentials stay in memory."""
import argparse
import getpass
import hashlib
import http.cookiejar
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = Path(__file__).with_name('hzmotion-cad.json')


def verified(content, record):
    if not content.lstrip().startswith(b'ISO-10303-21;'):
        raise ValueError('Supplier response is not STEP')
    if len(content) != record['bytes'] or hashlib.sha256(content).hexdigest() != record['sha256']:
        raise ValueError('Supplier CAD changed; review before accepting a new revision')
    return content


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offline', action='store_true')
    args = parser.parse_args()
    records = json.loads(MANIFEST.read_text())['assets']
    cache = ROOT / 'build/suppliers/hzmotion/cad/assets'
    cache.mkdir(parents=True, exist_ok=True)
    missing = []
    for record in records:
        path = cache / (record['sha256'] + '.step')
        if path.exists():
            verified(path.read_bytes(), record)
        else:
            missing.append(record)
    if missing:
        if args.offline:
            raise ValueError('Missing pinned CAD assets')
        opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
        payload = {'email': input('HZMotion email: '), 'password': getpass.getpass('HZMotion password: ')}
        request = urllib.request.Request('https://www.hzmotion.com/api/auth/login',
            data=json.dumps(payload).encode(), headers={'Content-Type': 'application/json',
            'Origin': 'https://www.hzmotion.com'})
        with opener.open(request, timeout=60) as response:
            ok = json.load(response).get('ok')
        del payload, request
        if not ok:
            raise ValueError('Supplier authentication failed')
        for record in missing:
            with opener.open(record['url'], timeout=120) as response:
                content = verified(response.read(), record)
            (cache / (record['sha256'] + '.step')).write_bytes(content)
            print('Downloaded', record['model'])
    print(f'Verified {len(records)} pinned HZMotion STEP assets')


if __name__ == '__main__':
    main()
