#!/usr/bin/env python3
"""Measure the unmodified Halfen ASCII DXF, without a CAD converter."""
import argparse
import hashlib
import json
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('dxf', type=Path)
p.add_argument('--out', type=Path, default=Path(__file__).parent/'build/tfixx-anchor')
a = p.parse_args()
raw = a.dxf.read_bytes()
s = raw.decode('cp1252').splitlines()
assert len(s) % 2 == 0, 'Incomplete DXF'
pairs = [(int(s[i]), s[i+1].strip()) for i in range(0, len(s), 2)]
assert any(pairs[i] == (9, '$INSUNITS') and pairs[i+1] == (70, '4') for i in range(len(pairs)-1)), 'Expected mm'
records, record = [], []
for k, v in pairs:
    if k == 0:
        if record:
            records.append(record)
        record = [(k, v)]
    else:
        record.append((k, v))
records.append(record)
section, entities = '', []
for record in records:
    d = dict(record)
    if d[0] == 'SECTION':
        section = d.get(2, '')
    elif d[0] == 'ENDSEC':
        section = ''
    elif section == 'ENTITIES':
        entities.append(d)
texts = [d.get(1, '') for d in entities if d[0] == 'TEXT']
assert 'T-FIXX M10 x 50 - A4' in texts and '0020.270-00101' in texts
# The first drawing row has a shared side view for the GV/A4 labels, axis along X.
side = [d for d in entities if d[0] == 'LINE' and -70 < float(d.get(20, 0)) < -30 and -70 < float(d.get(21, 0)) < -30]
xs = [float(d[10]) for d in side if abs(float(d[10])-float(d[11])) < 1e-6]
assert any(abs(x) < 1e-6 for x in xs) and any(abs(x-50) < 1e-6 for x in xs)
ys = []
for d in side:
    x, u, y, v = [float(d[k]) for k in (10, 11, 20, 21)]
    if abs(y-v) < 1e-5 and min(x, u) > 18 and max(x, u) < 40 and abs(u-x) > 10:
        ys.append(y)
diameter = max(ys)-min(ys)
assert abs(diameter-13.5) < 1e-5
clip = max(xs)-50
assert abs(clip-1.5) < 1e-5
report = dict(
    source='https://www.halfen.com/Softwaredaten/Downloads/Software-CAD/CAD-Bibliotheken-AutoCAD/Verankerungstechnik/demu_cad_tfixx_dxf.zip',
    sha256=hashlib.sha256(raw).hexdigest(), cadVersion='1.20, 2014-01-30', units='mm',
    candidate='T-FIXX M10 x 50 A4', legacyDrawingId='0020.270-00101', currentCatalogueOrder='1000200636',
    measuredBodyLengthMm=50, measuredSleeveDiameterMm=round(diameter, 6),
    measuredLegacyClipMm=round(clip, 6), currentCatalogueClipMm=2,
    currentCatalogueScrewInLengthMm=32, currentCatalogueEffectiveDepthMm=43.7,
    catalogue='https://www.halfen.com/cdn/Medien%20I%20Media/Druckschriften%20-%20Printed%20materials/Technische%20Produktinformationen/PB_Halfen_Demu-Fix_25_v03-2026_EN.pdf',
    cataloguePages=[14, 15], is3DSolid=False,
    review='Main dimensions agree. Clip differs; legacy/current order equivalence requires supplier confirmation.')
a.out.mkdir(parents=True, exist_ok=True)
(a.out/'cad-check.json').write_text(json.dumps(report, indent=2)+'\n')
print(f'T-FIXX CAD checked: 50 mm body, {diameter:.1f} mm sleeve; legacy clip 1.5 mm vs current 2 mm.')
