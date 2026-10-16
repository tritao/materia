#!/usr/bin/env python3
"""Review pinned supplier drawings and snapshot public catalog differences.

Writes a draft request only; does not contact the supplier or select drives.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
from html.parser import HTMLParser
import json
from pathlib import Path
import re
import subprocess
import urllib.request
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parent
BUILD = ROOT / 'build/original-evaluation/supplier'


class Tables(HTMLParser):
    def __init__(self):
        super().__init__()
        self.rows, self.row, self.cell = [], None, None

    def handle_starttag(self, tag, attrs):
        if tag == 'tr':
            self.row = []
        elif tag in ('td', 'th') and self.row is not None:
            self.cell = []

    def handle_data(self, text):
        if self.cell is not None:
            self.cell.append(text)

    def handle_endtag(self, tag):
        if tag in ('td', 'th') and self.cell is not None:
            self.row.append(re.sub(r'\s+', ' ', ''.join(self.cell)).strip())
            self.cell = None
        elif tag == 'tr' and self.row is not None:
            self.rows.append(self.row)
            self.row = None


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, default=ROOT/'build/zeroerr-5kg.zip')
    parser.add_argument('--refresh', action='store_true')
    args = parser.parse_args()
    BUILD.mkdir(parents=True, exist_ok=True)
    config_path = ROOT/'original-comparison-config.json'
    layout_path = ROOT/'reference-layout.json'
    config = json.loads(config_path.read_text())
    layout = json.loads(layout_path.read_text())
    if digest(args.archive) != layout['source']['sha256']:
        raise ValueError('Changed original supplier archive')
    with ZipFile(args.archive) as archive:
        names = []
        drawing = None
        for entry in archive.infolist():
            name = entry.filename
            if not entry.flag_bits & 0x800:
                try:
                    name = name.encode('cp437').decode('gbk')
                except UnicodeError:
                    pass
            names.append(name)
            if Path(name).name == 'eRobot 5.pdf':
                drawing = archive.read(entry)
        if drawing is None:
            raise ValueError('Missing original assembly drawing')
        (BUILD/'assembly.pdf').write_bytes(drawing)
    subprocess.run(['pdftotext', '-layout', str(BUILD/'assembly.pdf'), str(BUILD/'assembly.txt')], check=True)
    assembly = (BUILD/'assembly.txt').read_text()
    for joint, family in [('J1','eRob110T'),('J2','eRob110T'),('J3','eRob110T'),('J4','eRob70T'),('J5','eRob70T'),('J6','eRob70T')]:
        if not re.search(joint+r'\s*\('+family+r'\)', assembly):
            raise ValueError('Assembly drawing does not confirm '+joint+' '+family)
    inputs = {str(config_path):digest(config_path), str(layout_path):digest(layout_path),
              str(args.archive.resolve()):digest(args.archive), str(BUILD/'assembly.pdf'):digest(BUILD/'assembly.pdf')}
    def catalog(family):
        candidate = config['catalog'][family]
        html = BUILD/(family+'.html')
        record = BUILD/(family+'-snapshot.json')
        if args.refresh or not html.exists():
            request = urllib.request.Request(candidate['productUrl'], headers={'User-Agent':'Materia model review'})
            with urllib.request.urlopen(request, timeout=30) as response:
                body = response.read()
            html.write_bytes(body)
            record.write_text(json.dumps(dict(url=candidate['productUrl'],fetchedAtUtc=datetime.now(timezone.utc).isoformat(),
                                              sha256=digest(html)),indent=2)+'\n')
        snapshot = json.loads(record.read_text())
        if digest(html) != snapshot['sha256']:
            raise ValueError('Changed public catalog snapshot')
        tables = Tables()
        tables.feed(html.read_text())
        def numbers(label):
            rows = [r for r in tables.rows if r and label in r[0]]
            if len(rows) != 1:
                raise ValueError('Catalog table label ambiguous/missing: '+label)
            return [float(v) for cell in rows[0][1:] for v in re.findall(r'\d+(?:\.\d+)?', cell)]
        rated, mass = numbers('额定扭矩'), numbers('重量')
        if len(rated) != len(candidate['ratios']) or len(mass) != 1:
            raise ValueError('Unexpected catalog column count')
        pdf = ROOT/('build/110t-interface-drawing.pdf' if family=='eRob110T' else 'build/70t-interface.pdf')
        if digest(pdf) != candidate['pdfSha256']:
            raise ValueError('Changed pinned V4 drawing')
        return dict(family=family,snapshot=snapshot,files={str(html):digest(html),str(record):digest(record),str(pdf):digest(pdf)},
                    modelAssumedNominalRatio=100,ratios=candidate['ratios'],publicRatedTorqueNm=rated,
                    pinnedV4RatedTorqueNm=candidate['ratedTorqueNm'],ratedTorqueTableMatches=rated==candidate['ratedTorqueNm'],
                    pinnedV4BrakeMassKg=candidate['brakeMassKg'],publicPageMassKg=mass[0],
                    massDifferenceKg=mass[0]-candidate['brakeMassKg'],publicPeakSpeedRpm=numbers('输出端峰值转速'),
                    distinction='Public page labels peak speed; V4 drawing adds 48 V/rated-torque condition. Exact delivered revision still unconfirmed.')
    with ThreadPoolExecutor(max_workers=2) as pool:
        catalogs = list(pool.map(catalog,['eRob110T','eRob70T']))
    for row in catalogs:
        inputs.update(row.pop('files'))
    result = dict(status='public_documents_reviewed_exact_arm_configuration_unconfirmed',archiveSha256=layout['source']['sha256'],
                  archiveMembers=names,assemblyDrawingSha256=digest(BUILD/'assembly.pdf'),moduleFamiliesConfirmedByAssemblyDrawing=True,
                  genericModuleStepNames=[a['file'] for a in layout['assets'] if a['file'].startswith('eRob')],catalogs=catalogs,
                  hypotheticalModuleMassDeltaKg=3*sum(r['massDifferenceKg'] for r in catalogs),
                  unresolved=['Six complete drive ordering codes and physical joint travel/signs',
                              'Revision-matched torque-speed-temperature and continuous holding/duty limits',
                              'Original 5 kg arm payload/CoG/inertia envelope and specified operating conditions',
                              'Measured moving-side mass, centers of mass and inertia tensors',
                              'Flange drawing, harness travel and delivered brake/encoder/ESI configuration'],
                  scope='Reviewed pinned assembly PDF/STEP names and V4 drawings plus captured public pages. Wildcard STEP names and family labels do not establish an exact drive BOM. No mass/catalog substitutions made.',
                  producerSha256=digest(Path(__file__)),inputs=inputs)
    (BUILD/'review.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    load_path = ROOT.parent/'reference-arms/build/cell-registration/original-registered-load-report.json'
    load = json.loads(load_path.read_text())
    refined_load_path = load_path.with_name('original-refined-registered-load-report.json')
    refined_load = json.loads(refined_load_path.read_text()) if refined_load_path.exists() else None
    latest_load = refined_load or load
    state = latest_load['states'][-1]
    shoulder = config['jointCandidates'][1]
    shoulder_catalog = config['catalog'][shoulder['family']]
    rated = shoulder_catalog['ratedTorqueNm'][shoulder_catalog['ratios'].index(shoulder['nominalRatio'])]
    request = dict(status='draft_not_sent_not_ordered',reference='Unchanged ZeroErr eRobot 5 assembly',
                   sourceArchive=layout['source'],jointFamilies=[dict(joint=j['joint'],family=j['family'],completeOrderingCode=None,
                     nominalRatioAssumedForSimulation=j['nominalRatio']) for j in config['jointCandidates']],
                   questions=result['unresolved'],comparisonApplication=config['application'],
                   actualCellTool=dict(toolMassKg=latest_load['states'][0]['toolMassKg'],blankMassKg=latest_load['stockMassKg'],
                                      doubleHeldFlangeLoadKg=state['flangeLoadKg']),
                   originalCellLoadReportSha256=digest(load_path),
                   refinedCellLoadReportSha256=digest(refined_load_path) if refined_load else None,
                   shoulderScreen=dict(nominalGravityNm=state['maxNominalGravityNm'][1],
                     conservativeEnvelopeNm=state['maxHeavyAccelerationEnvelopeWithDesignFactorNm'][1],assumedRatedNm=rated),
                   sizingQuestion='Please supply delivered-drive duty/holding curves; gearbox average/peak values are not treated as continuous motor ratings.',
                   configurationBoundary='Retain original axes and mounting geometry until specifications are confirmed. No 80T substitution requested.')
    (ROOT/'original-supplier-request.json').write_text(json.dumps(request,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(dict(status=result['status'],hypotheticalModuleMassDeltaKg=result['hypotheticalModuleMassDeltaKg'],catalogs=catalogs),indent=2))


if __name__ == '__main__':
    main()
