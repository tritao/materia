#!/usr/bin/env python3
"""Traceable original STEP BOM; unknown fastener sizes remain procurement gaps."""
from collections import defaultdict
import csv
import hashlib
import json
from pathlib import Path
from reference_dynamics import ROOT,load_json


def main():
    frozen=load_json(ROOT/'reference-baseline.json');layout=load_json(ROOT/'reference-layout.json')
    archive=ROOT/'build/zeroerr-5kg.zip'
    assert hashlib.sha256(archive.read_bytes()).hexdigest()==layout['source']['sha256']
    assert hashlib.sha256((ROOT/'reference-layout.json').read_bytes()).hexdigest()==frozen['layoutSha256']
    assets={a['id']:a for a in layout['assets']};groups=defaultdict(list)
    for o in layout['occurrences']:groups[o['asset']].append(o['id'])
    rows=[]
    for asset,occurrences in groups.items():
        source=assets[asset]
        assert hashlib.sha256((ROOT/'build/vendor'/(asset+'.STEP')).read_bytes()).hexdigest()==source['sha256']
        module=all(x.startswith('joint') for x in occurrences)
        family='eRob110T' if module and source['file'].startswith('eRob110') else 'eRob70T' if module else None
        rows.append(dict(id=asset,description=source['file'],quantity=len(occurrences),occurrences=occurrences,
            sourceSha256=source['sha256'],solidCountPerOccurrence=source['solidCount'],
            procurementUnit='complete module including internal hardware' if module else 'supplier part/assembly',
            family=family,likelyOrderingCode=('eRob110H120T-BM-18EN' if family=='eRob110T' else 'eRob70H100T-BM-18EN') if module else None,
            codeStatus='RFQ hypothesis: manual figure 6-7 shows proximal 110H120I and distal 70H100I; T and options need confirmation' if module else 'part drawing/STEP identity only'))
    hardware=[]
    # Thread-hole capacities are not the number of installed purchased screws.
    for joint in range(1,7):
        hardware.append(dict(id='joint'+str(joint)+'-mounting-hardware',quantity=None,
            outputThread='M4' if joint<=3 else 'M3',
            outputDrawingHoleCount=15 if joint<=3 else 14,
            outputDrawingThreadDepthMm=10 if joint<=3 else 6,
            screwLengthMm=None,grade=None,washerDowelSeal=None,
            status='Hole capacity from module drawing. Installed quantity, length, grade and preload require assembly/drawing confirmation; do not buy all holes as screws.'))
    budget=[('110T module',3,1200,2000),('70T module',3,800,1300),
            ('original machined links/base/adapters/flange kit',1,1200,3000),
            ('mounting fasteners/dowels/seals',1,150,400),('harness/connectors/batteries',1,350,900),
            ('48 V supply/regeneration/distribution',1,500,1200),('controller/EtherCAT master and cabinet',1,500,1500),
            ('STO/emergency-stop/protection hardware',1,300,900),('single-joint fixture/metrology allowance',1,400,1500)]
    costs=[dict(item=n,quantity=q,unitLowUsd=lo,unitHighUsd=hi,extendedLowUsd=q*lo,extendedHighUsd=q*hi) for n,q,lo,hi in budget]
    output=dict(status='reference_build_package_rfq_ready_not_manufacturing_release',source=layout['source'],
        occurrences=len(layout['occurrences']),stepRows=rows,fastenerRegister=hardware,
        budget=dict(currency='USD',basis='Engineering planning allowances; no supplier prices/quotation received; excludes tax, shipping and engineering labour',
                    items=costs,totalLowUsd=sum(x['extendedLowUsd'] for x in costs),totalHighUsd=sum(x['extendedHighUsd'] for x in costs),contingencyPercent=25),
        unresolved=['Exact module codes/ratios/options','Installed external fastener quantities/lengths/grades/preloads',
                    'Connector part numbers for STEP aviation connectors','Harness lengths and approved travel loops',
                    'Supply/regeneration sizing and vendor approval of power topology','Supplier quote and as-delivered revisions'])
    build=ROOT/'build/reference-build';build.mkdir(exist_ok=True)
    (build/'bom.json').write_text(json.dumps(output,indent=2,ensure_ascii=False)+'\n')
    with (build/'bom.csv').open('w') as handle:
        writer=csv.writer(handle);writer.writerow(['id','description','qty','likely_code','occurrences','source_sha256'])
        for row in rows:writer.writerow([row['id'],row['description'],row['quantity'],row['likelyOrderingCode'] or '',','.join(row['occurrences']),row['sourceSha256']])
    print('Verified',len(layout['occurrences']),'STEP occurrences;',len(rows),'BOM rows; six complete modules')
    print('Planning allowance USD:',output['budget']['totalLowUsd'],'-',output['budget']['totalHighUsd'])


if __name__=='__main__':main()
