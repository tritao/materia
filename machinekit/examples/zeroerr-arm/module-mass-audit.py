#!/usr/bin/env python3
"""Audit original-arm module/mass evidence and sampled load sensitivity; no cell inputs."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import shutil
import subprocess
import urllib.parse
import urllib.request
from zipfile import ZipFile
import numpy as np

ROOT = Path(__file__).resolve().parent
BUILD = ROOT/'build/module-audit'
REF = ROOT.parent/'reference-arms'
INPUTS = {}


def digest(path):
    value = hashlib.sha256(path.read_bytes()).hexdigest()
    INPUTS[str(path.resolve())] = value
    return value


def read(path):
    digest(path)
    return json.loads(path.read_text())


def module(name, path):
    digest(path)
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def capacity(gravity, inertia, payload_g, payload_m, rated, accel, factor, cap):
    """Largest zero-connected load interval passing every sampled convex constraint."""
    def usage(load):
        demand = (np.abs(gravity+load*payload_g) +
                  np.abs(inertia+load*payload_m).sum(axis=2)*np.deg2rad(accel))*factor
        return demand/rated
    empty = usage(0)
    if empty.max() > 1:
        row, joint = np.unravel_index(empty.argmax(), empty.shape)
        return dict(status='arm_alone_exceeds_assumed_rating', totalFlangeLoadKg=None,
                    limitingJoint='j'+str(joint+1), witnessIndex=int(row), zeroLoadMaxRatio=float(empty.max()))
    lo, hi = 0., cap
    for _ in range(45):
        mid = (lo+hi)/2
        if usage(mid).max() <= 1:
            lo = mid
        else:
            hi = mid
    row, joint = np.unravel_index(usage(lo).argmax(), empty.shape)
    return dict(status='target_cap_reached' if usage(cap).max()<=1 else 'sampled_limit',
                totalFlangeLoadKg=float(cap if usage(cap).max()<=1 else lo),
                limitingJoint='j'+str(joint+1), witnessIndex=int(row))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    capture = parser.add_mutually_exclusive_group()
    capture.add_argument('--refresh', action='store_true', help='Capture current official module pages')
    capture.add_argument('--reuse-reviewed-snapshots', action='store_true', help='Copy previously captured official pages, retaining their original dates and hashes')
    args = parser.parse_args()
    BUILD.mkdir(parents=True, exist_ok=True)
    config = read(ROOT/'original-comparison-config.json')
    layout = read(ROOT/'reference-layout.json')
    geometry = read(ROOT/'build/engineering-input.json')
    assert geometry['layoutSha256'] == digest(ROOT/'reference-layout.json'), 'Stale STEP properties'
    assert digest(ROOT/'build/zeroerr-5kg.zip') == layout['source']['sha256']
    # Reject stale standalone observations rather than silently reusing changed geometry/code.
    previous = read(REF/'build/arm-design/review.json')
    for path, sha in previous['inputs'].items():
        assert digest(Path(path)) == sha, 'Changed standalone dependency: '+path
    observed = read(REF/'build/arm-design/zeroerr-original-native.json')
    clear = [o for o in observed['observations'] if o['clear']]
    poses = np.array([o['q'] for o in clear])
    # Horizontal flange reach, excluding the common 150 mm tool extension.
    kinematics = module('audit_kinematics', REF/'arm-design-study.py')
    fk = kinematics.original_fk(geometry,layout)
    flange_reach = np.array([np.linalg.norm((fk(q)@kinematics.translated(-.15))[:2,3]) for q in poses])
    assert len(poses) == previous['originalLoads']['evaluatedPoses']
    requirements = read(REF/'arm-design-requirements.json')
    assumed = requirements['screeningAssumptions']
    engineering = module('audit_engineering', ROOT/'engineering-study.py')
    supplier = module('audit_supplier', ROOT/'original-supplier-review.py')
    assembly = ROOT/'build/original-evaluation/supplier/assembly.pdf'
    assembly_text = subprocess.check_output(['pdftotext','-layout',str(assembly),'-'], text=True)
    assert digest(assembly) == '14b70632a78f1b4a30f4ec3f2c55e470b0e5f55a0029e277c3b58985136dbeff'
    manual = BUILD/'manual.pdf'
    if not manual.exists():
        manual.write_bytes(urllib.request.urlopen(urllib.parse.quote(config['manual']['url'],safe=':/?=&'), timeout=45).read())
    assert digest(manual) == config['manual']['sha256'], 'Manual revision changed'
    text = subprocess.check_output(['pdftotext','-layout',str(manual),'-'], text=True)
    manual_values = {
        'eRob110T': dict(massKg=4.49, cogFromOutputMm=[59.08,4.09,-.02],
                        inertiaAtOutputGmm2=[7932281.31,23979335.11,24380530.78,1340312.33,-11423.88,1693.37],
                        massTable='A3-7', inertiaTable='A3-8', bearingC_N=21800, bearingC0_N=35800,
                        bearingStaticMomentNm=258, unpoweredBrakeAcceptanceNmAtNominal100=206),
        'eRob70T': dict(massKg=1.24, cogFromOutputMm=[40.06,2.62,-.06],
                       inertiaAtOutputGmm2=[955610.80,3098889.83,3157219.,152791.69,-5421.69,875.15],
                       massTable='A3-1', inertiaTable='A3-2', bearingC_N=5800, bearingC0_N=8600,
                       bearingStaticMomentNm=74, unpoweredBrakeAcceptanceNmAtNominal100=50)}
    material_specs = [('base','2-091-001','7075-T6',2810),
                      ('baseCover','3-091-001','AL6061',2700),
                      ('upperArm','2-091-002','7075-T6',2810),
                      ('forearm','2-091-005','AL6061',2700),
                      ('elbowOutputAdapter','2-130-003','7075-T6',2810),
                      ('elbowHousingAdapter','2-130-004','7075-T6',2810),
                      ('toolFlange','2-054-006','7075-T6',2810)]
    materials=[]
    with ZipFile(ROOT/'build/zeroerr-5kg.zip') as archive:
        for part, drawing_id, alloy, density in material_specs:
            entry=next(e for e in archive.infolist() if drawing_id in e.filename and e.filename.endswith('.pdf'))
            drawing=BUILD/(drawing_id+'.pdf');drawing.write_bytes(archive.read(entry))
            drawing_text=subprocess.check_output(['pdftotext','-layout',str(drawing),'-'],text=True)
            assert alloy in drawing_text, 'Changed material callout: '+drawing_id
            solid=next(s for s in geometry['solids'] if s['id']==part)
            old_mass=next(b[1] for b in engineering.Study(geometry,config).bodies if b[0]==part)
            materials.append(dict(part=part,drawing=drawing_id,alloyCallout=alloy,
                                  assumedDensityKgM3=density,drawingSha256=digest(drawing),
                                  previousMassKg=old_mass,materialBasedMassKg=solid['volumeMm3']*density*1e-9))
    for family, values in manual_values.items():
        row = re.search(family+r'\s+V4\s+含制动器\s+([\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', text)
        assert row and np.allclose([float(x) for x in row.groups()], [values['massKg']]+values['cogFromOutputMm'])
        inertia_row = re.search(family+r'\s+V4\s+含制动器\s+'+r'\s+'.join([r'([-\d.]+)']*6),text)
        assert inertia_row and np.allclose([float(x) for x in inertia_row.groups()],values['inertiaAtOutputGmm2'])
    def catalog(family):
        candidate = config['catalog'][family]
        html, meta = BUILD/(family+'.html'), BUILD/(family+'-snapshot.json')
        if args.reuse_reviewed_snapshots:
            previous_dir = ROOT/'build/original-evaluation/supplier'
            prior = json.loads((previous_dir/meta.name).read_text())
            assert hashlib.sha256((previous_dir/html.name).read_bytes()).hexdigest() == prior['sha256']
            shutil.copyfile(previous_dir/html.name,html)
            shutil.copyfile(previous_dir/meta.name,meta)
        if args.refresh or not html.exists():
            request = urllib.request.Request(candidate['productUrl'],headers={'User-Agent':'Materia arm-design audit'})
            html.write_bytes(urllib.request.urlopen(request,timeout=45).read())
            meta.write_text(json.dumps(dict(url=candidate['productUrl'],fetchedAtUtc=datetime.now(timezone.utc).isoformat(),
                                           sha256=hashlib.sha256(html.read_bytes()).hexdigest()),indent=2)+'\n')
        snapshot = json.loads(meta.read_text())
        assert hashlib.sha256(html.read_bytes()).hexdigest() == snapshot['sha256']
        table = supplier.Tables();table.feed(html.read_text())
        def numbers(label):
            rows = [r for r in table.rows if r and label in r[0]]
            assert len(rows)==1, label
            return [float(n) for cell in rows[0][1:] for n in re.findall(r'\d+(?:\.\d+)?',cell)]
        rated, speeds, masses = numbers('额定扭矩'), numbers('输出端峰值转速'), numbers('重量')
        assert len(rated)==len(speeds)==len(candidate['ratios']) and len(masses)==1
        return dict(family=family,snapshot=snapshot, massKg=masses[0],ratedTorqueNm=rated,
                    peakSpeedRpm=speeds,ratios=candidate['ratios'])
    with ThreadPoolExecutor(max_workers=2) as pool:
        catalogs = {c['family']:c for c in pool.map(catalog,config['catalog'])}
    # Register input hashes on the main thread.
    for family in catalogs:
        digest(BUILD/(family+'.html'));digest(BUILD/(family+'-snapshot.json'))
        drawing=ROOT/('build/110t-interface-drawing.pdf' if family=='eRob110T' else 'build/70t-interface.pdf')
        assert digest(drawing)==config['catalog'][family]['pdfSha256']
    baseline = engineering.Study(geometry,config)
    bodies=[]
    for name,mass,cog,inertia,active in baseline.bodies:
        if name=='tool+payload': continue
        bodies.append(dict(id=name,estimatedMassKg=mass,zeroPoseCogMetres=cog.tolist(),
                           estimatedCentralInertiaKgM2=inertia.tolist(),activeJoints=['j'+str(i+1) for i in active],
                           massBasis='catalog mass distributed by exterior volume' if name.startswith('joint') else 'CAD volume times assumed density'))
    joints=[]
    for i,j in enumerate(config['jointCandidates']):
        family=j['family'];cat=config['catalog'][family];index=cat['ratios'].index(j['nominalRatio'])
        assert re.search('J'+str(i+1)+r'\s*\('+family+r'\)',assembly_text)
        pieces=[b for b in bodies if b['id'].startswith('joint'+str(i+1)+'-')]
        assert np.isclose(sum(b['estimatedMassKg'] for b in pieces),cat['brakeMassKg'])
        joints.append(dict(joint=j['joint'],familyConfirmedByAssembly=family,completeOrderingCode=None,
                           nominalRatio=j['nominalRatio'],ratioStatus='assumed_not_identified_by_STEP_or_assembly_PDF',
                           pinnedDrawingMassKg=cat['brakeMassKg'],manualV4Brake=manual_values[family],
                           publicCatalog=catalogs[family],
                           catalogRatedMatchesPinned=catalogs[family]['ratedTorqueNm']==cat['ratedTorqueNm'],
                           assumedRatedNm=cat['ratedTorqueNm'][index],
                           estimatedOwnModuleMassMovingAboutThisJointKg=sum(b['estimatedMassKg'] for b in pieces if j['joint'] in b['activeJoints']),
                           measuredMovingMassKg=None, movingSideMassStatus='exterior_volume_surrogate_only'))
    scenarios=[]
    variants=[('pinned_drawing',None,1.),('manual_mass_only',manual_values,1.),
              ('public_mass_only',catalogs,1.),('pinned_plus25percent_arm',None,assumed['massSensitivityFactor']),
              ('drawing_materials_only',None,1.)]
    for name, replacements, mass_factor in variants:
        settings=copy.deepcopy(config);settings.pop('missionStudy',None)
        if name=='drawing_materials_only':
            settings['partDensitiesKgM3']={r['part']:r['assumedDensityKgM3'] for r in materials}
        if replacements:
            for family,values in replacements.items(): settings['catalog'][family]['brakeMassKg']=values['massKg']
        settings['application'].update(toolKg=0,payloadKg=0)
        rated=np.array([j['assumedRatedNm'] for j in joints])
        for offset in assumed['combinedCogOffsetsMm']:
            settings['application']['combinedCogAlongToolNormalMm']=offset
            bare=engineering.Study(geometry,settings)
            one_settings=copy.deepcopy(settings);one_settings['application']['payloadKg']=1
            one=engineering.Study(geometry,one_settings)
            zero_terms=[bare.terms(q,mass_factor) for q in poses]
            one_terms=[one.terms(q,mass_factor) for q in poses]
            g=np.array([v[0] for v in zero_terms]);m=np.array([v[1] for v in zero_terms])
            pg=np.array([v[0] for v in one_terms])-g;pm=np.array([v[1] for v in one_terms])-m
            target=requirements['targets']['totalFlangeLoadKg']['value']
            demands=g+target*pg
            witnesses=np.abs(demands).argmax(axis=0)
            screens=[]
            for acceleration,factor in [(0,1.),(0,assumed['designFactor']),(30,assumed['designFactor']),
                                        (assumed['jointAccelerationDegS2'],assumed['designFactor'])]:
                result=capacity(g,m,pg,pm,rated,acceleration,factor,target)
                result.update(accelerationDegS2=acceleration,designFactor=factor,
                              witnessQ=poses[result['witnessIndex']].tolist())
                screens.append(result)
            reach_screens=[]
            for low, high in [(0,.35),(.35,.6),(.6,2.)]:
                indices=np.flatnonzero((flange_reach>=low)&(flange_reach<high))
                if not len(indices): continue
                for acceleration in [0,assumed['jointAccelerationDegS2']]:
                    result=capacity(g[indices],m[indices],pg[indices],pm[indices],rated,
                                    acceleration,assumed['designFactor'],target)
                    result.update(horizontalFlangeReachMetres=[low,high],sampleCount=len(indices),
                                  accelerationDegS2=acceleration,designFactor=assumed['designFactor'],
                                  witnessQ=poses[indices[result['witnessIndex']]].tolist())
                    result['witnessIndex']=int(indices[result['witnessIndex']])
                    reach_screens.append(result)
            scenarios.append(dict(massScenario=name,armMassFactor=mass_factor,combinedCogOffsetMm=offset,
                estimatedArmMassKg=sum(b[1] for b in bare.bodies if b[0]!='tool+payload')*mass_factor,
                targetTotalLoadKg=target,maxGravityNm=np.abs(demands).max(axis=0).tolist(),
                gravityWitnesses=[dict(joint='j'+str(j+1),q=poses[row].tolist(),armTorqueNm=float(g[row,j]),
                    flangeLoadTorqueNm=float(target*pg[row,j]),totalTorqueNm=float(demands[row,j])) for j,row in enumerate(witnesses)],
                capacityScreens=screens,reachBinCapacityScreens=reach_screens))
    result=dict(status='document_audit_complete_delivered_BOM_and_moving_mass_unconfirmed',jointRegister=joints,
                bodies=bodies,materialAudit=materials,scenarios=scenarios,sampledClearPoses=len(poses),
                scope='Original STEP axes and fixed common tool clearance samples. Capacities are zero-connected sampled point-load screens capped at target, not a safe payload rating, duty or bearing qualification. CoG offset zero is an artificial point-load diagnostic. Mass-only substitutions preserve unverified exterior distribution. No cell study.',
                manualFrame='Whole-module inertia about output-face origin; X toward shaft, Y toward base, right-handed Z. g mm^2 to kg m^2 is 1e-9. Not a central inertia or a housing/flange split.',
                sourceConflicts=['110T mass: drawing 4.47 kg, manual V4 4.49 kg, current page 4.64 kg',
                                 '70T mass: drawing 1.16 kg, manual V4 and current page 1.24 kg',
                                 'Public high-load emergency-braking wording conflicts with manual section 7 restrictions; use delivered procedure'],
                hardwareQualified=False,cncTendingInScope=False,inputs=INPUTS.copy(),
                producerSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (BUILD/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Audited six modules,',len(bodies),'body estimates,',len(poses),'clear poses and',len(scenarios),'load cases')
    for row in scenarios:
        if row['combinedCogOffsetMm']==150:
            print(row['massScenario'],'mass',round(row['estimatedArmMassKg'],3),'J2 gravity',round(row['maxGravityNm'][1],3),
                  'static total load',row['capacityScreens'][0]['totalFlangeLoadKg'])


if __name__=='__main__': main()
