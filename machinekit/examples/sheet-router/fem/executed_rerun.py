"""Bind executed J6 histories to a corrected structural model.

Default is a read-only producer audit and compact rerun plan, permitted below
8 GB. --models enables modal replay only above that floor. Never invokes CCX,
changes frozen CAD, copies producer histories or compares their cycle times.
"""
import argparse
import hashlib
import importlib
import itertools
import json
import math
import re
import shutil
import sys
from pathlib import Path

import numpy as np

DATASETS = {'j6-executed-pose-grid': ('exports', 90),
            'j6-executed-phase-grid': ('histories', 64),
            'j6-full-feed-resume': ('histories', 32)}
FLOOR = 8_000_000_000


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def inside(root, relative):
    child = Path(relative)
    if child.is_absolute() or '..' in child.parts:
        raise ValueError('Producer index path must stay inside its dataset')
    path = root / child
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError('Producer path escapes its dataset')
    return path


def pose_for(prefix, metadata):
    parts = Path(prefix).parts
    beds = [int(p[1:]) for p in parts if re.fullmatch(r'y(2600|3150)', p)]
    poses = [int(p[1:]) for p in parts if re.fullmatch(r'p[0-8]', p)]
    if len(beds) != 1 or len(poses) != 1:
        raise ValueError('Executed history needs an unambiguous bed and nominal pose')
    bed, i = beds[0], poses[0]
    expected = [(.0, .8, 1.6)[i//3], (0, bed/2000, bed/1000)[i % 3], .2]
    origin = metadata['fixture']['placement']['originM']
    if len(origin) != 3 or not np.allclose(origin, expected, rtol=0, atol=1e-9):
        raise ValueError('History placement disagrees with its nominal structural pose')
    return bed, dict(zip(('x', 'y', 'z'), [v*1000 for v in expected]))


def check_clock(metadata, manifest):
    control = metadata.get('control')
    if control is None:
        if 'actual host executor completed' not in metadata.get('scope', ''):
            raise ValueError('Nominal history lacks executed provenance')
        return
    if 'Observed actual host executor' not in metadata.get('scope', ''):
        raise ValueError('Offline compiler suffix is not an executed J6 history')
    if not 0 < control['feedOverride'] <= 1:
        raise ValueError('This handoff covers only at-or-below-100% overrides')
    clock = metadata['clockCertificate']
    if clock != manifest['clockCertificate']:
        raise ValueError('Metadata and manifest clock certificates differ')
    checks = clock['checks']
    if not {1, 2, 3}.issubset({c['kind'] for c in checks if c['status'] == 1}) or any(c['status'] == 2 for c in checks):
        raise ValueError('Missing or failed mapped-clock V/A/J certificate')
    config = metadata['config']
    # Reduced contour jerk and assumed physical drive jerk are distinct inputs.
    # J6 stops/clock transitions certify the latter; do not silently relabel it.
    caps = [config.get('driveJerk'+axis.upper(), config['jerk'+axis.upper()]) for axis in ('x','y','z')]
    mapped = np.asarray(clock['jointMappedBounds'], dtype=float)
    limits = np.asarray([[2/3, 3, cap] for cap in caps])
    if not np.isfinite(limits).all() or np.any(limits<=0) or mapped.shape != (3, 3) or not np.isfinite(mapped).all() or np.any(mapped < 0) or np.any(mapped > limits*(1+1e-6)+1e-6):
        raise ValueError('Mapped-clock bounds exceed the declared drive envelope')
    step=clock['mappedAccelerationStepBound'];allocation=config['clockAdmission']['accelerationStepMPerS2']
    if not math.isfinite(step) or step<0 or not math.isfinite(allocation) or allocation<=0 or step > allocation*(1+1e-6):
        raise ValueError('Mapped acceleration seam exceeds its admission allocation')
    if not metadata['runnerReport']['completionChecked']:
        raise ValueError('Executor run did not complete its outcome check')


def audit(root, reader):
    records, indices, verified = [], {}, {}
    reader_path = reader / 'curved_error.py'
    before_reader = sha(reader_path)
    sys.path.insert(0, str(reader.resolve()))
    loader = importlib.import_module('curved_error').load_export
    def verify(path, expected):
        path = Path(path)
        # Hash each shared immutable producer once; output CSVs remain streamed.
        key = (str(path), expected)
        if key not in verified:
            if sha(path) != expected:
                raise ValueError('Handoff content hash mismatch: '+str(path))
            verified[key] = True
    for dataset, (key, expected_count) in DATASETS.items():
        folder = root/dataset
        index_path = folder/'INDEX.json'
        before = sha(index_path)
        index = json.loads(index_path.read_text())
        entries = index[key]
        if len(entries) != expected_count or len({e['prefix'] for e in entries}) != expected_count:
            raise ValueError('Incomplete or duplicate J6 dataset: '+dataset)
        for entry in entries:
            prefix = inside(folder, entry['prefix'])
            for name, digest in entry['sha256'].items():
                verify(inside(prefix.parent, name), digest)
            meta, samples, manifest = loader(prefix, prefix.with_suffix('.manifest.json'))
            check_clock(meta, manifest)
            for path, digest in manifest['identity'].items():
                verify(path, digest)
            for path, digest in manifest['actualLoadedPlannerLibraries'].items():
                relocated = manifest.get('libraryRelocations', {}).get(path, path)
                verify(relocated, digest)
            bed, pose = pose_for(entry['prefix'], meta)
            physical_jerk = [meta['config'].get('driveJerk'+a, meta['config']['jerk'+a]) for a in ('X','Y')]
            maxima = np.max(np.abs(samples[:, 3:9]), axis=0)
            if not np.all(maxima <= np.array([2/3,2/3,3,3,*physical_jerk])*(1+1e-6)+1e-6):
                raise ValueError('Executed samples exceed assumed drive V/A/J')
            if not np.isfinite(samples).all() or samples[0,0] != 0 or np.any(np.diff(samples[:,0]) <= 0):
                raise ValueError('Invalid executed sample time series')
            if np.max(np.abs(samples[[0,-1],3:7])) > 1e-6:
                raise ValueError('Executed history lacks stopped PVA boundaries')
            records.append(dict(dataset=dataset, prefix=entry['prefix'], bedMm=bed, poseMm=pose,
                sha256=entry['sha256'], samples=len(samples),
                scenario=prefix.name.removeprefix(str(bed)+'-'),
                contourJerkXY=[meta['config']['jerkX'],meta['config']['jerkY']],
                assumedDriveJerkXY=physical_jerk,
                heldRestartFeedOverride=index.get('heldRestartFeedOverride',index.get('scenarioInterpretation',{}).get('heldRestartFeedOverride'))))
        if sha(index_path) != before:
            raise ValueError('Producer index changed during audit; re-run against the re-export')
        indices[dataset] = before
    if len(records) != 186 or sha(reader_path) != before_reader:
        raise ValueError('Incomplete audit or changed reader')
    for bed in (2600,3150):
        poses = {(r['poseMm']['x'],r['poseMm']['y'],r['poseMm']['z']) for r in records
                 if r['bedMm']==bed and r['dataset']=='j6-executed-pose-grid'}
        if poses != {(x,y,200.) for x in (0.,800.,1600.) for y in (0.,bed/2, float(bed))}:
            raise ValueError('Missing structural pose coverage')
    return dict(schema=1, status='Executed-history inputs validated; corrected-model replay pending',
        producerRoot=str(root.resolve()), readerSource=str(reader.resolve()),readerSha256=before_reader,
        indexSha256=indices, histories=records, historyCount=len(records),
        cycleTimeReporting='Suppressed: publication-reserve correction/re-export pending',
        requires=['corrected full-length mounting mass and geometry-derived compliance',
            'both meshes/all poses and qualified weld reactions',
            'revalidate index/output hashes after any re-export'],
        scope='Generic Cartesian executor histories, assumed drives; no selected router hardware or corrected structure pass'), loader


def reserve():
    free = shutil.disk_usage(Path.cwd()).free
    if free < FLOOR:
        raise RuntimeError('No modal replay below 8 GB root reserve; free='+str(free))


def replay(plan, loader, models, response_source, output):
    reserve()
    sys.path.insert(0,str(response_source.resolve()))
    from contour_response import response
    from modal_response import modal_displacement
    results = []
    for record in plan['histories']:
        reserve()
        prefix = Path(plan['producerRoot'])/record['dataset']/record['prefix']
        if sha(Path(plan['readerSource'])/'curved_error.py') != plan['readerSha256']:
            raise ValueError('Reader changed after preflight')
        if sha(Path(plan['producerRoot'])/record['dataset']/'INDEX.json') != plan['indexSha256'][record['dataset']]:
            raise ValueError('Handoff re-exported after preflight; audit the new index first')
        for name,digest in record['sha256'].items():
            if sha(prefix.parent/name)!=digest:raise ValueError('History changed after preflight')
        meta, samples, _ = loader(prefix,prefix.with_suffix('.manifest.json'))
        for name,digest in record['sha256'].items():
            if sha(prefix.parent/name)!=digest:raise ValueError('History changed while loading')
        p=record['poseMm']; candidates=sorted((models/f"y{record['bedMm']}"/'baseline').glob(
            f"x{p['x']:g}-y{p['y']:g}-h*/summary.json"))
        if len(candidates)!=2:
            raise ValueError('Both structural meshes required for '+str(p))
        for model in candidates:
            detail=json.loads(model.read_text())
            if detail['pose'] != p or len(detail['modes'])<40:
                raise ValueError('Corrected pose and at least 40 modes required; do not reuse frozen response results')
            if detail.get('mountingQualification')!='corrected-mass-compliance':
                raise ValueError('Model lacks corrected-mount provenance gate')
            worst={k:None for k in ('peakContourErrorMm','movingHandoverPeakMm','residualAfterStopMm','brakingFollowingPeakMm','physicalRestResidualMm','brakingYPeakMm','peakElasticFollowingMm')}
            dt=meta['config']['dt'];end=samples[-1,0]
            now=np.arange(1,math.ceil((end+.5)/dt)+1)*dt
            def acc(t):
                a=np.column_stack([np.interp(t,samples[:,0],samples[:,i]) for i in (5,6)])
                a[(t<0)|(t>=end)]=0
                return a
            whole,mid=acc(now),acc(now-dt/2)
            control=meta.get('control')
            for scale,damping in itertools.product(np.linspace(.85,1.15,129),(.005,.0075,.01,.015,.02,.025,.03)):
                reserve()
                values=response(detail['modes'],detail['staticAccelerationResponseMmPerMPerS2'],samples,meta['geometry'],
                    float(scale),damping,dt,process_start=meta['processStartTimeS'],handover_window=.5)
                error=modal_displacement(detail['modes'],detail['staticAccelerationResponseMmPerMPerS2'],mid,whole,float(scale),damping,dt)
                if control:
                    braking=(now>=control['holdRequestTimeSeconds'])&(now<=control['physicalHeldTimeSeconds'])
                    values['brakingFollowingPeakMm']=float(np.linalg.norm(error[braking],axis=1).max())
                    values['brakingYPeakMm']=float(np.abs(error[braking,1]).max())
                    if control['safeHeldPrefixOnly']:
                        values['physicalRestResidualMm']=float(np.linalg.norm(error[now>=control['physicalHeldTimeSeconds']],axis=1).max())
                elif meta.get('sourceDurationS') is not None:
                    values['physicalRestResidualMm']=float(np.linalg.norm(error[now>=meta['sourceDurationS']],axis=1).max())
                for key in worst:
                    value=values.get(key)
                    if value is not None and (worst[key] is None or value>worst[key]['valueMm']):
                        worst[key]=dict(valueMm=value,frequencyScale=float(scale),dampingRatio=damping)
            results.append(dict(dataset=record['dataset'],history=record['prefix'],poseMm=p,meshMm=detail['meshMm'],
                structuralSha256=sha(model),worst=worst))
            output.write_text(json.dumps(dict(status='Conditional executed-history modal screen; fatigue/commissioning gates separate',
                cycleTimeReporting=plan['cycleTimeReporting'],results=results),indent=2)+'\n')
    return results


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--handoff-root',type=Path,required=True)
    p.add_argument('--reader-source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--models',type=Path)
    p.add_argument('--response-source',type=Path)
    a=p.parse_args()
    if a.models:reserve()
    plan,loader=audit(a.handoff_root,a.reader_source)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(plan,indent=2)+'\n')
    print('Validated 186 executed histories; cycle-time comparison suppressed',flush=True)
    if a.models:
        if not a.response_source:p.error('--response-source required for replay')
        replay(plan,loader,a.models,a.response_source,a.output.with_name(a.output.stem+'-response.json'))


if __name__=='__main__':main()
