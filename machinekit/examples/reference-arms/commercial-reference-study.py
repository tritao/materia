#!/usr/bin/env python3
"""Manufacturer DH oracle versus fresh native FK; source precision differences bounded, never fitted."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
BUILD=ROOT/'build/manufacturer-validation'
SOURCE='https://fairino-doc-en.readthedocs.io/latest/CobotsManual/robot_brief_introduction.html'


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def design_module():
    spec=importlib.util.spec_from_file_location('manufacturer_helpers',ROOT/'arm-design-study.py')
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result


def prepare():
    BUILD.mkdir(parents=True,exist_ok=True)
    snapshot=ROOT/'fr5-manufacturer-dh.json'
    evidence=json.loads(snapshot.read_text())
    rows=evidence['rows']
    assert evidence['source']==SOURCE and len(rows)==6
    a=[float(r[2])/1000 for r in rows];d=[float(r[3])/1000 for r in rows]
    alpha=[{'π/2':.5,'-π/2':-.5,'0':0}[r[4]] for r in rows]
    assert a==[0,-.425,-.395,0,0,0] and d==[.152,0,0,.102,.102,.1]
    assert alpha==[.5,0,0,.5,-.5,0] and all(float(r[1])==0 for r in rows)
    design=design_module();oracles=json.loads((ROOT/'arm-reference-oracles.json').read_text())
    fr=dict(kind='manufacturer_nominal_standard_DH',source=SOURCE,section='2.4 table 2.4-5',
            aMetres=a,dMetres=d,alphaOverPi=alpha,baseRotationZRad=0.,
            baseAndToolConvention='ROS base_link equals manufacturer base; DH terminal flange includes 100 mm offset; common TCP adds 150 mm; no fitted transforms',
            snapshotSha256=digest(snapshot),sourceRows=[r[:5] for r in rows])
    models=[]
    for id,oracle in [('ur5e',oracles['ur5e']),('fairino5_v6',fr)]:
        manifest=json.loads((ROOT/'build'/id/'manifest.json').read_text())
        joints=[j for j in manifest['joints'] if j['type']!='fixed'];limits=[[j['limits']['lower'],j['limits']['upper']] for j in joints]
        probes=[[0.]*6]+[[math.radians(sign*30) if j==axis else 0. for j in range(6)] for axis in range(6) for sign in [-1,1]]
        probes += [[lo+(hi-lo)*design.halton(n,b) for (lo,hi),b in zip(limits,[2,3,5,7,11,13])] for n in range(1,129)]
        models.append(dict(id=id,oracle=oracle,probes=probes,manifestSha256=digest(ROOT/'build'/id/'manifest.json'),
                           urdfSha256=digest(ROOT/'build'/id/'robot.urdf'),limits=limits,
                           sceneSha256=digest(ROOT/'build'/id/(id+'.mtrg'))))
    result=dict(models=models,manufacturerSnapshot=dict(url=SOURCE,sha256=digest(snapshot)),
                sourceLockSha256=digest(ROOT/'source-lock.json'),helperSha256=digest(ROOT/'arm-design-study.py'),producerSha256=digest(Path(__file__)))
    (BUILD/'inputs.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Prepared independent manufacturer oracles for UR5e and FR5')


def precision_bound(manifest):
    """Bound FR5 nominal-vs-URDF difference from documented dimensions/RPY precision."""
    design=design_module();joints=[j for j in manifest['joints'] if j['type']!='fixed']
    translations=[[0,0,0],[0,0,.152],[-.425,0,0],[-.395,0,0],[0,0,.102],[0,0,.102]]
    rolls=[0,math.pi/2,0,0,math.pi/2,-math.pi/2]
    actual=[design.frame(j['origin']) for j in joints]
    position=rotation=0.;rows=[]
    lengths=[np.linalg.norm(a[:3,3]) for a in actual]+[.25]
    for i,(a,t,roll) in enumerate(zip(actual,translations,rolls)):
        r=design.rotation([1,0,0],roll)
        rotation_error=float(np.linalg.norm(a[:3,:3]-r,2))
        distance=float(np.linalg.norm(a[:3,3]-t))
        lever=sum(lengths[i+1:])
        position+=distance+rotation_error*lever;rotation+=rotation_error
        rows.append(dict(joint=joints[i]['id'],translationDifferenceM=distance,rotationOperatorDifference=rotation_error,
                         downstreamLengthBoundM=lever))
    return dict(translationBoundM=position+2e-9,rotationOperatorBound=rotation+2e-9,derivation=rows,
                basis='Triangle inequality over fixed source-origin differences; sum rotation operator error times downstream length. No samples used to select bounds.')


def review():
    data=json.loads((BUILD/'inputs.json').read_text());native=json.loads((BUILD/'native.json').read_text());design=design_module()
    assert native['inputSha256']==digest(BUILD/'inputs.json')
    assert native['producerSha256']==digest(ROOT/'CommercialManufacturerChecks.hx')
    assert native['moduleSha256']==digest(ROOT/'build/host/main.hl')
    assert data['producerSha256']==digest(Path(__file__))
    assert data['helperSha256']==digest(ROOT/'arm-design-study.py')
    assert data['sourceLockSha256']==digest(ROOT/'source-lock.json')
    assert data['manufacturerSnapshot']['sha256']==digest(ROOT/'fr5-manufacturer-dh.json')
    results=[]
    for model,observed in zip(data['models'],native['models']):
        id=model['id'];directory=ROOT/'build'/id
        assert model['manifestSha256']==digest(directory/'manifest.json') and model['urdfSha256']==digest(directory/'robot.urdf')
        assert model['sceneSha256']==digest(directory/(id+'.mtrg'))
        manifest=json.loads((directory/'manifest.json').read_text())
        assert manifest['sourceFiles']==json.loads((ROOT/'source-lock.json').read_text())['models'][id]['files']
        for link in manifest['links']:
            for mesh in link['visuals']+link['collisions']:
                for ext,sha in mesh['sha256'].items():
                    if sha is not None:assert digest(directory/(mesh['stem']+'.'+ext))==sha
        assert observed['id']==id and len(observed['observations'])==len(model['probes'])==141
        source_fk=design.urdf_fk(directory/'robot.urdf',manifest['tip'],design.frame(manifest['flangeOffset']) if manifest.get('flangeOffset') else np.eye(4))
        bounds=precision_bound(manifest) if id=='fairino5_v6' else dict(translationBoundM=1e-8,rotationOperatorBound=1e-8,basis='UR nominal source precision tolerance')
        max_p=max_r=max_source=max_jac=0.
        for q,observation in zip(model['probes'],observed['observations']):
            assert q==observation['q'];actual=design.frame(observation['tcp']);nominal=design.dh_fk(q,model['oracle'])
            max_p=max(max_p,float(np.linalg.norm(actual[:3,3]-nominal[:3,3])))
            max_r=max(max_r,float(np.linalg.norm(actual[:3,:3]-nominal[:3,:3],2)))
            source=source_fk(q);max_source=max(max_source,float(np.max(abs(actual-source))))
            max_jac=max(max_jac,float(np.max(abs(np.array(observation['jacobian']).reshape(6,6)-design.numeric_jacobian(source_fk,q)))))
        assert max_p<=bounds['translationBoundM'] and max_r<=bounds['rotationOperatorBound'],id+' manufacturer DH mismatch'
        assert max_source<1e-8 and max_jac<1e-5,id+' compiled model/source inconsistency'
        results.append(dict(id=id,passed=True,status='trusted_nominal_manufacturer_DH_regression_with_explicit_source_precision',
            configurations=141,manufacturer=model['oracle'],maxManufacturerPositionErrorM=max_p,
            maxManufacturerRotationOperatorError=max_r,maxSourceMatrixError=max_source,maxJacobianError=max_jac,
            precisionBound=bounds,physicalCalibrationValidated=False,controllerRecordedSamples=False))
        print(id,'manufacturer FK passed; max position error metres',max_p,'rotation',max_r)
    result=dict(passed=True,models=results,scope='Nominal independent manufacturer kinematics; no fit or physical calibration claim',
                producerSha256=digest(Path(__file__)),inputSha256=digest(BUILD/'inputs.json'),
                nativeSha256=digest(BUILD/'native.json'),cncTendingInScope=False)
    (BUILD/'review.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--prepare',action='store_true');args=parser.parse_args()
    prepare() if args.prepare else review()
