#!/usr/bin/env python3
"""Compare retained modules with isolated shoulder/wrist sizing concepts, without cell data."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import numpy as np

ROOT=Path(__file__).resolve().parent
REF=ROOT.parent/'reference-arms'
BUILD=ROOT/'build/arm-options'
INPUTS={}


def digest(path):
    result=hashlib.sha256(path.read_bytes()).hexdigest();INPUTS[str(path.resolve())]=result
    return result


def read(path):
    digest(path);return json.loads(path.read_text())


def load_module(name,path):
    digest(path);spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result


def fast_terms(study,q,mass_factor=1.):
    """Vectorized serial-chain rigid-body terms, equivalent to engineering.Study.terms."""
    # This arm is a serial chain; each body has a contiguous ancestor prefix.
    for b in study.bodies:
        if b[4] != list(range(len(b[4]))): raise ValueError('Non-prefix ancestors')
    rotations=[np.eye(3)];translations=[np.zeros(3)]
    for axis,origin,angle in zip(study.axes,study.origins,q):
        x,y,z=axis;k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        r=np.eye(3)+np.sin(angle)*k+(1-np.cos(angle))*(k@k)
        rotations.append(rotations[-1]@r)
        translations.append(translations[-1]+rotations[-2]@(origin-r@origin))
    axes=np.array([rotations[i]@study.axes[i] for i in range(6)])
    origins=np.array([rotations[i]@study.origins[i]+translations[i] for i in range(6)])
    depths=np.array([len(b[4]) for b in study.bodies])
    r=np.array(rotations)[depths];t=np.array(translations)[depths]
    p=np.einsum('nij,nj->ni',r,np.array([b[2] for b in study.bodies]))+t
    factors=np.array([1. if b[0]=='tool+payload' else mass_factor for b in study.bodies])
    masses=np.array([b[1] for b in study.bodies])*factors
    inertia=np.array([b[3] for b in study.bodies])*factors[:,None,None]
    active=np.arange(6)[None,:]<depths[:,None]
    jv=np.cross(axes[None,:,:],p[:,None,:]-origins[None,:,:])*active[:,:,None]
    jw=axes[None,:,:]*active[:,:,None]
    world_inertia=r@inertia@np.swapaxes(r,1,2)
    matrix=np.einsum('n,nic,njc->ij',masses,jv,jv)+np.einsum('nic,ncd,njd->ij',jw,world_inertia,jw)
    matrix+=np.diag(study.reflected)
    gravity=np.einsum('n,ni->i',masses*9.80665,jv[:,:,2])
    return gravity,matrix


def build_study(engineering,geometry,original,options,variant,materials,mass_case,offset,load):
    config=copy.deepcopy(original);config.pop('missionStudy',None)
    config['partDensitiesKgM3']={m['part']:m['assumedDensityKgM3'] for m in materials}
    config['application'].update(toolKg=0,payloadKg=load,combinedCogAlongToolNormalMm=offset)
    for family,cat in options['catalog'].items():
        config['catalog'][family]=dict(brakeMassKg=cat['massKg'],motorRotorInertiaGmm2=cat['motorRotorInertiaGmm2'])
    for j in config['jointCandidates']:
        if j['joint'] in variant['changes']:
            j['family'],j['nominalRatio']=variant['changes'][j['joint']]
    if mass_case=='public_module_masses':
        for j in config['jointCandidates']:
            config['catalog'][j['family']]['brakeMassKg']=options['publicMassSensitivityKg'][j['family']]
    study=engineering.Study(geometry,config)
    # Preserve mass positions for an isolated sizing screen; enlarged housings need new CAD.
    dimensions={'eRob110T':(116,147.5),'eRob70T':(73,99)}
    adjusted=[]
    for name,mass,cog,inertia,active in study.bodies:
        if name.startswith('joint'):
            index=int(name[5])-1;joint='j'+str(index+1)
            if joint in variant['changes']:
                cat=options['catalog'][variant['changes'][joint][0]]
                old=dimensions[original['jointCandidates'][index]['family']]
                scale=max(cat['diameterMm']/old[0],cat['lengthMm']/old[1])
                inertia=inertia*scale**2
        adjusted.append((name,mass,cog,inertia,active))
    allowance=options['assumptions']['connectionMassAllowanceKgPerChangedJoint']
    distance=options['assumptions']['connectionMassOffsetMm']/1000
    for joint in variant['changes']:
        index=int(joint[1:])-1
        if index<5: direction=study.origins[index+1]-study.origins[index]
        else: direction=np.array(geometry['toolNormal'],dtype=float)
        direction=direction/np.linalg.norm(direction)
        adjusted.append(('adapter-'+joint,allowance,study.origins[index]+direction*distance,
                         np.zeros((3,3)),list(range(index+1))))
    study.bodies=adjusted
    return study,config


def limits(config,options):
    rated=[];speed=[]
    for joint in config['jointCandidates']:
        family=joint['family'];ratio=joint['nominalRatio']
        if family in options['catalog']:
            cat=options['catalog'][family];i=cat['ratios'].index(ratio)
            rated.append(cat['ratedTorqueNm'][i]);speed.append(cat['peakSpeedRpm'][i]*6)
        else:
            cat=config['catalog'][family];i=cat['ratios'].index(ratio)
            rated.append(cat['ratedTorqueNm'][i]);speed.append(cat['ratedLoadSpeedRpm48V'][i]*6)
    return np.array(rated),np.array(speed)


def acceleration_bound(gravity,matrix,rated,factor):
    """Uniform simultaneous acceleration bound at zero speed over given poses."""
    reserve=rated/factor-np.abs(gravity)
    if reserve.min()<0: return None
    coupling=np.abs(matrix).sum(axis=2)
    bounds=np.divide(reserve,coupling,out=np.full_like(reserve,np.inf),where=coupling>0)
    result=float(np.rad2deg(bounds.min()))
    return result if np.isfinite(result) else None


def main():
    options=read(ROOT/'arm-options-config.json');original=read(ROOT/'original-comparison-config.json')
    requirements=read(REF/'arm-design-requirements.json');assumed=requirements['screeningAssumptions']
    audit=read(ROOT/'build/module-audit/audit.json')
    assert audit['producerSha256']==digest(ROOT/'module-mass-audit.py'),'Stale module audit'
    for path,sha in audit['inputs'].items(): assert digest(Path(path))==sha,'Changed audit dependency: '+path
    geometry=read(ROOT/'build/engineering-input.json');layout=read(ROOT/'reference-layout.json')
    native=read(REF/'build/arm-design/zeroerr-original-native.json')
    engineering=load_module('options_engineering',ROOT/'engineering-study.py')
    bounds=load_module('options_bounds',ROOT/'module-mass-audit.py')
    kinematics=load_module('options_kinematics',REF/'arm-design-study.py')
    fk=kinematics.original_fk(geometry,layout)
    sources=read(ROOT/'build/revision-review/sources.json')+read(ROOT/'build/revision-review/170-source.json')
    for family,cat in options['catalog'].items():
        source=next(s for s in sources if s['key']==cat['drawingKey'])
        pdf=ROOT/'build/revision-review'/(cat['drawingKey']+'.pdf')
        assert digest(pdf)==cat['pdfSha256']==source['pdfSha256']
        text=subprocess.check_output(['pdftotext','-layout',str(pdf),'-'],text=True)
        for number in [cat['massKg'],cat['motorRotorInertiaGmm2']]+cat['ratedTorqueNm']+cat['peakSpeedRpm']:
            assert str(number) in text, 'Changed drawing value: '+str(number)
    clear=[o for o in native['observations'] if o['clear']]
    poses=[o['q'] for o in clear];path_groups=[]
    samples=options['assumptions']['pathInteriorSamplesPerSegment']
    for path in native['paths']:
        assert path['complete'] and path['continuous']['clear'], 'Missing original path clearance'
        start=len(poses)
        for a,b in zip(path['path'][:-1],path['path'][1:]):
            # Quintic joint interpolation visits the same configuration segment as linear u.
            poses.extend([(np.array(a)*(1-u)+np.array(b)*u).tolist() for u in np.linspace(0,1,samples,endpoint=False)])
        poses.append(path['path'][-1]);path_groups.append((path['id'],np.arange(start,len(poses))))
    poses=np.array(poses);domain=np.arange(len(clear))
    flange_reach=np.array([np.linalg.norm((fk(q)@kinematics.translated(-.15))[:2,3]) for q in poses])
    results=[]
    for variant in options['options']:
        cases=[];settings=None;estimated_mass=None
        for mass_case in options['assumptions']['massScenarios']:
            factor=assumed['massSensitivityFactor'] if mass_case=='drawing_plus25percent' else 1.
            for offset in options['assumptions']['cogOffsetsMm']:
                study,settings=build_study(engineering,geometry,original,options,variant,audit['materialAudit'],mass_case,offset,0)
                one,_=build_study(engineering,geometry,original,options,variant,audit['materialAudit'],mass_case,offset,1)
                g,m=map(np.array,zip(*(fast_terms(study,q,factor) for q in poses)))
                p,n=map(np.array,zip(*(fast_terms(one,q,factor) for q in poses)))
                pg,pm=p-g,n-m
                rated,speed=limits(settings,options)
                total=requirements['targets']['totalFlangeLoadKg']['value']
                gravity=g+total*pg;matrix=m+total*pm
                scenarios=[]
                for acceleration in options['assumptions']['accelerationScenariosDegS2']:
                    demand=(np.abs(gravity)+np.abs(matrix).sum(axis=2)*np.deg2rad(acceleration))*assumed['designFactor']
                    ratios=demand/rated;passing=np.max(ratios,axis=1)<=1
                    by_joint=ratios[domain].max(axis=0)
                    witness=np.unravel_index(ratios[domain].argmax(),ratios[domain].shape)
                    bounds_row=bounds.capacity(g[domain],m[domain],pg[domain],pm[domain],rated,acceleration,assumed['designFactor'],total)
                    reach=[]
                    for low,high in [(0,.35),(.35,.6),(.6,2.)]:
                        indices=domain[(flange_reach[domain]>=low)&(flange_reach[domain]<high)]
                        if len(indices):
                            reach.append(dict(horizontalFlangeReachM=[low,high],sampleCount=len(indices),
                                sampled5kgPassing=int(passing[indices].sum()),
                                **bounds.capacity(g[indices],m[indices],pg[indices],pm[indices],rated,acceleration,assumed['designFactor'],total)))
                    scenarios.append(dict(accelerationDegS2=acceleration,designFactor=assumed['designFactor'],
                        sampled5kgPassing=int(passing[domain].sum()),sampleCount=len(domain),
                        passingPoseIndices=domain[passing[domain]].tolist(),
                        maxDemandOverRatedByJoint=by_joint.tolist(),
                        witnessQ=poses[domain[witness[0]]].tolist(),witnessJoint='j'+str(witness[1]+1),
                        totalLoadBound=bounds_row,reachBins=reach,
                        paths=[dict(id=name,sampledLoadPass=bool(passing[indices].all()),sampleCount=len(indices),
                                    maxDemandOverRated=float(ratios[indices].max()),
                                    maxDemandOverRatedByJoint=ratios[indices].max(axis=0).tolist(),
                                    totalLoadBound=bounds.capacity(g[indices],m[indices],pg[indices],pm[indices],rated,
                                                                  acceleration,assumed['designFactor'],total),
                                    uniformZeroSpeedAccelerationLimitDegS2=acceleration_bound(gravity[indices],matrix[indices],rated,assumed['designFactor']))
                               for name,indices in path_groups]))
                cases.append(dict(massScenario=mass_case,combinedCogOffsetMm=offset,
                    estimatedArmMassKg=sum(b[1] for b in study.bodies if b[0]!='tool+payload')*factor,
                    nominalMaxGravityNm=np.abs(gravity[domain]).max(axis=0).tolist(),screens=scenarios))
        drawing_case=next(c for c in cases if c['massScenario']=='drawing_materials')
        rated,speed=limits(settings,options)
        results.append(dict(id=variant['id'],changes=variant['changes'],estimatedArmMassKg=drawing_case['estimatedArmMassKg'],
            ratedTorqueNm=rated.tolist(),publishedPeakSpeedDegS=speed.tolist(),
            eachJointPeakMeets180Benchmark=bool((speed>=requirements['targets']['peakJointSpeedDegS']['value']).all()),
            jointChoices=settings['jointCandidates'],cases=cases,
            geometryStatus='Original imported CAD' if not variant['changes'] else 'Original-axis mass/inertia sizing surrogate; resized CAD and self-clearance not established'))
        print('Evaluated',variant['id'],flush=True)
    baseline=engineering.Study(geometry,original)
    for q in [poses[0],poses[17]]:
        for factor in [1.,1.25]:
            g,m=fast_terms(baseline,q,factor);old=baseline.terms(q,factor)
            np.testing.assert_allclose(g,old[0],atol=1e-10)
            np.testing.assert_allclose(m,old[1],atol=1e-10)
    output=dict(status='sizing_options_compared_hardware_not_selected',scope=options['scope'],
        assumptions=options['assumptions'],sampledOriginalClearPoses=len(clear),
        poseCoordinatesRad=poses[:len(clear)].tolist(),
        options=results,catalog=options['catalog'],drawingSources=sources,
        hardwareQualified=False,resizedCadIntegrated=False,coupledTorqueSpeedValidated=False,cncTendingInScope=False,
        inputs=INPUTS.copy(),producerSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    BUILD.mkdir(parents=True,exist_ok=True)
    (BUILD/'comparison.json').write_text(json.dumps(output,indent=2)+'\n')
    print('Wrote',BUILD/'comparison.json')


if __name__=='__main__': main()
