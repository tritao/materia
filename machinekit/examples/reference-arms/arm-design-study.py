#!/usr/bin/env python3
"""Prepare and review standalone arm evidence; no cell-registration inputs."""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from zipfile import ZipFile
import numpy as np

ROOT = Path(__file__).resolve().parent
ARM = ROOT.parent/'zeroerr-arm'
BUILD = ROOT/'build/arm-design'
IDS = ['ur5e','fairino5_v6','zeroerr-original']
INPUTS = {}


def digest(path):
    value = hashlib.sha256(path.read_bytes()).hexdigest()
    INPUTS[str(path.resolve())] = value
    return value


def read(path):
    digest(path)
    return json.loads(path.read_text())


def require(ok, message):
    if not ok:
        raise ValueError(message)


def rotation(axis, q):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x,y,z = axis
    k = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(q)*k+(1-math.cos(q))*(k@k)


def frame(raw):
    q = np.array([raw[k] for k in ['qx','qy','qz']], dtype=float)
    w = raw['qw']
    k = np.array([[0,-q[2],q[1]],[q[2],0,-q[0]],[-q[1],q[0],0]])
    m = np.eye(4)
    m[:3,:3] += 2*w*k+2*k@k
    m[:3,3] = [raw[k] for k in 'xyz']
    return m


def translated(z):
    m = np.eye(4);m[2,3]=z;return m


def urdf_fk(path, tip, offset):
    tree = ET.parse(path).getroot()
    incoming = {j.find('child').get('link'):j for j in tree.findall('joint')}
    chain = []
    while tip in incoming:
        joint = incoming[tip];chain.insert(0,joint);tip=joint.find('parent').get('link')
    def evaluate(q):
        m,index = np.eye(4),0
        for j in chain:
            origin = j.find('origin')
            xyz = [float(v) for v in (origin.get('xyz','0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy','0 0 0') if origin is not None else '0 0 0').split()]
            t = np.eye(4);t[:3,3]=xyz
            t[:3,:3]=rotation([0,0,1],rpy[2])@rotation([0,1,0],rpy[1])@rotation([1,0,0],rpy[0])
            m=m@t
            if j.get('type') != 'fixed':
                require(j.get('type') in ['revolute','continuous'], 'Unsupported reference joint')
                a=j.find('axis');axis=[float(v) for v in (a.get('xyz') if a is not None else '1 0 0').split()]
                r=np.eye(4);r[:3,:3]=rotation(axis,q[index]);m=m@r;index+=1
        return m@offset@translated(.15)
    return evaluate


def dh_fk(q, oracle):
    m=np.eye(4);m[:3,:3]=rotation([0,0,1],oracle['baseRotationZRad'])
    for t,a,d,alpha in zip(q,oracle['aMetres'],oracle['dMetres'],oracle['alphaOverPi']):
        ct,st,ca,sa=math.cos(t),math.sin(t),math.cos(alpha*math.pi),math.sin(alpha*math.pi)
        m=m@np.array([[ct,-st*ca,st*sa,a*ct],[st,ct*ca,-ct*sa,a*st],[0,sa,ca,d],[0,0,0,1]])
    return m@translated(.15)


def original_fk(geometry, layout):
    occurrence=next(o for o in layout['occurrences'] if o['id']=='toolFlange')
    zero=np.eye(4);zero[:3,:]=np.array(occurrence['matrix']).reshape(3,4);zero[:3,3]/=1000
    connector=np.eye(4);connector[:3,:3]=rotation([1,0,0],math.pi/2)
    zero=zero@connector@translated(.15)
    def evaluate(q):
        m=np.eye(4)
        for angle,axis in zip(q,geometry['axes']):
            o=np.array(axis['originMm'])/1000;r=rotation(axis['axis'],angle)
            t=np.eye(4);t[:3,:3]=r;t[:3,3]=o-r@o;m=m@t
        return m@zero
    return evaluate


def halton(n, base):
    value,factor=0.,1.
    while n:
        n,d=divmod(n,base);factor/=base;value+=d*factor
    return value


def make_pose(x,y,z,yaw=0):
    return dict(x=x,y=y,z=z,qx=math.cos(yaw/2),qy=math.sin(yaw/2),qz=0.,qw=0.)


def prepare():
    requirements=read(ROOT/'arm-design-requirements.json')
    oracles=read(ROOT/'arm-reference-oracles.json')
    original=read(ARM/'original-comparison-config.json')
    assumed=requirements['screeningAssumptions']
    count=assumed['haltonSamples']
    require(isinstance(count,int) and count>0,'Halton sample count must be positive')
    tool=assumed['commonTool']
    require((tool['lengthMm'],tool['widthMm'],tool['massKg'],assumed['toolKg'])==(150,40,1,1),'Changed tool requires matching geometry and mass model')
    require(math.isfinite(assumed['collisionMarginMetres']) and assumed['collisionMarginMetres']>0,'Invalid collision margin')
    require(assumed['payloadKg']+assumed['toolKg']==requirements['targets']['totalFlangeLoadKg']['value'],'Flange-load requirement and screen differ')
    models=[]
    for id in IDS:
        if id=='zeroerr-original':
            limits=[np.deg2rad(requirements['screeningAssumptions']['originalJointWindowDeg']).tolist() for _ in range(6)]
            source=ARM/'build/zeroerr5.mtrg'
            basis='Provisional +/-170 degree diagnostic range, not physical limits'
        else:
            manifest=read(ROOT/'build'/id/'manifest.json')
            joints=[j for j in manifest['joints'] if j['type']!='fixed']
            limits=[[j['limits']['lower'],j['limits']['upper']] for j in joints]
            source=ROOT/'build'/id/(id+'.mtrg');basis='Pinned official ROS description; discrepancies recorded separately'
        probes=[[0.]*6]
        probes += [[math.radians(sign*30) if j==axis else 0. for j in range(6)] for axis in range(6) for sign in [-1,1]]
        probes += [[lo+(hi-lo)*halton(n,b) for (lo,hi),b in zip(limits,[2,3,5,7,11,13])] for n in range(1,count+1)]
        models.append(dict(id=id,limits=limits,limitBasis=basis,probes=probes,sourceFingerprint=digest(source)))
    targets=[]
    for radius in [.35,.55,.75]:
        for azimuth in [-90,0,90,180]:
            for z in [.3,.6]:
                for yaw in [0,math.pi/2]:
                    targets.append(dict(id=f'r{radius}-a{azimuth}-z{z}-yaw{yaw}',pose=make_pose(radius*math.cos(math.radians(azimuth)),radius*math.sin(math.radians(azimuth)),z,yaw)))
    paths=[]
    for name in ['line','lift','square','yaw']:
        poses=[]
        if name=='square':
            corners=[(0,0),(.04,0),(.04,.04),(0,.04),(0,0)]
            for a,b in zip(corners,corners[1:]):
                for n in range(5):
                    t=n/5;poses.append(make_pose(.45+a[0]*(1-t)+b[0]*t,.15+a[1]*(1-t)+b[1]*t,.45))
            poses.append(make_pose(.45,.15,.45))
        else:
            for n in range(21):
                t=n/20
                poses.append(make_pose(.45+(.04*t if name=='line' else 0),.15,.45+(.04*t if name=='lift' else 0),math.radians(20)*t if name=='yaw' else 0))
        paths.append(dict(id=name,poses=poses))
    BUILD.mkdir(parents=True,exist_ok=True)
    output=dict(scope=requirements['scope'],collisionMarginMetres=assumed['collisionMarginMetres'],models=models,targets=targets,paths=paths,inputs=INPUTS.copy())
    (BUILD/'inputs.json').write_text(json.dumps(output,indent=2)+'\n')
    print('Prepared standalone inputs:',len(models),'arms,',len(targets),'oriented targets and',len(paths),'basic paths')


def numeric_jacobian(fk,q):
    reference=fk(q);columns=[]
    for j in range(6):
        a=np.array(q,dtype=float);b=a.copy();a[j]+=1e-6;b[j]-=1e-6
        plus,minus=fk(a),fk(b);angular=(plus[:3,:3]-minus[:3,:3])/(2e-6)@reference[:3,:3].T
        columns.append(np.r_[(plus[:3,3]-minus[:3,3])/(2e-6),[angular[2,1],angular[0,2],angular[1,0]]])
    return np.array(columns).T


def loads(geometry, config, observations):
    spec=importlib.util.spec_from_file_location('engineering',ARM/'engineering-study.py')
    engineering=importlib.util.module_from_spec(spec);spec.loader.exec_module(engineering)
    digest(ARM/'engineering-study.py')
    requirements=read(ROOT/'arm-design-requirements.json');assumed=requirements['screeningAssumptions']
    poses=[np.array(o['q']) for o in observations if o['clear']]
    require(bool(poses),'No load poses')
    reports=[]
    for offset in assumed['combinedCogOffsetsMm']:
        settings=copy.deepcopy(config)
        settings.pop('missionStudy',None)
        settings['application'].update({k:assumed[k] for k in ['toolKg','payloadKg','jointAccelerationDegS2','designFactor','massSensitivityFactor']})
        settings['application']['combinedCogAlongToolNormalMm']=offset
        study=engineering.Study(geometry,settings)
        probe=np.array([.17,-.63,.48,.29,-.37,.23]);g,m,_,_=study.terms(probe)
        gradient=[]
        for j in range(6):
            dq=np.eye(6)[j]*1e-6;gradient.append((study.terms(probe+dq)[3]-study.terms(probe-dq)[3])/2e-6)
        np.testing.assert_allclose(g,gradient,atol=1e-6,rtol=1e-6)
        np.testing.assert_allclose(m,m.T,atol=1e-10)
        require(np.linalg.eigvalsh(m).min()>=-1e-10,'Non-PSD mass matrix')
        scenarios=[]
        rated=np.array([settings['catalog'][j['family']]['ratedTorqueNm'][settings['catalog'][j['family']]['ratios'].index(j['nominalRatio'])] for j in settings['jointCandidates']])
        for factor in [1.,assumed['massSensitivityFactor']]:
            gravity,envelope=np.zeros(6),np.zeros(6)
            for q in poses:
                g,m,_,_=study.terms(q,factor)
                gravity=np.maximum(gravity,np.abs(g));envelope=np.maximum(envelope,(np.abs(g)+np.abs(m).sum(axis=1)*math.radians(assumed['jointAccelerationDegS2']))*assumed['designFactor'])
            scenarios.append(dict(armMassFactor=factor,gravityNm=gravity.tolist(),zeroSpeedAccelerationEnvelopeNm=envelope.tolist(),ratioToAssumedRated=(envelope/rated).tolist()))
        reports.append(dict(combinedCogOffsetMm=offset,totalFlangeLoadKg=assumed['toolKg']+assumed['payloadKg'],ratedTorqueNm=rated.tolist(),scenarios=scenarios))
    return dict(scope='Same sampled original-arm configurations clear with the synthetic tool box; combined load treated as point mass. Nominal-100 and module mass split assumed. No velocities, friction, thermal duty, flexible links or measured accuracy.',evaluatedPoses=len(poses),estimatedArmMassKg=sum(b[1] for b in study.bodies if b[0]!='tool+payload'),studies=reports)


def review():
    data=read(BUILD/'inputs.json');requirements=read(ROOT/'arm-design-requirements.json');oracles=read(ROOT/'arm-reference-oracles.json')
    for p,h in data['inputs'].items():require(digest(Path(p))==h,'Changed standalone preparation input: '+p)
    layout=read(ARM/'reference-layout.json');geometry=read(ARM/'build/engineering-input.json');config=read(ARM/'original-comparison-config.json')
    require(geometry['layoutSha256']==digest(ARM/'reference-layout.json'),'Stale original engineering input')
    source_lock=read(ROOT/'source-lock.json');results=[];native_rows={}
    for model in data['models']:
        id=model['id'];native=read(BUILD/(id+'-native.json'));native_rows[id]=native
        require(native['inputSha256']==digest(BUILD/'inputs.json'),'Changed standalone inputs')
        require(native['collisionMarginMetres']==requirements['screeningAssumptions']['collisionMarginMetres'],'Changed collision margin')
        require(native['producerSha256']==digest(ROOT/'ArmDesignReview.hx'),'Stale native diagnostics')
        require(native['moduleSha256']==digest(ROOT/'build/host/main.hl'),'Changed native review bytecode')
        for dep in native['dependencies']:require(dep['sha256']==digest(ROOT/dep['name']),'Changed native review dependency')
        require(native['sourceFingerprint']==model['sourceFingerprint'],'Changed standalone arm')
        if id=='zeroerr-original':
            fk=original_fk(geometry,layout);oracle_status='STEP_derived_transform_consistency_only'
        else:
            directory=ROOT/'build'/id;manifest=read(directory/'manifest.json')
            require(manifest['sourceFiles']==source_lock['models'][id]['files'],'Unpinned reference sources')
            require(manifest['urdfSha256']==digest(directory/'robot.urdf'),'Changed reference URDF')
            for link in manifest['links']:
                for mesh in link['visuals']+link['collisions']:
                    for ext,h in mesh['sha256'].items():
                        if h is not None:require(digest(directory/(mesh['stem']+'.'+ext))==h,'Changed reference mesh')
            fk=urdf_fk(directory/'robot.urdf',manifest['tip'],frame(manifest['flangeOffset']) if manifest.get('flangeOffset') else np.eye(4))
            oracle_status='manufacturer_nominal_DH_checked' if id=='ur5e' else 'drawing_dimensions_and_URDF_consistency_full_vendor_FK_pending'
        max_pos=max_rot=max_jac=max_dh_pos=max_dh_rot=0.;singular=0;sigmas=[];positions=[];ik_checks=0
        require(len(native['observations'])==len(model['probes'])==13+requirements['screeningAssumptions']['haltonSamples'],'Incomplete joint sampling')
        for q,obs in zip(model['probes'],native['observations']):
            require(q==obs['q'],'Changed joint probe')
            actual=frame(obs['tcp']);expected=fk(q)
            max_pos=max(max_pos,float(np.linalg.norm(actual[:3,3]-expected[:3,3])))
            max_rot=max(max_rot,float(np.max(np.abs(actual[:3,:3]-expected[:3,:3]))))
            require(max_pos<1e-8 and max_rot<1e-8,'Independent source-transform FK mismatch: '+id)
            jac=np.array(obs['jacobian']).reshape(6,6)
            require(np.isfinite(jac).all(),'Nonfinite native Jacobian')
            max_jac=max(max_jac,float(np.max(np.abs(jac-numeric_jacobian(fk,q)))))
            require(max_jac<1e-5,'Finite-difference Jacobian mismatch: '+id)
            scaled=jac.copy();scaled[:3]/=requirements['screeningAssumptions']['jacobianCharacteristicLengthM']
            sigma=float(np.linalg.svd(scaled,compute_uv=False)[-1]);sigmas.append(sigma)
            if sigma<requirements['screeningAssumptions']['nearSingularSigmaThreshold']:singular+=1
            positions.append(dict(position=actual[:3,3].tolist(),clear=obs['clear'],sigmaMin=sigma))
            if obs['ik'] is not None:
                require(obs['clear'] and obs['ik']['positionError']<=1e-6 and obs['ik']['angleError']<=1e-6,'Failed reachable IK regression');ik_checks+=1
            if id=='ur5e':
                golden=dh_fk(q,oracles[id]);max_dh_pos=max(max_dh_pos,float(np.linalg.norm(actual[:3,3]-golden[:3,3])));max_dh_rot=max(max_dh_rot,float(np.max(np.abs(actual[:3,:3]-golden[:3,:3]))))
        require(ik_checks>0,'No reachable IK checks')
        if id=='ur5e':require(max_dh_pos<=oracles[id]['translationToleranceM'] and max_dh_rot<=oracles[id]['rotationMatrixTolerance'],'Manufacturer DH mismatch')
        dimensions=None
        if id=='fairino5_v6':
            oracle=oracles[id];archive=ROOT/oracle['archive'];require(digest(archive)==oracle['archiveSha256'],'Changed manufacturer drawing archive')
            with ZipFile(archive) as z:require(hashlib.sha256(z.read(oracle['drawing'])).hexdigest()==oracle['drawingSha256'],'Changed manufacturer drawing')
            j=[j for j in manifest['joints'] if j['type']!='fixed'];values=[j[1]['origin']['z']*1000,abs(j[2]['origin']['x'])*1000,abs(j[3]['origin']['x'])*1000,j[4]['origin']['z']*1000,j[5]['origin']['z']*1000,manifest['flangeOffset']['z']*1000]
            dimensions=[dict(name=name,drawingMm=value,descriptionMm=found,differenceMm=found-value) for (name,value),found in zip(oracle['dimensionsMm'].items(),values)]
            require(all(abs(d['differenceMm'])<=oracle['dimensionToleranceMm'] for d in dimensions),'Manufacturer dimensional mismatch')
        require(len(native['targets'])==len(data['targets'])==48 and len(native['paths'])==4,'Incomplete standalone tasks')
        for path in native['paths']:
            if path['continuous']['clear']:
                require(path['complete'] and len(path['path'])==21 and path['continuous']['error'] is None and path['continuous']['queries']>0,'Invalid continuous path evidence')
        if id in ['ur5e','fairino5_v6']:
            require(all(p['continuous']['clear'] for p in native['paths']),'Commercial nominal-model basic-motion regression')
        result=dict(model=id,referenceValidation=oracle_status,observedConfigurations=len(native['observations']),hullClearConfigurations=sum(o['clear'] for o in native['observations']),
                    maxSourceFkErrorM=max_pos,maxSourceRotationMatrixError=max_rot,maxJacobianDifference=max_jac,reachableIkChecks=ik_checks,
                    manufacturerDhErrorM=max_dh_pos if id=='ur5e' else None,manufacturerDhRotationMatrixError=max_dh_rot if id=='ur5e' else None,
                    drawingDimensions=dimensions,clearOrientedTargets=sum(t['clear'] for t in native['targets']),orientedTargets=48,
                    nearSingularSamples=singular,minimumScaledJacobianSigma=min(sigmas),periodicityJoints=len(native['periodicity']),
                    paths=[dict(id=p['id'],clear=p['continuous']['clear'],maxJointStep=p['maxJointStep'],error=p['continuous'].get('error'),failure=p['failure']) for p in native['paths']],positions=positions,
                    limits=model['limits'],limitBasis=model['limitBasis'])
        results.append(result)
    load=loads(geometry,config,native_rows['zeroerr-original']['observations'])
    report=dict(status='standalone_arm_baseline_reviewed_with_explicit_evidence_gaps',models=results,originalLoads=load,
                hardwareQualified=False,independentControllerValidationComplete=False,cncTendingInScope=False,
                evidenceGaps=['FR5 independent full-controller FK oracle absent','UR5e nominal DH is not serial-number calibration','Manufacturer joint-travel vs source-description differences unresolved','Self-clearance uses enclosing meshes and standard adjacent-body exclusions','Sampled workspace and singularities are not complete workspace or path guarantees','Original actuator ratios, masses/inertias, stiffness, torque-speed/thermal duty and harness travel unconfirmed'],
                producerSha256=digest(Path(__file__)),inputs=INPUTS)
    (BUILD/'review.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['inputs','models','originalLoads']},indent=2))
    for row in results:print(row['model'],row['referenceValidation'],row['clearOrientedTargets'],'/48 target solutions',sum(p['clear']for p in row['paths']),'/4 basic paths')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--prepare',action='store_true');args=parser.parse_args()
    prepare() if args.prepare else review()


if __name__=='__main__':main()
