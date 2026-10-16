#!/usr/bin/env python3
"""Hand/static reconciliation and explicitly conditional operating-envelope samples."""
import hashlib
import csv
import json
from pathlib import Path
import numpy as np
from reference_dynamics import reference_study,terms,transformed_bodies,load_json,ROOT


def main():
    frozen=load_json(ROOT/'reference-baseline.json');study=reference_study()
    # Both long links horizontal: rotate original upright chain about J2 by -90 degrees.
    q=np.array([0,-np.pi/2,0,0,0,0]);g,_=terms(study,q)
    rows=[]
    for name,mass,p,active in transformed_bodies(study,q):
        if 1 not in active: continue
        lever=float(p[1]-study.origins[1,1])
        rows.append(dict(body=name,massKg=mass,signedShoulderLeverM=lever,
                         handTorqueNm=mass*9.80665*lever))
    hand=sum(r['handTorqueNm'] for r in rows)
    np.testing.assert_allclose(hand,g[1],atol=1e-9)
    # The old 119 Nm is a different sampled witness, not this stretched pose.
    old=load_json(ROOT.parent/'reference-arms/build/arm-design/review.json')
    native=load_json(ROOT.parent/'reference-arms/build/arm-design/zeroerr-original-native.json')
    old_engineering=__import__('reference_dynamics').engineering_module()
    config=load_json(ROOT/'original-comparison-config.json');old_study=old_engineering.Study(load_json(ROOT/'build/engineering-input.json'),config)
    poses=np.array([o['q'] for o in native['observations'] if o['clear']])
    zero_load=reference_study(0,0)
    flange_positions=np.array([next(b[2] for b in transformed_bodies(zero_load,p) if b[0]=='tool+payload') for p in poses])
    reaches=np.linalg.norm(flange_positions[:,:2],axis=1)
    previous=[abs(old_study.terms(p)[0][1]) for p in poses];witness=poses[int(np.argmax(previous))]
    peak=np.array([frozen['ratingsAtAssumedNominal100'][f]['repeatedStartStopPeakNm'] for f in frozen['modules']])
    brake=np.array([frozen['ratingsAtAssumedNominal100'][f]['brakeStaticAcceptanceNm'] for f in frozen['modules']])
    envelope=[];csv_rows=[]
    for offset in [0,100,150]:
        for load in [0,1,2,3,4,5]:
            model=reference_study(load,offset)
            gravity=np.array([terms(model,p)[0] for p in poses]);passing=(np.abs(gravity)<=np.minimum(peak,brake)).all(axis=1)
            bins=[]
            for low,high in [(0,.35),(.35,.6),(.6,2.)]:
                indices=np.flatnonzero((reaches>=low)&(reaches<high))
                if len(indices):bins.append(dict(horizontalFlangeReachM=[low,high],sampleCount=len(indices),
                    passingCount=int(passing[indices].sum()),observedReachMinM=float(reaches[indices].min()),observedReachMaxM=float(reaches[indices].max())))
            envelope.append(dict(totalFlangeLoadKg=load,combinedCogMm=offset,
                staticBrakedAndRepeatedGearPassCount=int(passing.sum()),sampleCount=len(poses),
                passingPoseIndices=np.flatnonzero(passing).tolist(),maxGravityNm=np.abs(gravity).max(axis=0).tolist(),reachBins=bins))
            for i in range(len(poses)):
                csv_rows.append([i,load,offset,float(reaches[i]),*flange_positions[i].tolist(),bool(passing[i]),*gravity[i].tolist(),*poses[i].tolist()])
    output=dict(status='trusted_vendor_baseline_reconciled_rating_and_mass_model_assumptions_identified',
        frozenGeometry=True,handPoseRad=q.tolist(),handShoulderTorqueNm=hand,handRows=rows,
        solverShoulderTorqueNm=float(g[1]),old119WitnessRad=witness.tolist(),
        oldWitnessShellDistributedNm=float(max(previous)),wholeLumpAtSameWitnessNm=float(terms(study,witness)[0][1]),
        wholeModuleArmMassKg=sum(b[1] for b in study.bodies if b[0]!='tool+payload'),
        brakedRestLimitsNm=brake.tolist(),repeatedGearLimitsNm=peak.tolist(),envelope=envelope,
        poseFlangeCoordinatesM=flange_positions.tolist(),poseJointCoordinatesRad=poses.tolist(),
        identifiedFailedAssumptions=['Shell-volume allocation is not module physical mass distribution',
            '87 Nm rated gearbox torque at specified input speed was treated as universal static holding capability',
            'Zero-speed simultaneous acceleration/design-factor screens were treated as a timed duty-cycle qualification',
            '119 Nm sampled worst-case witness was conflated with a particular stretched-out pose'],
        qualification='Static sampled envelope conditional on nominal-100, manual frame registration and housing lump. Vendor rating is trusted. Motion/duty and commissioned limits are separate; no full vendor envelope is invented.',
        geometryChanges=False,hardwareQualified=False,
        inputs={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                [ROOT/'reference-baseline.json',ROOT/'reference_dynamics.py',ROOT/'reference-layout.json',ROOT/'build/engineering-input.json',ROOT/'reconcile-reference.py']})
    build=ROOT/'build/reference-reconciliation';build.mkdir(parents=True,exist_ok=True)
    (build/'reconciliation.json').write_text(json.dumps(output,indent=2)+'\n')
    with (build/'static-operating-envelope.csv').open('w') as handle:
        writer=csv.writer(handle);writer.writerow(['poseIndex','totalFlangeLoadKg','combinedCogMm','horizontalFlangeReachM','flangeX_M','flangeY_M','flangeZ_M','brakedStaticAndRepeatedGearPass']+
            ['gravityJ'+str(i+1)+'_Nm' for i in range(6)]+['q'+str(i+1)+'_rad' for i in range(6)])
        writer.writerows(csv_rows)
    print('Hand shoulder Nm:',hand,'old witness Nm:',max(previous),'lumped same witness Nm:',output['wholeLumpAtSameWitnessNm'])
    print('Static 5 kg passes:',[(e['combinedCogMm'],e['staticBrakedAndRepeatedGearPassCount'],e['sampleCount']) for e in envelope if e['totalFlangeLoadKg']==5])


if __name__=='__main__':main()
