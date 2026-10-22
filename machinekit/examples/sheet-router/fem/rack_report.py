"""Condense retained rack-joint solver and motion evidence without re-solving."""
import argparse,json
from pathlib import Path

def read(path):
    return json.loads(path.read_text())

def mode_row(m):
    return {key:m[key] for key in ('frequencyHz','strainEnergyShare','gantryKineticFraction',
        'accelerationResidueXY','gammaXY','toolRelativeEigenvectorXY','bodies')}

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    beds=[]
    for bed in (2600,3150):
        meshes=[]
        for mesh in (100,50):
            base=a.root/f'macro-h{mesh}'/f'y{bed}'/'baseline'
            cases=[read(f) for f in sorted(base.glob('*/summary.json'))]
            assert len(cases)==9
            minima=[]
            for axis in (0,1):
                d=min(cases,key=lambda c:c['xyStiffnessNPerUm'][axis])
                minima.append(dict(axis='XY'[axis],value=d['xyStiffnessNPerUm'][axis],pose=d['pose']))
            low=min(cases,key=lambda c:c['modes'][0]['frequencyHz'])
            gantry=min(((m,c) for c in cases for m in c['modes'] if m['gantryKineticFraction']>=.5),key=lambda pair:pair[0]['frequencyHz'])
            packet=read(base/'packet.json')
            meshes.append(dict(meshMm=mesh,minStiffnessNPerUm=minima,lowestModePose=low['pose'],lowestSix=[mode_row(m) for m in low['modes'][:6]],
                lowestGantry=dict(pose=gantry[1]['pose'],mode=mode_row(gantry[0])),rackSeatScreen=packet['rackSeatScreen'],
                movingYMassKg=next(r['movingMassKg'] for r in packet['axes'] if r['axis']=='y')))
        beds.append(dict(bedMm=bed,meshes=meshes))
    base=read(a.root/'macro-h50/y2600/baseline/x800-y1300-h50/summary.json');base_f=next(m['frequencyHz'] for m in base['modes'] if m['gantryKineticFraction']>=.5)
    options=[]
    for option in ('gearbox2','mount2','seat4','module2','seat4both','balanced'):
        d=read(a.root/f'drive-{option}/y2600/baseline/x800-y1300-h50/summary.json')
        f=next(m['frequencyHz'] for m in d['modes'] if m['gantryKineticFraction']>=.5)
        policy=read(a.root/f'drive-{option}/policy.json')['policies'][0]
        options.append(dict(option=option,poseMm=d['pose'],stiffnessNPerUm=d['xyStiffnessNPerUm'],deltaStiffnessNPerUm=[x-y for x,y in zip(d['xyStiffnessNPerUm'],base['xyStiffnessNPerUm'])],
            firstGantryHz=f,deltaFirstGantryHz=f-base_f,worstMotion=policy['worst'],scope=policy['scope'],mass='held fixed optimistic bound, except module2 fixed Y rack mass'))
    policies={name:read(a.root/name)['policies'] for name in ('policy-h100.json','policy-h50.json','policy-native.json')}
    result=dict(status='Draft rejected: static and nominal dynamic failures; no frozen CAD edit or J5 acceptance',verification=read(a.root/'verification.json'),beds=beds,driveOptions=options,policies=policies)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
if __name__=='__main__':main()
