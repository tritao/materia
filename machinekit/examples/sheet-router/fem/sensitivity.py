"""Single-pose parameter screens at fixed mounting interfaces, not replacement CAD."""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from study import solve


def part(packet,name):return next(p for p in packet['parts'] if p['id']==name)
def plate(packet,name):return next(p for p in packet['plates'] if p['id']==name)


def mass_change(packet,name,mass,dimensions=None,centre=None):
    p=part(packet,name);old=p['massKg'];ratio=mass/old
    p['massKg']=mass
    if centre:p['centreOfMassMm']=dict(zip(('x','y','z'),centre))
    if dimensions:
        x,y,z=dimensions
        p['centroidalInertiaKgMm2World']=dict(xx=mass*(y*y+z*z)/12,yy=mass*(x*x+z*z)/12,zz=mass*(x*x+y*y)/12,xy=0,xz=0,yz=0)
    else:p['centroidalInertiaKgMm2World']={k:v*ratio for k,v in p['centroidalInertiaKgMm2World'].items()}
    packet['totalMassKg']+=mass-old
    for axis in packet['axes']:
        for row in axis['movingMembers']:
            if row['id']==name:
                row['kg']+=mass-old;axis['movingMassKg']+=mass-old


def variant(base,pose,name):
    p=deepcopy(base);q=deepcopy(pose);note='Fixed CAD interfaces; analytical parameter screen.'
    if name=='welds x0.1':p['interfaces']['weldSpringFactor']=.1
    elif name=='X plate 25 mm':
        row=plate(p,'xCarriage');mass=part(p,'xCarriage')['massKg']*25/row['thicknessMm'];row['thicknessMm']=25
        origin=row['poseMm'];mass_change(p,'xCarriage',mass,(row['widthMm'],25,row['depthMm']),(origin['x'],origin['y']-12.5,origin['z']))
        p['guideLayout']['carriageThickness']=25
        note+=' Front datum retained; new joint spacers/stack and collision qualification required.'
    elif name=='Y blocks 400 mm':
        increase=400-p['yBlockSpacing'];p['yBlockSpacing']=400
        for side in ('Left','Right'):
            row=plate(p,'beamFoot'+side);old=row['depthMm'];row['depthMm']+=increase
            mass_change(p,'beamFoot'+side,part(p,'beamFoot'+side)['massKg']*row['depthMm']/old,(row['widthMm'],row['depthMm'],row['thicknessMm']))
            for block in q['blocks']:
                if block['axis']=='y' and side in block['id']:
                    delta=increase/2*(1 if block['id'].endswith('Rear') else -1)
                    for key in ('railPointMm','mountPointMm'):block[key][1]+=delta
                    component=part(p,block['id']);component['centreOfMassMm']['y']+=delta;component['poseMm']['y']+=delta
            # Preserve endpoint travel: 120 mm extra rail per side; treated as fixed mass.
            rail=part(p,'railY'+side);length=rail['boundsMm']['maxY']-rail['boundsMm']['minY']
            mass_change(p,rail['id'],rail['massKg']*(length+increase)/length,(23,length+increase,22))
        note+=' Feet lengthened 120 mm with exact rectangular mass/inertia; rail mass extended 120 mm per side. Block mass positions follow the new spacing.'
    elif name=='Y HGR30':
        p['interfaces']['yBlockRadialNPerMm']=618000
        for side in ('Left','Right'):
            rail=part(p,'railY'+side);length=(rail['boundsMm']['maxY']-rail['boundsMm']['minY'])/1000
            mass_change(p,rail['id'],rail['massKg']+(4.47-3.21)*length,(28,length*1000,26))
            for suffix in ('','Rear'):
                block=part(p,'blockY'+side+suffix);mass_change(p,block['id'],block['massKg']+(.88-.51),(60,97.4,19))
        note+=' HIWIN ZA radial 539 to 618 N/um; HGH25CA to HGH30CA increments 0.51 to 0.88 kg/block, rail 3.21 to 4.47 kg/m added to generic CAD masses. Lateral stiffness held at assumed 100 N/um, rail treated rigid; height datum held by hypothetical mounting adjustment.'
    elif name=='free levelling feet':p['floorSupport']='free'
    elif name=='steel Z ram':
        row=plate(p,'zCarriage');row['youngsMPa']=210000
        mass_change(p,'zCarriage',part(p,'zCarriage')['massKg']*7850/2700)
        note+=' Same geometry and inertia tensor scaled by steel/aluminium density; includes both side boxes.'
    elif name!='baseline':raise ValueError(name)
    return p,q,note


def first_gantry(row):return next(m for m in row['modalParticipation'] if m['gantryFraction']>=.5)


def signature(mode):
    result=[]
    for name in ('gantry','footLeft','footRight','x','z','spindle','frame'):
        body=mode['bodies'][name]
        result+=body['translationAtCentreNormalized']+[200*v for v in body['rotationRadPerMmMaxMotion']]
    return result


def mac(a,b):
    return sum(x*y for x,y in zip(a,b))**2/(sum(x*x for x in a)*sum(x*x for x in b))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--packets',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--ccx',type=Path,required=True);parser.add_argument('--criterion',choices=('gantry','static'),default='gantry');args=parser.parse_args()
    data=json.loads((args.packets/'results.json').read_text());fine=[r for r in data['results'] if r['meshMm']==50]
    worst=min(fine,key=lambda r:first_gantry(r)['frequencyHz']) if args.criterion=='gantry' else max(fine,key=lambda r:max(r['cases'][i]['complianceUmPerN'][i] for i in (0,1)))
    length=worst['workY'];base=json.loads((args.packets/f'y{length}'/'manifest.json').read_text());base['floorSupport']='anchored'
    names=['baseline','welds x0.1','X plate 25 mm','Y blocks 400 mm','Y HGR30','free levelling feet','steel Z ram']
    args.output.mkdir(parents=True,exist_ok=True);rows=[]
    baseline=None
    for i,name in enumerate(names):
        packet,pose,note=variant(base,worst['pose'],name)
        result=solve(packet,pose,50,args.ccx.resolve(),args.output/str(i))
        moving=next(a['movingMassKg'] for a in packet['axes'] if a['axis']=='y')
        mode=first_gantry(result)
        if baseline is None:baseline=result;ref=signature(mode);base_mass=moving
        tracked=max(result['modalParticipation'],key=lambda m:mac(ref,signature(m)))
        row=dict(name=name,note=note,movingYMassKg=moving,deltaMovingMassKg=moving-base_mass,
            xyStiffnessNPerUm=[1/result['cases'][j]['complianceUmPerN'][j] for j in (0,1)],
            firstGantryHz=mode['frequencyHz'],firstGantryMode=mode['mode'],firstGlobalHz=result['firstModesHz'][0],
            trackedBaselineGantryHz=tracked['frequencyHz'],bodyMotionSignatureMAC=mac(ref,signature(tracked)),result=result)
        row['deltaXYStiffnessNPerUm']=[v-1/baseline['cases'][j]['complianceUmPerN'][j] for j,v in enumerate(row['xyStiffnessNPerUm'])]
        row['deltaFirstGantryHz']=mode['frequencyHz']-first_gantry(baseline)['frequencyHz']
        row['deltaTrackedGantryHz']=tracked['frequencyHz']-first_gantry(baseline)['frequencyHz']
        rows.append(row);print(name,row['xyStiffnessNPerUm'],row['firstGantryHz'],row['deltaMovingMassKg'],flush=True)
        identity={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(__file__).with_name('study.py'),Path(__file__).with_name('modes.py'),Path(__file__).with_name('front_guides.py')]}
        (args.output/'sensitivities.json').write_text(json.dumps(dict(workY=length,pose=worst['pose'],criterion=('Lowest gantry-dominant mode across both beds/all fine-mesh poses; governing target failure' if args.criterion=='gantry' else 'Worst directional X/Y cutting compliance across both beds/all fine-mesh poses'),sourceSha256=identity,rows=rows),indent=2)+'\n')


if __name__=='__main__':main()
