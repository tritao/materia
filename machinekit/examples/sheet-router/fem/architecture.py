"""One governing-pose architecture screen. Gross member masses, no CAD release."""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from sensitivity import part,plate,mass_change,first_gantry,signature,mac
from study import solve,project


def gross(row):
    w,h,t=row['widthMm'],row['heightMm'],row['wallMm']
    return (w*h-(w-2*t)*(h-2*t))*math.dist(row['startMm'],row['endMm'])*row['densityKgPerMm3']


def add_part(packet,row,moving=()):
    packet['parts'].append(row);packet['totalMassKg']+=row['massKg']
    for axis in packet['axes']:
        if axis['axis'] in moving:axis['movingMassKg']+=row['massKg'];axis['movingMembers'].append(dict(id=row['id'],kg=row['massKg']))


def turn_top(row,old_y,old_z,new_y,new_z):
    for key in ('centreOfMassMm','poseMm'):
        p=row[key];y,z=p['y']-old_y,p['z']-old_z;p['y']=new_y+z;p['z']=new_z-y
    t=row['centroidalInertiaKgMm2World'];row['centroidalInertiaKgMm2World']=dict(xx=t['xx'],yy=t['zz'],zz=t['yy'],xy=t['xz'],xz=-t['xy'],yz=-t['yz'])


def variant(base,pose,name):
    p=deepcopy(base);q=deepcopy(pose);notes=[]
    if name=='deep side girders':
        rows=[r for r in p['beams'] if r['id'].startswith('base/')];old_gross=sum(gross(r) for r in rows);factor=part(p,'base')['massKg']/old_gross
        links=[]
        for row in rows:
            if row['id'] not in ('base/tableSide0','base/tableSide1'):continue
            old_a=list(row['startMm']);old_b=list(row['endMm']);shift=(300-row['heightMm'])/2
            row['heightMm']=300;row['widthMm']=200;row['wallMm']=6
            row['startMm'][2]-=shift;row['endMm'][2]-=shift
            for other in rows:
                if other is row:continue
                for endpoint in (other['startMm'],other['endMm']):
                    if math.dist(endpoint,project(endpoint,old_a,old_b))<1e-5:links.append([list(endpoint),list(project(endpoint,row['startMm'],row['endMm']))])
        p['analysisFrameLinksMm']=links
        mass_change(p,'base',sum(gross(r) for r in rows)*factor)
        notes+=['Replace both under-table side stringers with closed RHS 300 vertical x200x6; keep top datum fixed. Existing crossmembers/braces retained. Rigid section offsets connect real intersecting members. Gross-stock mass uses baseline CAD-union/gross ratio; local weld/wall effects excluded.']
    elif name=='additional leg pair':
        rows=[r for r in p['beams'] if r['id'].startswith('base/')];factor=part(p,'base')['massKg']/sum(gross(r) for r in rows);added=0.
        for side in (0,1):
            ref=next(r for r in rows if r['id']=='base/post'+str(side)+'1');row=deepcopy(ref);row['id']='base/extraPost'+str(side)
            for key in ('startMm','endMm'):row[key][1]=q['y']
            p['beams'].append(row);added+=gross(row)*factor
            foot=deepcopy(p['supports'][side*3+1]);foot['id']='additionalFoot'+str(side);foot['pointMm'][1]=q['y'];p['supports'].append(foot)
            for id in ('foot'+str(side)+'1','footCap'+str(side)+'1'):
                component=deepcopy(part(p,id));component['id']='additional'+id
                delta=q['y']-component['poseMm']['y']
                component['poseMm']['y']+=delta;component['centreOfMassMm']['y']+=delta
                add_part(p,component)
        mass_change(p,'base',part(p,'base')['massKg']+added)
        notes+=['Baseline already has a middle pair: six feet at three stations. Add a fourth pair directly beneath Y=1575 rather than counting the existing middle pair twice. Added posts/feet share the anchored 50,000 N/mm XYZ support assumption.']
    elif name=='beam down 10 mm':
        for row in p['beams']:
            if row['id']=='beam':row['startMm'][2]-=10;row['endMm'][2]-=10
        for side in ('Left','Right'):
            row=plate(p,'beamFoot'+side);row['thicknessMm']=20;component=part(p,row['id']);origin=row['poseMm']
            mass_change(p,row['id'],component['massKg']*2/3,(row['widthMm'],row['depthMm'],20),(origin['x'],origin['y'],origin['z']+10))
        notes+=['20 mm feet instead of 30 lower the beam 10 mm. X rails and Z ram remain at their original absolute heights: ram-to-sheet gap stays149 mm. Upper front X rail has only5 mm nominal edge margin; 60 mm beam lowering is geometrically infeasible with this rail seating and clearance. Shorter feet are not ideal rigid links.']
    elif name=='top/front X rails':
        beam=next(r for r in p['beams'] if r['id']=='beam');centre_y=beam['startMm'][1];centre_z=beam['startMm'][2]
        upper=[b for b in q['blocks'] if b['id'] in ('blockX','blockX01')];old_y,old_z=upper[0]['railPointMm'][1:];new_y=centre_y+25;new_z=centre_z+100+17.5
        p['analysisTopXBridge']=dict(midZMm=new_z+12.5+10,thicknessMm=20)
        for b in upper:
            b['railPointMm'][1:]=[new_y,new_z];b['mountPointMm'][1:]=[new_y,new_z+12.5];b['topMounted']=True;b['translationNPerMmXYZ']=[0,b['lateralNPerMm'],b['radialNPerMm']]
            turn_top(part(p,b['id']),old_y,old_z,new_y,new_z)
        turn_top(part(p,'railX'),old_y,old_z,new_y,new_z)
        xp=plate(p,'xCarriage');width=xp['widthMm'];depth=new_y-(xp['poseMm']['y']-xp['thicknessMm']/2);thickness=20;mass=width*depth*thickness*2700e-9
        row=deepcopy(part(p,'xCarriage'));row['id']='analysisTopBridge';row['massKg']=mass
        row['centreOfMassMm']=dict(x=xp['poseMm']['x'],y=new_y-depth/2,z=p['analysisTopXBridge']['midZMm'])
        row['centroidalInertiaKgMm2World']=dict(xx=mass*(depth*depth+thickness*thickness)/12,yy=mass*(width*width+thickness*thickness)/12,zz=mass*(width*width+depth*depth)/12,xy=0,xz=0,yz=0)
        add_part(p,row,('x','y'))
        notes+=['Upper X rail on beam top, lower rail on front: roll-couple span214 mm instead of150 mm. Rotate block radial/lateral stiffness and component inertia; add a flexible20 mm horizontal bridge, including its2.75 kg mass. No rigid bridge bypass. Tool lever and149 mm ram clearance unchanged; mounting is a concept, not collision-checked CAD.']
    elif name!='baseline':raise ValueError(name)
    return p,q,notes


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--packets',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--ccx',type=Path,required=True);parser.add_argument('--mesh',type=int,choices=(50,100),default=50);a=parser.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    packet=json.loads((a.packets/'y3150/manifest.json').read_text());packet['floorSupport']='anchored';pose=packet['poses'][4];rows=[];baseline=None
    for i,name in enumerate(('baseline','deep side girders','additional leg pair','beam down 10 mm','top/front X rails')):
        p,q,notes=variant(packet,pose,name);result=solve(p,q,a.mesh,a.ccx.resolve(),a.output/str(i));mode=first_gantry(result)
        moving=next(ax['movingMassKg'] for ax in p['axes'] if ax['axis']=='y');base_moving=next(ax['movingMassKg'] for ax in packet['axes'] if ax['axis']=='y')
        if baseline is None:baseline=result;reference=signature(mode)
        tracked=max(result['modalParticipation'],key=lambda m:mac(reference,signature(m)))
        beam=next(b for b in p['beams'] if b['id']=='beam')
        row=dict(name=name,notes=notes,firstGantryHz=mode['frequencyHz'],deltaGantryHz=mode['frequencyHz']-first_gantry(baseline)['frequencyHz'],firstGlobalHz=result['firstModesHz'][0],trackedGantryHz=tracked['frequencyHz'],trackedMAC=mac(reference,signature(tracked)),deltaMovingYKg=moving-base_moving,deltaFrameKg=p['totalMassKg']-packet['totalMassKg']-(moving-base_moving),beamCentreToToolTipMm=beam['startMm'][2]-q['toolTipMm'][2],ramClearanceMm=149,
            xyStiffnessNPerUm=[1/result['cases'][j]['complianceUmPerN'][j] for j in (0,1)],deltaXYStiffnessNPerUm=[1/result['cases'][j]['complianceUmPerN'][j]-1/baseline['cases'][j]['complianceUmPerN'][j] for j in (0,1)],result=result)
        rows.append(row);print(name,row['deltaGantryHz'],row['deltaXYStiffnessNPerUm'],row['deltaMovingYKg'],row['deltaFrameKg'],flush=True)
        sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(__file__).with_name('study.py'),Path(__file__).with_name('front_guides.py'),Path(__file__).with_name('modes.py')]}
        (a.output/'architecture.json').write_text(json.dumps(dict(status='Single governing-pose architecture screen; anchored baseline, no geometry adopted',meshMm=a.mesh,sourceSha256=sources,pose=pose,rows=rows),indent=2)+'\n')


if __name__=='__main__':main()
