"""Geometry-derived, provisional local mount screen. No structural/modal solve.

Reads the native-CAD witness archive without extraction. Uses its actual section,
source drilling, bolt grips and stations; reports load-path assumptions explicitly.
This is neither a full-length machine packet nor a qualified contact/preload model.
"""
import argparse
import hashlib
import json
import math
import re
import tarfile
from pathlib import Path

import numpy as np

BASE = 'build/sheet-router-work/shared-mount/'


def decode_wire(value):
    if isinstance(value, list):
        if value and all(isinstance(p,list) and len(p)==2 and isinstance(p[0],str)
                         and re.match(r'^\d+:',p[0]) for p in value):
            return {k.split(':',1)[1]:decode_wire(v) for k,v in value}
        return [decode_wire(v) for v in value]
    if isinstance(value,dict):
        return {k:decode_wire(v) for k,v in value.items()}
    return value


def witness(archive, validation):
    proof=json.loads(validation.read_text())
    with archive.open('rb') as stream:
        if hashlib.file_digest(stream,'sha256').hexdigest()!=proof['archiveSha256']:
            raise ValueError('Native witness archive hash mismatch')
    known={r['path']:r for r in proof['verifiedFiles']}
    result={}
    with tarfile.open(archive) as source:
        for name in ('assembly/assembly.json','assembly/geometry.json','assembly/mass.json','access/access.json','access/mass.json'):
            path=BASE+name;member=source.getmember(path)
            if not member.isfile() or member.size>20_000_000:raise ValueError('Invalid native witness member')
            raw=source.extractfile(member).read()
            if hashlib.sha256(raw).hexdigest()!=known[path]['sha256']:raise ValueError('Native witness member differs')
            result[name]=json.loads(raw)
    result['archiveSha256']=proof['archiveSha256']
    return result


def parameters(encoded):
    machine=decode_wire(encoded)['value']['machine'];out={}
    for member in machine['members']:
        typ,entries=member['source']['Typed']
        values={}
        for entry in entries:
            value=entry['value']
            if len(value)!=1:raise ValueError('Unknown recipe parameter encoding')
            values[entry['name']]=next(iter(value.values()))[0]
        out[member['occurrence']]={'type':typ,'values':values}
    return out


def section(rectangles):
    """Signed rectangles (area sign,width X,width Z,centroid X,Z), mm."""
    area=sum(sign*x*z for sign,x,z,_,_ in rectangles)
    if area<=0:raise ValueError('No net receiver section')
    centre=np.array([sum(sign*x*z*p[i] for sign,x,z,*p in rectangles)/area for i in (0,1)])
    inertia=np.zeros((2,2))
    for sign,x,z,cx,cz in rectangles:
        a=sign*x*z;offset=np.array([cx,cz])-centre
        inertia+=a*np.outer(offset,offset)+sign*np.diag([z*x**3/12,x*z**3/12])
    if np.min(np.linalg.eigvalsh(inertia))<=0:raise ValueError('Invalid net section tensor')
    return area,centre,inertia


def weights(stations,point):
    stations=sorted(set(stations))
    if len(stations)<2:raise ValueError('At least two distinct supports required')
    i=next((i for i in range(len(stations)-1) if point<=stations[i+1]),len(stations)-2)
    a,b=stations[i:i+2];right=(point-a)/(b-a)
    return (a,b),np.array([1-right,right])


def beam_factor(stations,point):
    """Idealized support law, not a resolved preload/contact joint.

    Interior point load: simply supported span. Outside: last station assumed
    rotationally fixed; actual overhang restraint can be softer.
    """
    (a,b),_=weights(stations,point)
    if point<a:return (a-point)**3/3
    if point>b:return (point-b)**3/3
    return (point-a)**2*(b-point)**2/(3*(b-a))


def screen(data,end_anchors=False,contact='washer-annulus',host_frame_centre=None):
    recipes=parameters(data['assembly/assembly.json'])
    rec=recipes['receiver']['values'];pad=recipes['pad']['values'];layer=recipes['replication']['values']
    holes=json.loads(rec['holes'])['holes'];parts={p['id']:p for p in data['assembly/mass.json']['parts']}
    mass=data['assembly/mass.json'];access=data['access/mass.json']
    if not math.isclose(sum(p['massKg'] for p in mass['parts']),mass['totalMassKg'],abs_tol=1e-10):
        raise ValueError('CAD body masses do not sum')
    source=[h for h in holes if h['id'].startswith('mount-')]
    anchor=[h for h in holes if h['id'].startswith('anchor')]
    pushes=[h for h in holes if h['id'].startswith('push')]
    rack_stations=[h['frame']['y'] for h in source]
    anchors=[h['frame']['y'] for h in anchor]
    width,thickness,flange,foot=[rec[k] for k in ('width','thickness','flangeWidth','flangeThickness')]
    low,high=parts['rack']['boundsMm']['minY'],parts['rack']['boundsMm']['maxY']
    added_mass=0.
    if end_anchors:
        # Proposal only: not silently added to the CAD witness. 20 mm inset
        # from each actual rack end; same anchor station Z as existing holes.
        anchors=sorted(set(anchors+[low+20,high-20]))
        added_mass=2*sum(parts[k]['massKg'] for k in ('anchor-bolt1','anchor-nut1','anchor-washer1'))
    E=210000.;G=E/(2*1.3);epoxyE=4000.
    major,pitch=source[0]['thread']['majorDiameter'],source[0]['thread']['pitch']
    # Same metric external root convention as cadkit ThreadSpec. Root area is
    # used as an explicitly conservative elastic-section surrogate, not proof.
    root=major-1.226869*pitch;area=math.pi*root**2/4
    grip=data['assembly/geometry.json']['anchors'][0]['gripMm']
    spacer=parts['rack-spacer1']['boundsMm']['maxZ']-parts['rack-spacer1']['boundsMm']['minZ']
    rack_face=parts['rack']['boundsMm']['maxZ']-parts['rack']['boundsMm']['minZ']
    # Source counterbore depth from its bearing plane: rack mounting bore has
    # published 7 mm recess; receiver source requirement retains that geometry
    # via rack-bolt pose. Project the actual native bolt/receiver separation.
    bolt=parts['rack-bolt1']['poseMm'];mount=source[0]['frame']
    rack_grip=abs(bolt['z']-mount['z'])
    if rack_grip<=0:raise ValueError('Missing native rack bolt grip')
    washer=parts['anchor-washer1'];w=washer['boundsMm']
    washer_area=washer['volumeMm3']/(w['maxX']-w['minX'])
    frame_width=data['access/access.json']['tubeMm']['width']
    frame_centre=parts['rail']['poseMm']['z'] if host_frame_centre is None else host_frame_centre
    edge=frame_centre+frame_width/2
    lever=abs(anchor[0]['frame']['z']-edge)
    base=[(1.,width,thickness,0.,thickness/2),
          (1.,foot,flange,-width/2-foot/2,flange/2)]
    # Weak prismatic surrogate: superpose maximum hole slices, although source
    # bores/push screws and anchors occupy different longitudinal stations.
    # This preserves a lower section than either drilled slice; it is a bound
    # on section stiffness, not a calibrated joint uncertainty interval.
    h=source[0];base.append((-1.,h['diameter'],h['depth'],h['frame']['x'],thickness-h['depth']/2))
    h=anchor[0];base.append((-1.,foot,h['diameter'],-width/2-foot/2,h['frame']['z']))
    if h.get('counterbore'):
        cb=h['counterbore'];base.append((-1.,cb['depth'],cb['diameter']-h['diameter'],-width/2-cb['depth']/2,h['frame']['z']))
    h=pushes[0];base.append((-1.,foot,h['diameter'],-width/2-foot/2,h['frame']['z']))
    # Integrate the union of stock minus the union of void slices. Never
    # subtract overlapping anchor/push bore slices twice.
    xs=sorted({cx+side*x/2 for _,x,_,cx,_ in base for side in (-1,1)})
    zs=sorted({cz+side*z/2 for _,_,z,_,cz in base for side in (-1,1)})
    cells=[]
    for x0,x1 in zip(xs,xs[1:]):
        for z0,z1 in zip(zs,zs[1:]):
            cx,cz=(x0+x1)/2,(z0+z1)/2
            def covers(r):return abs(cx-r[3])<r[1]/2+1e-10 and abs(cz-r[4])<r[2]/2+1e-10
            if any(r[0]>0 and covers(r) for r in base) and not any(r[0]<0 and covers(r) for r in base):
                cells.append((1.,x1-x0,z1-z0,cx,cz))
    net,centroid,I=section(cells);invI=np.linalg.inv(I)
    helix=math.acos(3*math.pi/10);pressure=math.atan(math.tan(math.radians(20))/math.cos(helix))
    observed=[]
    for point in np.linspace(low,high,101):
        _,r=weights(rack_stations,float(point));(a,b),f=weights(anchors,float(point));rf=float(r@r);ff=float(f@f)
        # X normal/back, Y tangential, Z rack width/helical direction. All
        # compliance terms are mm/N until the final explicit SI conversion.
        terms={}
        terms['rack-bolts']=np.diag([rf*rack_grip/(G*area),rf*rack_grip/(G*area),rf*rack_grip/(E*area)])
        terms['frame-bolts']=np.diag([ff*grip/(E*area),ff*grip/(G*area),ff*grip/(G*area)])
        beam=np.zeros((3,3));beam[np.ix_([0,2],[0,2])]=beam_factor(anchors,float(point))*invI/E
        beam[1,1]=max(b-a,1)/(E*net)
        terms['integral-receiver-foot']=beam
        terms['welded-pad']=np.diag([ff*lever**3/(3*E*((b-a)*pad['thickness']**3/12)),
            ff*lever/(G*(b-a)*pad['thickness']),ff*lever/(E*(b-a)*pad['thickness'])])
        normal_area=washer_area if contact=='washer-annulus' else flange*(b-a)
        compound=np.zeros((3,3));compound[0,0]=ff*layer['thickness']/(epoxyE*normal_area)
        # Tangential/shear path is through bolts: no bonded-grout shear or
        # clamp-friction stiffness is credited.
        terms['replication-normal-only']=compound
        pushroot=pushes[0]['thread']['majorDiameter']-1.226869*pushes[0]['thread']['pitch']
        push=np.zeros((3,3));push[0,0]=rf*recipes['adjuster1']['values']['length']/(E*math.pi*pushroot**2/4)
        terms['push-screw']=push
        projections=[]
        for sx,sz in ((-1,-1),(-1,1),(1,-1),(1,1)):
            path={k:v.copy() for k,v in terms.items()}
            if sx<0:
                # Compressive/back load uses the push tips and bedding; do not
                # put the parallel rack-bolt shear/anchor tension path in series.
                path['rack-bolts'][0,0]=0;path['frame-bolts'][0,0]=0
            else:
                # Uplift has no tensile push-screw or bonded-grout credit.
                path['push-screw'][0,0]=0;path['replication-normal-only'][0,0]=0
            C=sum(path.values())
            load=np.array([sx*math.tan(pressure),1.,sz*math.tan(helix)])
            flexibility=float(load@C@load)
            projections.append((flexibility,load,C,path))
        flex,load,C,path=max(projections,key=lambda t:t[0])
        observed.append(dict(stationMm=float(point),equivalentTangentialStiffnessNPerM=1000/flex,
            flexibilityMatrixMPerN=(C/1000).tolist(),loadDirection=load.tolist(),
            projectedTermMPerN={k:float(load@v@load)/1000 for k,v in path.items()},
            frameReactionWeightSquared=ff,rackReactionWeightSquared=rf,normalPath='compression-push-bedding' if load[0]<0 else 'uplift-bolts'))
    worst=min(observed,key=lambda r:r['equivalentTangentialStiffnessNPerM'])
    return dict(variant='proposed-end-anchors' if end_anchors else 'native-three-anchor-fixture',contactModel=contact,
        anchorStationsMm=anchors,additionalHardwareMassKg=added_mass,netAreaMm2=net,
        frameDatum='native-prototype' if host_frame_centre is None else 'corrected-placement-proposal',frameCentreLocalWidthMm=frame_centre,
        sectionCentroidXZMm=centroid.tolist(),sectionInertiaCoordinateMm4=I.tolist(),
        geometry=dict(rackBoltGripMm=rack_grip,anchorBoltGripMm=grip,padOverhangLeverMm=lever,
            sourceBoltStationsMm=rack_stations,footThicknessMm=foot,compoundThicknessMm=layer['thickness']),
        worst=worst,stationCount=len(observed),
        assumptions=['E steel 210 GPa, Poisson 0.3; replication E 4 GPa; properties unmeasured',
          'Linear bolt root-area stiffness; contact, fit, preload loss and thread-bearing compliance unresolved',
          'Union of maximum drilled slices used as a weak prismatic section surrogate',
          'Simply supported interior spans; rotationally fixed end-overhang assumption, not qualified restraint',
          'Replicated footprint assumption is sensitivity only; no bonded-shear/friction credit',
          'No weld flexibility, rotational boundary qualification or rack-section joint model; not final seat stiffness'],
        scope='Local geometry-derived sensitivity. Do not use as a complete corrected-model release input.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--archive',type=Path,required=True);p.add_argument('--validation',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--clearance-plan',type=Path);a=p.parse_args()
    data=witness(a.archive,a.validation)
    screens=[screen(data,end,contact) for end in (False,True) for contact in ('washer-annulus','tributary-foot')]
    if a.clearance_plan:
        placement=json.loads(a.clearance_plan.read_text())
        if placement['fixtureArchiveSha256']!=data['archiveSha256']:raise ValueError('Placement and native fixture differ')
        centres=[p['requiredRailLocalWidthCoordinateMm'] for b in placement['beds'] for p in b['placements']]
        if max(centres)-min(centres)>1e-6:raise ValueError('Host sections need distinct mount models')
        screens += [screen(data,end,contact,float(np.mean(centres))) for end in (False,True) for contact in ('washer-annulus','tributary-foot')]
    total=data['assembly/mass.json']['totalMassKg'];parts={p['id']:p for p in data['assembly/mass.json']['parts']}
    out=dict(status='Provisional local geometry/compliance screen; no structural solve',archiveSha256=data['archiveSha256'],
        nativeFixtureMassKg=total,mountPartsExcludingExistingRailAndRackKg=total-parts['rail']['massKg']-parts['rack']['massKg'],
        screens=screens,limits=['No per-metre/full-bed mass extrapolation: fixture lengths differ',
            'No corrected static/modal/dynamic or fatigue acceptance',
            'Added end anchors are a study proposal, not a native-checked CAD change'])
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps([dict(variant=s['variant'],contact=s['contactModel'],datum=s['frameDatum'],worstSeatMNPerM=s['worst']['equivalentTangentialStiffnessNPerM']/1e6,
        stationMm=s['worst']['stationMm']) for s in screens]))


if __name__=='__main__':main()
