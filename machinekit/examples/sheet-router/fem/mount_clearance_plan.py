"""Light, conservative whole-machine Y-mount envelope screen; no CAD build.

Frames come from actual pinion pose/declared axial width, never a supportedSide
bounding-box heuristic. Positive box separation proves envelope clearance;
overlap only requests a native check and is never labelled a solid collision.
"""
import argparse
import hashlib
import itertools
import json
import math
import tarfile
from pathlib import Path

import numpy as np
from mount_compliance import witness,parameters


def rotate(q,v):
    xyz=np.array([q['qx'],q['qy'],q['qz']]);w=q['qw']
    if abs(float(xyz@xyz)+w*w-1)>1e-6:raise ValueError('Invalid source quaternion')
    v=np.asarray(v,dtype=float)
    return v+2*np.cross(xyz,np.cross(xyz,v)+w*v)


def box(lo,hi):
    return {'min'+a:float(lo[i]) for i,a in enumerate('XYZ')} | {'max'+a:float(hi[i]) for i,a in enumerate('XYZ')}


def corners(b):
    return np.array(list(itertools.product(*[(b['min'+a],b['max'+a]) for a in 'XYZ'])))


def transformed(bounds,origin,basis,longitudinal):
    points=corners(bounds)@basis.T+origin
    result=box(points.min(axis=0),points.max(axis=0));result['minY'],result['maxY']=longitudinal
    return result


def shifted(bounds,delta):
    return {key:float(value+delta['XYZ'.index(key[-1])]) for key,value in bounds.items()}


def screen(packet,native,pinion_width):
    if not math.isfinite(pinion_width) or pinion_width<=0:raise ValueError('Positive declared pinion width required')
    parts={p['id']:p for p in packet['parts']};recipes=parameters(native['assembly/assembly.json'])
    rec=recipes['receiver']['values'];geo=native['assembly/geometry.json']
    fixture={p['id']:p for p in native['assembly/mass.json']['parts']}
    origin_height=geo['datumMm']['rackBack']-fixture['rack']['boundsMm']['minX']
    local_pinion_x=geo['datumMm']['pinionCentre']-origin_height
    local_pinion_z=rec['thickness']+1.6+19/2
    # The two dimensions above are checked against the actual native rack and
    # washer bounds, not extrapolated from a supplier's assumed mounting side.
    rb=fixture['rack']['boundsMm'];sp=fixture['rack-spacer1']['boundsMm']
    local_pinion_z=(rb['minZ']+rb['maxZ'])/2
    if abs((sp['maxZ']-sp['minZ'])-1.6)>1e-6:raise ValueError('Changed source stand-off')
    moving={a['axis']:{p['id'] for p in a['movingMembers']} for a in packet['axes']}
    bed=packet['workY'];placements=[];flags={};compared=0
    for side in ('Left','Right'):
        pinion=parts['pinionY'+side];pose=pinion['poseMm'];normal=rotate(pose,[0,0,1])
        if abs(abs(normal[0])-1)>1e-6 or max(abs(normal[1:]))>1e-6:
            raise ValueError('Y pinion does not have the declared transverse shaft axis')
        axial=pinion['boundsMm']['maxX']-pinion['boundsMm']['minX']
        if abs(axial-pinion_width)>1e-5:raise ValueError('Declared pinion width differs from its source envelope')
        centre=np.array([pose['x'],pose['y'],pose['z']])+normal*pinion_width/2
        u=np.array([0.,0.,1.]);w=-normal;v=np.cross(w,u);basis=np.column_stack([u,v,w])
        origin=centre-u*local_pinion_x-w*local_pinion_z
        origin-=v*float(v@origin)  # fixed Y-rack longitudinal datum, independent of carriage Y
        rail=parts['railY'+side];rail_position=np.array([rail['poseMm'][a] for a in ('x','y','z')])
        required_local_w=float(w@(rail_position-origin))
        prototype_local_w=fixture['rail']['poseMm']['z']
        predicted=origin+u*fixture['rail']['poseMm']['x']+w*prototype_local_w
        difference=predicted[0]-rail_position[0]
        placements.append(dict(side=side,pinionPoseOriginMm=[pose[a] for a in ('x','y','z')],
            pinionAxialCentreMm=centre.tolist(),declaredPinionWidthMm=pinion_width,
            receiverOriginMm=origin.tolist(),receiverBasis=basis.tolist(),
            requiredRailLocalWidthCoordinateMm=required_local_w,
            prototypeRailLocalWidthCoordinateMm=prototype_local_w,
            prototypeRailLateralErrorMm=float(difference)))
        rack_bounds=parts['rackY'+side]['boundsMm'];longitudinal=(rack_bounds['minY']-20,rack_bounds['maxY']+20)
        candidates={
            'upright':transformed(box([-rec['width']/2,-1,0],[rec['width']/2,1,rec['thickness']]),origin,basis,longitudinal),
            'foot':transformed(box([-rec['width']/2-rec['flangeThickness'],-1,0],[-rec['width']/2,1,rec['flangeWidth']]),origin,basis,longitudinal)}
        frame=parts['frame'+side];x=frame['poseMm']['x'];top=frame['boundsMm']['maxZ'];sign=w[0]
        # Candidate shared datum follows the unchanged master rail/frame, after
        # the placement discrepancy is identified. No production CAD is edited.
        pad_x=sorted([x-sign*50,x+sign*90])
        candidates['pad']=box([pad_x[0],longitudinal[0],top],[pad_x[1],longitudinal[1],top+8])
        candidates['replication']=box([pad_x[0],longitudinal[0],top+8],[pad_x[1],longitudinal[1],top+12])
        skip={'rackYLeft','rackYRight','railPadYLeft','railPadYRight','chainX','chainY'}
        for px,py,pz in itertools.product((0,800,1600),(0,bed/2,bed),(0,200)):
            for name,candidate in candidates.items():
                for part in packet['parts']:
                    identity=part['id']
                    if identity in skip:continue
                    delta=np.array([px-800 if identity in moving['x'] else 0,
                        py-bed/2 if identity in moving['y'] else 0,
                        200-pz if identity in moving['z'] else 0])
                    other=shifted(part['boundsMm'],delta);compared+=1
                    overlap=[min(candidate['max'+a],other['max'+a])-max(candidate['min'+a],other['min'+a]) for a in 'XYZ']
                    if min(overlap)<=1e-5:continue
                    key=(side,name,identity)
                    if key not in flags:flags[key]=dict(side=side,candidate=name,existingMember=identity,
                        overlapPoses=0,firstPoseMm=dict(x=px,y=py,z=pz),maxOverlapBoxMm=overlap)
                    flags[key]['overlapPoses']+=1
                    flags[key]['maxOverlapBoxMm']=[max(a,b) for a,b in zip(flags[key]['maxOverlapBoxMm'],overlap)]
    return dict(bedMm=bed,poses=18,pairChecks=compared,placements=placements,
        nativeChecksRequired=list(flags.values()),envelopePass=not flags,
        scope='Conservative boxes and translation membership; not a native collision or full-clearance pass')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fixture-archive',type=Path,required=True);p.add_argument('--validation',type=Path,required=True)
    p.add_argument('--frozen-input-archive',type=Path,required=True);p.add_argument('--pinion-width-mm',type=float,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    native=witness(a.fixture_archive,a.validation);records=[];hashes={}
    with tarfile.open(a.frozen_input_archive) as archive:
        for bed in (2600,3150):
            name=f'rack-joint/inputs/packets/y{bed}/manifest.json';member=archive.getmember(name)
            if not member.isfile():raise ValueError('Invalid frozen input')
            raw=archive.extractfile(member).read();hashes[name]=hashlib.sha256(raw).hexdigest()
            records.append(screen(json.loads(raw),native,a.pinion_width_mm))
    out=dict(status='Y candidate placement/envelope screen only; native checks outstanding',sourceMembersSha256=hashes,
        fixtureArchiveSha256=native['archiveSha256'],beds=records,
        limitations=['Axial pinion centre derived from its actual pose and explicit shaft width; no supportedSide heuristic',
          'Prototype lateral reference discrepancy must be fixed before full-machine CAD qualification',
          'Overlapping boxes request native checks and do not prove material interference',
          'X mounting host, rack section joints, nut/end-cassette withdrawal and all articulated chains remain open',
          'No structural solve or frozen CAD edit'])
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps([dict(bed=r['bedMm'],pairChecks=r['pairChecks'],nativeChecks=len(r['nativeChecksRequired']),
        railPlacementErrorsMm=[p['prototypeRailLateralErrorMm'] for p in r['placements']]) for r in records]))


if __name__=='__main__':main()
