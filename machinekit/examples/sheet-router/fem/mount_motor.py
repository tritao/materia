"""Actual pivot/carrier/anchor local solid screen; no machine acceptance.

Both load reversals and both axial signs are retained. A force at the pitch
point includes the motor-housing torque once, through its moment arm. No extra
pinion torque is added. Pin/bush and thrust lost motion is reported separately.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from mount_solid import SolidFixture, mesh_stock
from mount_local import rotation, position
from mount_frame import Frame, Section


def pin_stiffness(diameter, span):
    """Simply supported pin, central bearing load; finite shear deformation."""
    area = math.pi*diameter**2/4
    inertia = math.pi*diameter**4/64
    section = Section(area,inertia,inertia,2*inertia,.9*area,.9*area)
    model = Frame()
    for name, z in [('front',-span/2),('bearing',0.),('rear',span/2)]:
        model.node(name,(0,0,z))
    for a,b in [('front','bearing'),('bearing','rear')]:
        model.beam(a,b,section,(1,0,0),'pin steel')
    model.fix('front',[0,1,2,5]); model.fix('rear',[0,1])
    radial = model.solve({'bearing':[1.,0,0,0,0,0]})
    tilt = model.solve({'bearing':[0,0,0,1.,0,0]})
    return 1/radial['work'],1/tilt['work']


def tooth_reaction(pinion, tangent):
    """Applied tooth force on the pinion, in the stationary carrier frame.

    The CAD loft has d(theta)/dz = hand*tan(beta)/radius. Its surface generator
    at the lower pitch point is (hand*tan(beta),0,1), so a normal contact force
    has Fz=-hand*Fx*tan(beta). Fx here is the reaction on the pinion, opposite
    the transmitted drive force. Axial sign is not an independent load choice.
    """
    hand = 1. if pinion['hand'] == 'right' else -1.
    beta = pinion['helixAngle']
    pressure = math.atan(math.tan(pinion['pressureAngle'])/math.cos(beta))
    return np.array([tangent,abs(tangent)*math.tan(pressure),-hand*tangent*math.tan(beta)])


def face(frame, radius):
    origin, normal = position(frame), rotation(frame)[:, 2]
    def selector(points):
        delta = points-origin
        distance = delta@normal
        radial2 = np.sum(delta*delta, axis=1)-distance*distance
        return (np.abs(distance) < 1e-5) & (radial2 <= radius*radius+1e-5)
    return selector


def canonical(x, y, z, normal):
    # Only a surface selector frame, not a machine mounting datum.
    normal = np.asarray(normal, float)
    # Selector avoids a quaternion conversion entirely.
    def selector(points, radius):
        delta = points-np.array([x,y,z])
        along = delta@normal
        return (np.abs(along) < 1e-5) & (np.sum(delta*delta, axis=1)-along*along <= radius*radius+1e-5)
    return selector


def cylinder(points, x, y, radius, low, high):
    return (np.abs(np.hypot(points[:,0]-x, points[:,1]-y)-radius) < 1e-5) & (points[:,2] >= low-1e-5) & (points[:,2] <= high+1e-5)


def fixture(data, drive, size):
    rows = {r['id']:r for r in data['mountInputs']}
    native = {r['id']:r for r in data['parts']}
    carrier, anchor = rows[drive+'Preload'], rows[drive+'Anchor']
    model = SolidFixture()
    model.port('carrier-bearing', (0,0,7.5)); model.port('anchor-bearing', (0,0,7.5))
    model.port('carrier-spring', (-56,-10,7.5)); model.port('anchor-spring', (-56,-25,7.5))
    model.port('motor', (80,0,0))
    carrier_patches = {'carrier-bearing':lambda p:cylinder(p,0,0,7.,0,15),
                       'carrier-spring':lambda p:canonical(-56,-10,7.5,[0,-1,0])(p,12.)}
    for i, hole in enumerate(carrier['carrierMotorFlange']['holes']):
        point = (80+hole['x'],hole['y'],15.)
        name = 'motor-bolt'+str(i)
        model.port(name, point)
        carrier_patches[name] = lambda p, point=point:canonical(*point,[0,0,1])(p,6.)
        grip = 15+carrier['carrierMotorFlange']['flangeThickness']+3.2
        axial = 1/(grip/(210000*20.1)+1/(210000*6*15))
        shear = 1/(grip/((210000/2.6)*20.1)+1/(210000*6*15))
        model.spring(name,'motor',[shear,shear,axial,0,0,0],'motor flange bolts',point)
    anchor_patches = {
        'anchor-bearing':lambda p:(cylinder(p,0,0,5.,-9,-1) | cylinder(p,0,0,5.,16,24)),
        'anchor-spring':lambda p:(np.abs(np.hypot(p[:,0]+56,p[:,2]-7.5)-5.) < 1e-5) & (p[:,1] >= -43-1e-5) & (p[:,1] <= -29+1e-5)}
    top = anchor['pivotAnchor']['topMount']
    for i, bore in enumerate(b for b in anchor['pivotAnchor']['bores'] if b['id'].startswith('host-')):
        name = 'host-bolt'+str(i)
        point = position(bore['frame'])
        model.port(name, point)
        anchor_patches[name] = face(bore['frame'],8.)
        grip = bore['depth']+1.6
        installed_bolt = rows[drive+bore['id']+'Bolt']['parameters']
        if installed_bolt['size'] != 'M8':
            raise ValueError('Root connection model requires its actual M8 bolt')
        engagement = installed_bolt['length']-grip
        host_e = 210000. if top else 70000.
        axial = 1/((grip+engagement/2)/(210000*36.6)+1/(host_e*8*engagement))
        shear = 1/(grip/((210000/2.6)*36.6)+1/(host_e*8*engagement))
        normal = rotation(bore['frame'])[:,2]
        stiffness = [axial if abs(v)>.99 else shear for v in normal]+[0,0,0]
        model.spring(name,None,stiffness,'root bolts/host thread bearing')
    proofs = []
    for row, patches in ((carrier,carrier_patches),(anchor,anchor_patches)):
        xyz, tets, surface, proof = mesh_stock(row,native[row['id']]['volumeMm3'],size)
        model.solid(row['id'],xyz,tets,patches,surface=surface)
        proof['id'] = row['id']; proofs.append(proof)
        print('Meshed',row['id'],proof['nodes'],'nodes',proof['tetrahedra'],'tetrahedra',flush=True)
    # Ten-millimetre pin, 25 mm support span; conservative simply-supported
    # pin bending and finite bronze compression. No rated hinge stiffness.
    pin_sections = json.loads(rows[drive+'PivotPin']['parameters']['sections'])
    bush = rows[drive+'PivotBush']['parameters']
    thrust_part = rows[drive+'PivotThrustFront']['parameters']
    if len(pin_sections) != 1:
        raise ValueError('Model each stepped pin section explicitly')
    diameter = pin_sections[0]['diameter']
    lugs = sorted((h for h in anchor['pivotAnchor']['bores'] if h['id'].startswith('hinge-')), key=lambda h:h['frame']['z'])
    lug_centres = [h['frame']['z']+h['depth']/2 for h in lugs]
    if len(lug_centres) != 2:
        raise ValueError('Pin requires two actual support lugs')
    span = abs(lug_centres[1]-lug_centres[0])
    wall = (bush['outerDiameter']-bush['boreDiameter'])/2
    length = bush['length']
    pin_translation,pin_tilt = pin_stiffness(diameter,span)
    bronze = 100000*diameter*length/wall
    radial = 1/(1/pin_translation+1/bronze)
    bronze_tilt = 100000*diameter*length**3/(12*wall)
    tilt = 1/(1/pin_tilt+1/bronze_tilt)
    thrust = 100000*math.pi*(thrust_part['outerDiameter']**2-thrust_part['boreDiameter']**2)/(4*thrust_part['length'])
    model.spring('carrier-bearing','anchor-bearing',[radial,radial,thrust,tilt,tilt,0.],
                 'pin bending/bush/thrust contact')
    pack_rate = carrier['parameters']['springRate']
    adjuster = 210000*(math.pi*(10-1.226869*1.5)**2/4)/17
    spring_rate = 1/(1/pack_rate+1/adjuster)
    model.spring('carrier-spring','anchor-spring',[0,spring_rate,0,0,0,0],
                 'disc pack/adjuster')
    joint = dict(pinDiameterMm=diameter, supportSpanMm=span, bushLengthMm=length,
                 bushRadialWallMm=wall, radialStiffnessNPerMm=radial,
                 tiltStiffnessNmmPerRad=tilt, thrustStiffnessNPerMm=thrust,
                 springRateNPerMm=spring_rate,
                 pinDiametralClearanceMm=bush['boreDiameter']-diameter,
                 totalAxialEndplayMm=lugs[1]['frame']['z']-lugs[0]['frame']['z']-lugs[0]['depth']-bush['length']-2*thrust_part['length'])
    return model, proofs, rows[drive]['parameters'], carrier['parameters'], joint


def run(data, drive, size):
    model, proofs, pinion, carrier, joint = fixture(data,drive,size)
    helix = pinion['helixAngle']
    radius = pinion['normalModule']*pinion['teeth']/2/math.cos(helix)
    point = np.array([carrier['leverArm'],-radius,carrier['thickness']+carrier['pinionAxialOffset']])
    results = []
    for tangent in (-1.,1.):
        force = tooth_reaction(pinion,tangent)
        wrench = np.r_[force,np.cross(point-model.ports['motor'],force)]
        result = model.solve({'motor':wrench})
        spring_delta = result['displacement']['carrier-spring'][1]-result['displacement']['anchor-spring'][1]
        results.append(dict(tangentialSign=tangent,axialSign=math.copysign(1,force[2]),
                            forcePerTangentialN=force.tolist(),motorWrenchPerTangentialN=wrench.tolist(),
                            equivalentComplianceMPerN=result['work']/1000,
                            equivalentStiffnessNPerM=1000/result['work'],
                            springForceIncrementPerTangentialN=-spring_delta*joint['springRateNPerMm'],**result))
        print(drive,size,tangent,'MN/m',.001/result['work'],flush=True)
    return dict(drive=drive,sizeMm=size,meshes=proofs,pitchPointMm=point.tolist(),cases=results,
                geometricJoint=joint,
                worst=min(results,key=lambda r:r['equivalentStiffnessNPerM']),
                lostMotionInputs={key:joint[key] for key in ('pinDiametralClearanceMm','totalAxialEndplayMm')},
                assumptions=['Steel E210 GPa/nu0.3; bronze E100 GPa; X host E70 GPa, all unmeasured',
                             'Linear tetrahedra and rigid bearing/washer patches; convergence required',
                             'Only bolt bearing/shear/axial stiffness credited at motor and host roots; no friction credit',
                             'Root host global plate bending belongs to global model; local thread bearing retained',
                             'Pin support and bronze contact are elastic geometric surrogates, not measured bearing stiffness',
                             'Gap/endplay, unilateral contact, preload retention and stroke must bound any final admission'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',required=True,type=Path)
    parser.add_argument('--drive',required=True,choices=['pinionYLeft','pinionYRight','pinionX'])
    parser.add_argument('--size',type=float,default=3.)
    parser.add_argument('--output',required=True,type=Path)
    args = parser.parse_args()
    raw = args.input.read_bytes()
    result = run(json.loads(raw),args.drive,args.size)
    args.output.write_text(json.dumps(dict(status='Local elastic diagnostic; no SR5/SR6 acceptance',
        inputSha256=hashlib.sha256(raw).hexdigest(),**result),indent=2)+'\n')


if __name__ == '__main__': main()
