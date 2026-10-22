"""Review-only rack-seat series compliance; all joint stiffnesses are assumptions.

Two neighbouring supports share a rack load. No friction, fitted dowel, or
infinite seat stiffness is credited. This is a sensitivity input, not a tested
joint or a proof of contact/preload retention.
"""
import argparse,json,math
from pathlib import Path

def series(*values):
    if any(not math.isfinite(v) or v<=0 for v in values):raise ValueError('Positive finite stiffnesses required')
    return 1/sum(1/v for v in values)

def screen(axis,base,grip):
    E=210000.;G=E/(2*1.3);epoxyE=4000.;epoxyG=epoxyE/(2*1.35)
    As=20.1;pad=30*30-math.pi*3.3**2;grout=4.209505690810 # mm, nominal meshing datum
    # Use tensile stress area conservatively for threaded shank shear.
    terms=dict(rackBolts=2*G*As/28,frameBolts=2*G*As/grip,
        bondedDatum=2*epoxyG*pad/grout,seatAxial=E*(50*8+8*11)/100)
    axial=series(*terms.values())
    # Local mid-span bending plus the transverse cantilever from bolt line to
    # the rack face centre; no continuous bedding credited.
    normal=series(48*E*(50*8**3/12)/100**3,3*E*(100*8**3/12)/28**3,
                  2*epoxyE*pad/grout)
    helix=math.acos(3*math.pi/10)
    pressure=math.atan(math.tan(math.radians(20))/math.cos(helix))
    lateral=series(2*E*As/28,terms['frameBolts'],terms['bondedDatum'],E*100*8/28)
    compliance=dict(tangential=1/axial,normalProjection=math.tan(pressure)**2/normal,
                    helicalAxialProjection=math.tan(helix)**2/lateral)
    seat=1/sum(compliance.values());combined=series(base/1000,seat)
    return dict(axis=axis,baseDriveNPerM=base,termsNPerMm=terms,seatNPerM=seat*1000,
        complianceMmPerN=compliance,unprojectedAxialSeatNPerM=axial*1000,lateralSeatNPerMm=lateral,
        combinedDriveNPerM=combined*1000,driveStiffnessChangePercent=100*(combined*1000/base-1),
        seatNormalNPerMm=normal,seatNormalDeflectionAt350NMm=350/normal,
        softSeatCombinedDriveNPerM=series(base,seat*100),stiffSeatCombinedDriveNPerM=series(base,seat*10000))

def main():
    p=argparse.ArgumentParser();p.add_argument('--packet',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    packet=json.loads(a.packet.read_text());axes={r['axis']:r for r in packet['axes']}
    rows=[screen(axis,axes[axis]['compliance']['stiffnessNPerM'],42.509505690810 if axis=='y' else 34.509505690810) for axis in ('x','y')]
    result=dict(reviewOnly=True,assumptions=dict(steelEMPa=210000,steelPoisson=.3,epoxyEMPa=4000,epoxyPoisson=.35,
        activeStations=2,threadStressAreaMm2=20.1,framePitchMm=100,rackSideGripMm=28,
        unknowns=['joint bearing/contact stiffness','bolt preload retention and grout creep','normal preload travel','supplier drilling and dowel allowance']),axes=rows,
        limitations='Local pressure/helix force projections condensed into equivalent tangential compliance. Friction/contact gaps and global normal-force coupling are not resolved. Run structural/modal model including added mass; no SR-D3 pass claimed. 0.1x/10x seat stiffness bracket is sensitivity, not measured uncertainty.')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    for r in rows:print(r['axis'],'seat MN/m',r['seatNPerM']/1e6,'combined MN/m',r['combinedDriveNPerM']/1e6,'normal kN/mm',r['seatNormalNPerMm']/1000)
if __name__=='__main__':main()
