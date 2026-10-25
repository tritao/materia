"""Provisional Y-drive series-stiffness ranking, without modal/structural solves.

Idealized stiffness multipliers have no selected replacement/mass or latency
claim. This never transplants the producer's frozen-model dynamic errors.
"""
import argparse
import hashlib
import json
import tarfile
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--frozen-input-archive',type=Path,required=True)
    p.add_argument('--mount-screen',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();seats=json.loads(a.mount_screen.read_text());rows=[];sources={}
    with tarfile.open(a.frozen_input_archive) as archive:
        for bed in (2600,3150):
            name=f'rack-joint/inputs/packets/y{bed}/manifest.json';member=archive.getmember(name)
            if not member.isfile():raise ValueError('Invalid frozen packet')
            raw=archive.extractfile(member).read();sources[name]=hashlib.sha256(raw).hexdigest()
            packet=json.loads(raw);axis=next(x for x in packet['axes'] if x['axis']=='y')
            compliance={k:v for k,v in axis['compliance'].items() if k.endswith('MPerN')}
            original=sum(compliance.values());base=1/original
            for seat in seats['screens']:
                if seat['frameDatum']!='corrected-placement-proposal':continue
                stiffness=seat['worst']['equivalentTangentialStiffnessNPerM']
                options=[]
                for name,hinge,reducer in [('unchanged',1,1),('gearbox2',1,2),('mount2',2,1),('mount4',4,1),('mount2-gearbox2',2,2),('mount4-gearbox2',4,2)]:
                    changed=dict(compliance);changed['hingeMPerN']/=hinge;changed['reducerMPerN']/=reducer
                    drive=1/sum(changed.values());combined=1/(1/drive+1/stiffness)
                    options.append(dict(option=name,driveOnlyPerPinionNPerM=drive,
                        withSeatPerPinionNPerM=combined,dualYDriveAndSeatsNPerM=2*combined,
                        changeAgainstFrozenNoSeatPercent=100*(combined/base-1),
                        replacementMassKg=None,geometryReleased=False))
                rows.append(dict(bedMm=bed,seatVariant=seat['variant'],contactModel=seat['contactModel'],
                    seatPerPinionNPerM=stiffness,additionalEndAnchorHardwarePerLocalFixtureKg=seat['additionalHardwareMassKg'],
                    frozenDrivePerPinionNPerM=base,
                    frozenDriveFlexibilityShares={k:v/original for k,v in compliance.items()},
                    driveRequiredToRestoreFrozenStiffnessNPerM=None if stiffness<=base else 1/(1/base-1/stiffness),
                    restoringFrozenStiffnessPossibleWithFiniteDrive=stiffness>base,
                    options=sorted(options,key=lambda r:r['withSeatPerPinionNPerM'],reverse=True)))
    out=dict(status='Provisional algebraic ranking only; corrected structural/error/fatigue results pending',sourceMembersSha256=sources,
        mountScreenSha256=hashlib.sha256(a.mount_screen.read_bytes()).hexdigest(),rows=rows,
        module2=dict(status='Excluded from current integral-foot geometry; redesign required',
            assumptions='m2, 15 teeth preserves pitch circumference; permissible feed force is not a stiffness rating',
            receiverFootThicknessIfOtherDatumsPreservedMm=9.584505690810467-(22-17.5),
            footCounterboreDepthMm=6,
            reason='Published m2 pitch height 22 vs 17.5 mm lowers this foot to 5.0845 mm, thinner than its 6 mm counterbore',
            additionalYRackFixedMassKg={2600:2*3.350*(4.1-2.6),3150:2*3.900*(4.1-2.6)},
            pinionMassChangeKg=None,applicationRatingQualified=False),
        limitations=['Multipliers are optimistic stiffness-only bounds; no supplier replacement or moving mass selected',
          'The native local geometry still needs placement correction, preload/bracket clearance and weld/contact qualification',
          'No frequency, braking error or cycle-time inference from this algebra',
          'Current exports have publication-reserve timing inflation; cycle comparisons suppressed',
          'No structural solve or frozen CAD edit'])
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=2)+'\n')
    best=[r for r in rows if r['bedMm']==2600 and r['seatVariant']=='proposed-end-anchors' and r['contactModel']=='washer-annulus'][0]
    print(json.dumps(dict(basePerPinionMNPerM=best['frozenDrivePerPinionNPerM']/1e6,
        endAnchoredSeatMNPerM=best['seatPerPinionNPerM']/1e6,
        options=[dict(name=o['option'],withSeatMNPerM=o['withSeatPerPinionNPerM']/1e6) for o in best['options']]),indent=2))


if __name__=='__main__':main()
