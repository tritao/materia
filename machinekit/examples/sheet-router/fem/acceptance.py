"""Assess the approved SR-D3 motion allocation; not hardware/cutting acceptance."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--packets',type=Path,required=True)
    parser.add_argument('--dynamic',type=Path,required=True,help='parent of y2600 and y3150 dynamic results')
    args=parser.parse_args()
    summary_path=args.packets/'summary.json'
    summary=json.loads(summary_path.read_text())
    rows=[];hashes={str(summary_path):hashlib.sha256(summary_path.read_bytes()).hexdigest(),str(Path(__file__)):hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    for bed in summary['variants']:
        length=bed['workY'];path=args.dynamic/f'y{length}/dynamic-error.json'
        dynamic=json.loads(path.read_text());hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        policies=[]
        for shaping in ('none','ZVD'):
            case=next(c for c in dynamic['cases'] if c['backend']=='ruckig' and c['structure']=='current' and c['shaper']==shaping)
            samples=[case,case['timeStepCheck'],*case['uncertainty']]
            contour=max(c['contour']['peakContourErrorMm'] for c in samples)
            residual=max(c['rapidToCut']['residualPeakAfterTransitionMm'] for c in samples)
            policies.append(dict(policy='jerk-limited '+shaping,contourMm=contour,residualMm=residual,
                                 addedTimePerMoveMs=case['contour']['addedTimePerMoveMs'],passes=contour<=.05 and residual<=.05))
        xy=[c['stiffnessNPerUm'] for c in bed['worstCutting'][:2]]
        rows.append(dict(workY=length,minimumXYStiffnessNPerUm=xy,staticPass=min(xy)>=2,
                         governingDynamicPose=dynamic['governingPose'],shaperTuningHz=dynamic['shaperTuningHz'],policies=policies,
                         modelPass=min(xy)>=2 and any(p['passes'] for p in policies)))
    if {r['workY'] for r in rows}!={2600,3150}:raise ValueError('Both beds required')
    passing=all(r['modelPass'] for r in rows)
    result=dict(status='Model SR-D3 passes; structure eligible to freeze' if passing else 'Model SR-D3 fails; do not freeze',
                passes=passing,rows=rows,evidenceSha256=hashes,
                cuttingAcceptance=False,remaining=['Required OM-R1 certified jerk-limited path execution and optional ZVD stage',
                  'SR6-C impact/sine-sweep modes and damping, shaper retuning and measured-value motion-error recheck',
                  'Fabrication, supplier interfaces, alignment, servo tracking and cutting qualification'])
    (args.packets/'acceptance.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if not passing:raise SystemExit(1)


if __name__=='__main__':main()
