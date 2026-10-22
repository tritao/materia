"""Audit pose coverage, spring-energy condensation and reciprocal compliance."""
import argparse,json,math
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    cases=modes=0;worst=0.;partition=0.;rows=[]
    for mesh in (100,50):
        for bed in (2600,3150):
            root=a.root/f'macro-h{mesh}'/f'y{bed}'/'baseline';packet=json.loads((root/'packet.json').read_text())
            axes={r['axis']:r for r in packet['axes']};seats={r['axis']:r['seatNPerM'] for r in packet['rackSeatScreen']['axes']}
            files=list(root.glob('*/summary.json'));assert len(files)==9,(mesh,bed,len(files))
            poses=set()
            for path in files:
                d=json.loads(path.read_text());poses.add((d['pose']['x'],d['pose']['y'],d['pose']['z']))
                assert d['cuttingReciprocityRelativeError']<=.003
                assert all(math.isfinite(k) and k>0 for k in d['xyStiffnessNPerUm'])
                assert len(d['modes'])==40
                for m in d['modes']:
                    e=m['strainEnergyShare'];observed=e.get('rackSeatBolts',0)
                    expected=sum(e.get(axis.upper()+'drive',0)*axes[axis]['compliance']['stiffnessNPerM']/seats[axis] for axis in ('x','y'))
                    error=abs(observed-expected);worst=max(worst,error)
                    # CCX's printed displacements have finite precision; allow
                    # 0.1% of total energy or 1% of the condensed spring share.
                    assert error<=max(.001,.01*observed),(path,m['frequencyHz'],observed,expected)
                    partition=max(partition,abs(sum(e.values())-1))
                    # Match the solver deck's existing brace-subset print guard.
                    assert abs(sum(e.values())-1)<1e-5
                    assert math.isfinite(m['frequencyHz']) and m['frequencyHz']>0
                    modes+=1
                cases+=1
            assert poses=={(x,y,200) for x in (0,800,1600) for y in (0,bed/2,bed)}
            rows.append(dict(meshMm=mesh,bedMm=bed,poses=len(files),stiffnessPass=all(all(k>=2 for k in json.loads(f.read_text())['xyStiffnessNPerUm']) for f in files)))
    report=dict(structuralCases=cases,modesAudited=modes,maxCondensedEnergyAbsoluteShareError=worst,maxEnergyPartitionPrintedError=partition,
        rows=rows,status='Numerical/coverage checks pass; static design failures remain failures')
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
if __name__=='__main__':main()
