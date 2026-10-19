"""Known four-seat flexibility/preload solver fixture for displacement calibration."""
import argparse
import json
from pathlib import Path
from independent_static import run_independent
from frame_solver import read_results
from leveling import add_influence_steps,offsets_from_tables
from packet import digest


def verify(ccx,output):
    output.mkdir(parents=True,exist_ok=True)
    nodes='\n'.join(f'{i}, {i}, 0, 0' for i in range(1,6))
    loads=[100,200,150,90]
    lines=['*NODE',nodes,'*ELEMENT,TYPE=SPRING2,ELSET=FEET',
        '1,1,5','2,2,5','3,3,5','4,4,5','*SPRING,ELSET=FEET','3,3','100000.',
        '*ELEMENT,TYPE=SPRING2,ELSET=COUPLED','5,1,2','6,2,3','7,3,4',
        '*SPRING,ELSET=COUPLED','3,3','50000.','*NSET,NSET=SUPPORTS','1,2,3,4','*NSET,NSET=OBS','1,2,3,4',
        '*BOUNDARY','1,1,2','2,1,2','3,1,2','4,1,2','5,1,3',
        '*STEP','*STATIC','*CLOAD',*[f'{i+1},3,{-f}' for i,f in enumerate(loads)],
        '*NODE PRINT,NSET=OBS','U','*NODE PRINT,NSET=SUPPORTS','RF','*NODE PRINT,NSET=SUPPORTS','U','*END STEP']
    path=output/'frame.inp';path.write_text('\n'.join(lines)+'\n')
    model={'supportConstraints':dict.fromkeys('abcd'),'springNPerMm':100000,
           'references':{'foot-'+c:{'translation':i+1} for i,c in enumerate('abcd')}}
    add_influence_steps(path,model);run_independent(ccx,output,60,1)
    offsets,check=offsets_from_tables(model,read_results((output/'frame.dat').read_text(),5))
    error=max(abs(offsets[c]*100000/f-1) for c,f in zip('abcd',loads))
    if error>1e-6:raise ValueError(f'Known leveling preloads failed: {error}')
    report={'passed':True,'maximumPreloadRelativeError':error,'calibration':check,
        'inputSha256':digest(path),'resultSha256':digest(output/'frame.dat'),
        'sourceSha256':{n:digest(Path(__file__).with_name(n)) for n in ('leveling.py','leveling_fixture.py','independent_static.py')}}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n');return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ccx',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(json.dumps(verify(Path(a.ccx).resolve(),a.output),indent=2))
