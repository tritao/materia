"""Re-screen baseline CAD at critical poses with the current model, separating model and hardware changes."""
import argparse
import hashlib
import json
from pathlib import Path
from study import solve


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--baseline',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--ccx',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    rows=[];hashes={}
    for length in (2600,3150):
        path=args.baseline/f'y{length}'/'manifest.json';packet=json.loads(path.read_text())
        hashes[str(length)]=hashlib.sha256(path.read_bytes()).hexdigest()
        for index in (1,4):
            row=solve(packet,packet['poses'][index],50,args.ccx.resolve(),args.output/f'y{length}-p{index}')
            rows.append(dict(workY=length,**row))
            print(length,index,[round(1/row['cases'][i]['complianceUmPerN'][i],4) for i in (0,1)],flush=True)
    identity={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__),Path(__file__).with_name('study.py'),Path(__file__).with_name('front_guides.py'))}
    (args.output/'baseline-corrected.json').write_text(json.dumps(dict(status='Critical-pose baseline comparison; not a full acceptance study',packetSha256=hashes,sourceSha256=identity,results=rows),indent=2)+'\n')


if __name__=='__main__':main()
