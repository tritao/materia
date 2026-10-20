"""Coarse-mesh check of governing prepared-weld reaction envelopes."""
import argparse,json
from pathlib import Path
from qualify_welds import read_stitches,run_case
from weld_model import with_foot_welds
from weld_effective import effective_stitches


def main():
    p=argparse.ArgumentParser()
    for name in ('packets','fabrication','ccx','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--envelope',type=Path)
    a=p.parse_args()
    for bed in (3150,2600):
        packet=json.loads((a.packets/f'y{bed}/manifest.json').read_text())
        native=json.loads((a.fabrication/f'y{bed}/native.json').read_text());physical=read_stitches(a.fabrication/f'y{bed}')
        governing=0 if bed==2600 else bed
        critical=[(800,governing),(800,bed/2)]
        if a.envelope:
            rows=[r for r in json.loads(a.envelope.read_text())['poses'] if r['workY']==bed]
            worst=max(rows,key=lambda r:max(c['fatigueQuasiStaticRangeMPa'] for c in r['contacts']))['pose']
            key=(worst['x'],worst['y'])
            if key not in critical:critical.append(key)
        for x,y in critical:
            pose=next(q for q in packet['poses'] if q['x']==x and q['y']==y)
            joined,stitches=with_foot_welds(packet,native,physical,pose);stitches,_=effective_stitches(joined,stitches)
            result=run_case(packet,joined,stitches,pose,100,a.ccx.resolve(),a.output/f'y{bed}-x{x:g}-y{y:g}',40 if x==800 and y==governing else 0)
            print(bed,x,y,'XY',[.3/result['cases'][i]['tipRelativeMm'][i] for i in (0,1)],flush=True)


if __name__=='__main__':main()
