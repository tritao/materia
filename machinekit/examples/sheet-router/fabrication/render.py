"""Render the native SR6 draft packet as inspectable shop-planning CSV/SVG files."""
import argparse
import csv
import hashlib
from html import escape
import json
import math
from collections import defaultdict
from pathlib import Path


def write_csv(path,header,rows):
    with path.open('w',newline='') as stream:
        writer=csv.writer(stream,lineterminator="\n");writer.writerow(header);writer.writerows(rows)


def render(source,output,packets):
    data=json.loads(source.read_text());bed=int(data['workY']);members=data['members']
    if abs(sum(m['stockLengthMm'] for m in members)-sum(r['totalLength'] for r in data['cutList']))>1e-5:
        raise ValueError('Detailed stock and native grouped cut list disagree')
    manifest=packets/f'y{bed}/manifest.json';packet=json.loads(manifest.read_text())
    upper=[dict(id=b['id'],profile=f"RECT-{b['widthMm']:g}x{b['heightMm']:g}x{b['wallMm']:g}",startMm=b['startMm'],endMm=b['endMm'],stockLengthMm=math.dist(b['startMm'],b['endMm']),centrelineLengthMm=math.dist(b['startMm'],b['endMm'])) for b in packet['beams'] if b['id'] in ('frameFront','frameBack','frameLeft','frameRight')]
    if len(upper)!=4:raise ValueError('Four upper frame tubes required')
    members=members+upper
    root=output/f'y{bed}';root.mkdir(parents=True,exist_ok=True)
    write_csv(root/'cut-list.csv',['member','profile','stock_mm','centreline_mm','start_x','start_y','start_z','end_x','end_y','end_z'],
              [[m['id'],m['profile'],round(m['stockLengthMm'],4),round(m['centrelineLengthMm'],4),*m['startMm'],*m['endMm']] for m in members])
    totals=defaultdict(lambda:[0,0.])
    for m in members:totals[m['profile']][0]+=1;totals[m['profile']][1]+=m['stockLengthMm']
    write_csv(root/'stock-totals.csv',['profile','quantity','total_mm'],[[k,*v] for k,v in sorted(totals.items())])
    tie_ins=[[f'post{s}{i}','frameLeft' if s==0 else 'frameRight','post top to upper rail-support tube; joint preparation and leg TBD'] for s in (0,1) for i in range(4)]
    tie_ins += [[a,b,'upper-frame corner; joint preparation and leg TBD'] for a in ('frameFront','frameBack') for b in ('frameLeft','frameRight')]
    write_csv(root/'upper-frame-joints-to-detail.csv',['member_a','member_b','detail_required'],tie_ins)
    write_csv(root/'weld-joints.csv',['joint','member_a','member_b','fillet_leg_mm','found_seam_candidates'],
              [[i+1,m['a'],m['b'],m['legSize'],sum({s['a']['member'],s['b']['member']}=={m['a'],m['b']} for s in data['seams'])] for i,m in enumerate(data['declaredWelds'])])
    write_csv(root/'seam-candidates.csv',['seam','member_a','member_b','joint_type','leg_mm','start_x','start_y','start_z','end_x','end_y','end_z'],
              [[s['id'],s['a']['member'],s['b']['member'],s['joint'],s['legMm'],*s['startMm'],*s['endMm']] for s in data['seams']])
    write_csv(root/'jig-stops.csv',['stop','x_mm','y_mm','z_mm'],[[f['id'],f['x'],f['y'],f['z']] for f in data['jigFootDatumsMm']])
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="1130" viewBox="0 0 1400 1130">',
         '<rect width="1400" height="1130" fill="white"/>',
         '<style>text{font:14px sans-serif;fill:#17202a}.note{font-size:12px}</style>',
         f'<text x="30" y="28">SR6 Y{bed} — frame centreline layout / jig locations (mm), draft</text>',
         '<text x="30" y="50" class="note">Not a weld-detail or cutting drawing. See CSV dimensions, seam findings and fabrication procedure.</text>']
    for label,i,j,x,y,w,h in [('TOP X/Y',0,1,40,100,620,600),('SIDE Y/Z',1,2,730,100,620,430),('END X/Z (front and rear projected)',0,2,730,640,620,390)]:
        points=[m[k] for m in members for k in ('startMm','endMm')]
        lo=[min(p[k] for p in points) for k in (i,j)];hi=[max(p[k] for p in points) for k in (i,j)]
        scale=min((w-60)/(hi[0]-lo[0]),(h-65)/(hi[1]-lo[1]))
        def project(p):return x+25+(p[i]-lo[0])*scale,y+h-30-(p[j]-lo[1])*scale
        svg.append(f'<text x="{x}" y="{y-15}">{label}</text>')
        for m in members:
            a,b=project(m['startMm']),project(m['endMm']);color='#8899a6' if 'Brace' in m['id'] else '#1d596b'
            svg.append(f'<path d="M{a[0]:.2f},{a[1]:.2f} L{b[0]:.2f},{b[1]:.2f}" stroke="{color}" stroke-width="3" fill="none"><title>{escape(m["id"])}: {m["profile"]}, stock {m["stockLengthMm"]:.2f} mm</title></path>')
        if label.startswith('TOP'):
            for foot in data['jigFootDatumsMm']:
                px,py=project([foot['x'],foot['y'],foot['z']]);svg.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="5" fill="#b83d35"/><text x="{px+8:.2f}" y="{py-6:.2f}" class="note">{foot["id"]}</text>')
    svg.append('</svg>');(root/'layout.svg').write_text('\n'.join(svg)+'\n')
    # Preserve the authoritative native output alongside the easier-to-review views.
    (root/'native.json').write_text(source.read_text())
    (root/'provenance.json').write_text(json.dumps(dict(nativeSha256=hashlib.sha256(source.read_bytes()).hexdigest(),packetSha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),rendererSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')
    print(bed,len(members),'stock members;',len(data['declaredWelds']),'joints;',len(data['seams']),'seam candidates;',len(data['seamDiagnostics']),'findings')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--packets',type=Path,required=True);parser.add_argument('--native',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    for length in (2600,3150):render(args.native/f'y{length}.json',args.output,args.packets)


if __name__=='__main__':main()
