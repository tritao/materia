"""Export the native-planner/modal response comparison using Matplotlib."""
import argparse
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def contour_error(row):
    points=[(0,0),(300,0),(300,180),(180,180),(180,100),(0,100),(0,0)]
    _,x,y,ex,ey=row;x+=ex;y+=ey;best=math.inf
    for a,b in zip(points,points[1:]):
        dx=b[0]-a[0];dy=b[1]-a[1];u=max(0,min(1,((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy)))
        best=min(best,math.hypot(x-a[0]-u*dx,y-a[1]-u*dy))
    return best*1000


def main():
    p=argparse.ArgumentParser();p.add_argument('data',type=Path);p.add_argument('output',type=Path);a=p.parse_args();data=json.loads(a.data.read_text())
    fig,axes=plt.subplots(2,2,figsize=(12,7),layout='constrained')
    for i,backend in enumerate(('toppra','ruckig')):
        for case in (r for r in data['cases'] if r['backend']==backend):
            label=('40 Hz unshaped' if case['structure']!='current' else '26 Hz '+case['shaper'])
            trace=case['contour']['trace'];axes[i,0].plot([r[0] for r in trace],[contour_error(r) for r in trace],label=label,linewidth=1)
            response=case['rapidToCut'];end=response['contourDurationS'];trace=[r for r in response['trace'] if end<=r[0]<=end+.25]
            axes[i,1].plot([r[0]-end for r in trace],[math.hypot(r[3],r[4])*1000 for r in trace],label=label,linewidth=1)
        for j in range(2):
            axes[i,j].axhline(50,color='black',linestyle='--',linewidth=.8,label='Proposed 50 µm motion allocation')
            axes[i,j].set_ylabel('Elastic motion error (µm)');axes[i,j].grid(alpha=.25)
        axes[i,0].set_title(('TOPPRA, acceleration steps' if i==0 else 'Ruckig, jerk ≤20 m/s³')+' — contour')
        axes[i,0].set_xlabel('Contour time (s)');axes[i,1].set_title('Rapid-to-cut residual');axes[i,1].set_xlabel('Time after shaper tail / transition (s)')
    axes[0,1].legend(fontsize=8);fig.suptitle('Router motion-error screen — 1% assumed damping, exact-stop contour, no servo/cutting dynamics')
    a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output.with_suffix('.svg'));fig.savefig(a.output.with_suffix('.png'),dpi=180)


if __name__=='__main__':main()
