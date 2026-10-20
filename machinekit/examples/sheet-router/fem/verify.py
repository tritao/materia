"""Independent checks for SR5 units, B32R ordering, moments and spring coupling."""
import argparse
import json
import math
from pathlib import Path
from study import Model, tables, run_solver, frequencies


def cantilever(ccx,folder,h,rotational_stiffness=None):
    model=Model(h);length=2000.;force=300.;youngs=210000.;width=100.;height=200.;wall=6.3
    model.beam((0,0,0),(length,0,0),('BOX',width,height,wall,wall,wall,wall),(0,1,0),(youngs,7850e-9),'gantry')
    root=model.root('fixed',(0,0,0),'frame');model.queries={'tip':((length,0,0),'frame')}
    if rotational_stiffness:model.spring(root,None,[1e9]*3+[1e12,1e12,rotational_stiffness],'joint')
    model.mesh()
    if not rotational_stiffness:
        r=model.roots[root];model.boundary += [(r['ref'],1,3),(r['rot'],1,3)]
    folder.mkdir(parents=True,exist_ok=True)
    (folder/'frame.inp').write_text(model.deck([('bend',{model.query_nodes['tip']:(0,force,0)},False,None)]))
    run_solver(ccx,folder,120);text=(folder/'frame.dat').read_text();u=tables(text)[(1,'U')]
    measured=u[model.query_nodes['tip']][1]
    inertia=(height*width**3-(height-2*wall)*(width-2*wall)**3)/12
    area=width*height-(width-2*wall)*(height-2*wall)
    shear=youngs/(2*1.3)
    expected=force*length**3/(3*youngs*inertia)+force*length/(shear*area*.5)
    if rotational_stiffness:expected+=force*length**2/rotational_stiffness+force/1e9
    error=abs(measured/expected-1)
    if error>.03:raise ValueError(f'Analytical beam mismatch {error}: {measured} / {expected}')
    modes=frequencies(text)
    # Euler-Bernoulli first weak-plane cantilever mode; shear lowers it slightly.
    analytic_mode=1.8751040687**2/(2*math.pi*length**2)*math.sqrt(youngs*inertia/(7850e-12*area))
    if not rotational_stiffness and abs(modes[0]/analytic_mode-1)>.04:raise ValueError('Modal beam unit/section check failed')
    return dict(meshMm=h,measuredMm=measured,analyticalMm=expected,relativeError=error,firstModeHz=modes[0],eulerBernoulliModeHz=analytic_mode)


def mill_reference(before,after):
    a=json.loads((before/'manifest.json').read_text());b=json.loads((after/'manifest.json').read_text())
    a.pop('provenance');b.pop('provenance')
    if a!=b:raise ValueError('Shared exporter changed mill physical reference packet')
    solids=[p for p in a['parts'] if p['stepFile']]
    for p in solids:
        if (before/p['stepFile']).read_bytes()!=(after/p['stepFile']).read_bytes():raise ValueError('Mill STEP changed: '+p['id'])
    return dict(components=len(a['parts']),identicalStepBodies=len(solids),method='Fresh exports before/after extraction; all manifest fields except source/build provenance and every STEP byte identical')


def main():
    p=argparse.ArgumentParser();p.add_argument('--ccx',required=True,type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--mill-before',type=Path);p.add_argument('--mill-after',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    rows=[cantilever(a.ccx.resolve(),a.output/f'beam-{h}',h) for h in [200,100]]
    rotation=cantilever(a.ccx.resolve(),a.output/'rotation',100,1e8)
    convergence=abs(rows[0]['measuredMm']/rows[1]['measuredMm']-1)
    if convergence>.01:raise ValueError('Beam mesh convergence exceeds 1%')
    result=dict(beam=rows,rotation=rotation,meshRelativeChange=convergence)
    if a.mill_before and a.mill_after:result['millReference']=mill_reference(a.mill_before,a.mill_after)
    (a.output/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
