"""C3D10 solid-density modal fixture through the real tet ordering/deck path."""
import argparse
import importlib
import re
import shutil
import subprocess
import json
import math
from pathlib import Path
from ccx_deck import node_lines,element_lines,nset_lines,tet10_order,run_solver
from modal_fixture import frequencies
from packet import digest,elastic_input,SOLVER_UNITS
from frame_mesh import GMSH_VERSION
from solver_fixtures import CCX_VERSION
from provenance import capture_sources


def verify(gmsh,ccx,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    version=subprocess.run([str(ccx),'-v'],capture_output=True,text=True,timeout=10)
    if version.returncode not in (0,201) or not re.search(rf'Version\s+{re.escape(CCX_VERSION)}\b',version.stdout):raise ValueError('Pinned CalculiX version required')
    if gmsh.__version__!=GMSH_VERSION:raise ValueError('Pinned Gmsh version required')
    sources=capture_sources(Path(__file__).resolve().parents[4],output.resolve()/'provenance')
    material=elastic_input(210e9,0,7850)
    length,width=100,10
    area=width**2;inertia=width**4/12
    beta=1.875104068711961
    expected=beta**2/(2*math.pi*length**2)*math.sqrt(material['youngsMPa']*inertia/(material['densityTonnePerMm3']*area))
    results=[]
    for size in (4,2.5):
        directory=output/f'mesh-{size:g}';directory.mkdir(exist_ok=True)
        gmsh.initialize()
        try:
            gmsh.option.setNumber('General.NumThreads',1)
            gmsh.option.setNumber('General.Terminal',0)
            gmsh.model.add('modal-cantilever')
            gmsh.model.occ.addBox(0,0,0,length,width,width);gmsh.model.occ.synchronize()
            for key,value in [('Mesh.MeshSizeMin',size),('Mesh.MeshSizeMax',size),('Mesh.ElementOrder',2)]:gmsh.option.setNumber(key,value)
            gmsh.model.mesh.generate(3)
            tags,coords,_=gmsh.model.mesh.getNodes()
            nodes={int(n):tuple(float(v) for v in coords[i*3:i*3+3]) for i,n in enumerate(tags)}
            types,ids,conn=gmsh.model.mesh.getElements(3)
            if list(types)!=[11]:raise ValueError('C3D10 fixture requires quadratic tets')
            order=tet10_order(gmsh)
            rows=[(int(e),tuple(int(conn[0][10*i+j]) for j in order)) for i,e in enumerate(ids[0])]
            clamp=[n for n,p in nodes.items() if abs(p[0])<1e-8]
            lines=['*HEADING','Solid C3D10 modal mass and unit verification']+node_lines(nodes)+element_lines(rows,'BEAM')
            lines+=nset_lines('CLAMP',clamp)+['*MATERIAL,NAME=STEEL','*ELASTIC','210000.,0.',
                '*DENSITY',f'{material["densityTonnePerMm3"]:.12g}',
                '*SOLID SECTION,ELSET=BEAM,MATERIAL=STEEL','*BOUNDARY','CLAMP,1,3',
                '*STEP','*FREQUENCY','2','*END STEP']
            (directory/'frame.inp').write_text('\n'.join(lines)+'\n')
        finally:gmsh.finalize()
        run_solver(ccx,directory,120)
        actual=frequencies((directory/'frame.dat').read_text())
        error=max(abs(f/expected-1) for f in actual)
        if len(actual)!=2 or error>.05:raise ValueError(f'C3D10 modal beam failed: {actual}, Euler {expected}')
        results.append({'sizeMm':size,'elements':len(rows),'frequenciesHz':actual,'analyticalRelativeError':error,
            'inputSha256':digest(directory/'frame.inp'),'resultSha256':digest(directory/'frame.dat')})
    change=max(abs(a/b-1) for a,b in zip(results[-1]['frequenciesHz'],results[-2]['frequenciesHz']))
    if change>.02:raise ValueError('C3D10 modal beam did not converge within 2%')
    report={'passed':True,'units':SOLVER_UNITS,'analyticalEulerFirstBendingHz':expected,'meshRelativeChange':change,
        'beamMassKg':7850*(length*area)*1e-9,'material':material,'meshes':results,
        'sourceSha256':{name:digest(Path(__file__).with_name(name)) for name in ('modal_solid_fixture.py','ccx_deck.py','modal_fixture.py','packet.py')},
        'analysisSources':sources,'solverThreads':1,'solverVersion':version.stdout.strip(),'solverSha256':digest(ccx),'gmshVersion':gmsh.__version__,'gmshLibrarySha256':digest(Path(gmsh.libpath))}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ccx',required=True)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'build/fem/modal-solid-fixture')
    args=parser.parse_args();executable=shutil.which(args.ccx)
    if executable is None:parser.error('CalculiX executable not found')
    result=verify(importlib.import_module('gmsh'),Path(executable).resolve(),args.output)
    print(f'C3D10 modal solid fixture passed: {args.output / "report.json"}')
