"""Six analytical modes verify point inertia and distributing coupling in CalculiX."""
import math
import re
from pathlib import Path
from ccx_deck import node_lines,equation_lines,rigid_equations,run_solver
from modal_mass import constellation,distributing_equations


def frequencies(text):
    active=False;rows=[]
    for line in text.splitlines():
        if 'E I G E N V A L U E' in line: active=True;continue
        if active:
            values=line.split()
            if len(values)>=4 and values[0].isdigit():
                try: rows.append(float(values[3].replace('D','E')))
                except ValueError: pass
            elif rows and line.strip():break
    if not rows: raise ValueError('No modal frequencies in CalculiX output')
    return rows


def verify(ccx,output,threads=1):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    nodes={1:(-10,-20,0),2:(10,-20,0),3:(10,20,0),4:(-10,20,0),9:(0,0,0),10:(0,0,0)}
    for n in range(1,5): nodes[n+4]=nodes[n]
    equations=distributing_equations(nodes,range(1,5),(0,0,0),9,10)
    mass=12;inertia=[[800,0,0],[0,600,0],[0,0,1000]]
    points=constellation(mass,[0,0,0],inertia)
    for i,(_,point) in enumerate(points):
        n=11+i;nodes[n]=point
        equations+=rigid_equations(nodes,[n],9,10,(0,0,0),(1,2,3))
    lines=['*HEADING','Modal mass and distributing coupling fixture']+node_lines(nodes)
    eid=1
    for dof in range(1,4):
        lines += [f'*ELEMENT,TYPE=SPRING2,ELSET=SPR{dof}']
        for n in range(1,5):lines.append(f'{eid},{n},{n+4}');eid+=1
        lines += [f'*SPRING,ELSET=SPR{dof}',f'{dof},{dof}','1000.']
    for n in range(1,5):
        lines += [f'*ELEMENT,TYPE=MASS,ELSET=P{n}',f'{eid},{n}',f'*MASS,ELSET=P{n}','0.0001'];eid+=1
    for i,(m,_) in enumerate(points):
        lines += [f'*ELEMENT,TYPE=MASS,ELSET=M{i}',f'{eid},{11+i}',f'*MASS,ELSET=M{i}',f'{m/1000:.12g}'];eid+=1
    # Zero-stiffness graph elements expose all induced off-diagonal MASS terms
    # to CalculiX's sparse-pattern builder after MPC elimination.
    lines+=['*ELEMENT,TYPE=SPRINGA,ELSET=GRAPH']
    for i in range(6):
        lines.append(f'{eid},{11+i},1');eid+=1
    lines+=['*SPRING,ELSET=GRAPH','','0.','*NSET,NSET=ALL','1,2,3,4,9,10']
    lines+=equation_lines(equations)+['*BOUNDARY','5,1,3','6,1,3','7,1,3','8,1,3','*STEP','*FREQUENCY','6','*NODE PRINT,NSET=ALL','U','*NODE FILE','U','*END STEP']
    (output/'frame.inp').write_text('\n'.join(lines)+'\n')
    run_solver(ccx,output,120,threads)
    actual=frequencies((output/'frame.dat').read_text())
    expected=sorted(math.sqrt(k/(m/1000))/(2*math.pi) for k,m in zip([4000]*3+[1600000,400000,2000000],[mass+.4]*3+[960,640,1200]))
    error=max(abs(a/b-1) for a,b in zip(actual,expected))
    if len(actual)!=6 or error>1e-4:raise ValueError(f'Modal fixture failed: {actual}, expected {expected}')
    return {'frequenciesHz':actual,'analyticalHz':expected,'maxRelativeError':error,'passed':True}
