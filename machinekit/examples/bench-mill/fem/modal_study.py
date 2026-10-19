"""Undamped, unprestressed upper-bound frame modes with mass and full inertia."""
import argparse
import json
import math
from pathlib import Path
import re
import shutil
import importlib

from ccx_deck import node_lines,equation_lines,rigid_equations,nset_lines,run_solver
from frame_solver import write_deck
from frame_mesh import xyz,GMSH_VERSION
from mesh_cache import cached_mesh
from packet import validate,digest
from modal_mass import aggregate,constellation,distributing_equations
from modal_fixture import verify,frequencies
from provenance import capture_sources
from static_study import validate_config
from modal_mass import tensor
import subprocess


def write_modal(mesh,packet,config,output,support_name,count):
    output.mkdir(parents=True,exist_ok=True)
    model=write_deck(mesh,packet,config,output,support_name)
    path=output/'frame.inp'
    prefix=path.read_text().split('*STEP\n',1)[0]
    refs=model['references'];parts={p['id']:p for p in packet['parts']}
    targets={}
    for allocation in model['gravityAllocations']:
        targets.setdefault(allocation['target'],[]).append(parts[allocation['member']])
    # Payload is a point mass at the declared COM (zero intrinsic inertia).
    zero={'xx':0,'xy':0,'xz':0,'yy':0,'yz':0,'zz':0}
    targets.setdefault('table',[]).append({'id':'study-payload','massKg':config['payloadKg'],
        'centreOfMassMm':packet['datums']['workpieceReferenceMm'],'centroidalInertiaKgMm2World':zero})
    next_node=max(list(mesh['nodes'])+[r[k] for r in refs.values() for k in ('translation','rotation')]+list(model['grounds'].values()))+1
    next_element=max(eid for rows in mesh['elements'].values() for eid,_ in rows)+len(model['grounds'])+1
    nodes={};lines=[];equations=[];records=[];graph=[]
    for index,(target,members) in enumerate(targets.items()):
        mass,centre,inertia=aggregate(members)
        if target in refs:
            ref=refs[target]
        else:
            ref={'translation':next_node,'rotation':next_node+1,'pointMm':centre};next_node+=2
            nodes[ref['translation']]=centre;nodes[ref['rotation']]=centre
            equations+=distributing_equations(mesh['nodes'],mesh['bodyNodes'][target],centre,ref['translation'],ref['rotation'])
        points=constellation(mass,centre,inertia)
        for i,(m,point) in enumerate(points):
            n=next_node;next_node+=1;nodes[n]=point
            equations+=rigid_equations(nodes,[n],ref['translation'],ref['rotation'],ref['pointMm'],(1,2,3))
            label=f'LUMP{index}_{i}'
            lines += [f'*ELEMENT,TYPE=MASS,ELSET={label}',f'{next_element},{n}',f'*MASS,ELSET={label}',f'{m/1000:.12g}'];next_element+=1
            graph.append((next_element,n,min(mesh['nodes'])));next_element+=1
        records.append({'target':target,'members':[m['id'] for m in members],'massKg':mass,
            'centreOfMassMm':centre,'centroidalInertiaKgMm2World':inertia,
            'attachment':'rigid physical ancestor' if target in refs else 'least-norm work-conjugate distributing coupling',
            'pointMasses':[{'massKg':m,'pointMm':p} for m,p in points]})
    lines+=['*ELEMENT,TYPE=SPRINGA,ELSET=MASS_GRAPH']+[f'{e},{a},{b}' for e,a,b in graph]+['*SPRING,ELSET=MASS_GRAPH','','0.']
    # A zero spring supplies the missing off-diagonal sparsity graph for MPC-
    # constrained MASS elements in CCX 2.21; contributes exactly zero energy.
    # Verified against six analytical translation/rotation modes by modal_fixture.
    sensors=sorted(set(n for member,ids in mesh['bodyNodes'].items() if member.endswith('/casting')
        for n in sorted(ids)[::max(1,len(ids)//160)]))
    sensor_groups={member:[n for n in sensors if n in set(ids)] for member,ids in mesh['bodyNodes'].items() if member.endswith('/casting')}
    obs=[r[k] for r in refs.values() for k in ('translation','rotation')]
    lines+=equation_lines(equations)+nset_lines('MODAL_OBS',obs+sensors)
    lines+=['*STEP','*FREQUENCY',str(count),'*NODE PRINT,NSET=MODAL_OBS','U','*NODE FILE','U','*END STEP']
    path.write_text(prefix+'\n'.join(node_lines(nodes)+lines)+'\n')
    return {'references':refs,'massAttachments':records,'massBudgetKg':model['massBudgetKg'],
        'sensorGroups':sensor_groups,'sensorNodes':{str(n):mesh['nodes'][n] for n in sensors},'inputSha256':digest(path),
        'supportConfiguration':support_name,'modeCount':count,
        'limitations':['Bonded interfaces and rigid axes overstate stiffness and raise frequencies.',
            'Linear unprestressed modes; no contact opening, damping, spindle spin or forced response.',
            'Catalogue joint springs are not inside this upper-bound FE model.',
            'Floor spring bounds are not joint uncertainty bounds.',
            'Mass-only hardware uses a distributing attachment over its whole represented ancestor; local mount flexibility is omitted.']}


def vectors(text):
    modes={};mode=None;active=False
    for line in text.splitlines():
        match=re.search(r'E I G E N V A L U E\s+N U M B E R\s+(\d+)',line)
        if match:mode=int(match[1]);modes[mode]={};active=False
        elif 'displacements ' in line and 'MODAL_OBS' in line:active=True
        elif active:
            row=line.split()
            if len(row)==4 and row[0].isdigit():modes[mode][int(row[0])]=[float(v.replace('D','E')) for v in row[1:]]
            elif modes[mode] and line.strip():active=False
    return modes


def assess_modal(text,model):
    values=frequencies(text);shapes=vectors(text)
    if len(values)!=model['modeCount'] or len(shapes)!=len(values) or min(values)<=.1:
        raise ValueError('Missing modes or unexplained near-rigid-body frequency')
    rows=[]
    for i,hz in enumerate(values,1):
        u=shapes[i];refs=model['references']
        relative=[u[refs['head']['translation']][j]-u[refs['table']['translation']][j] for j in range(3)]
        scale=max(math.sqrt(sum(v*v for v in u[int(n)])) for n in model['sensorNodes'])
        column=model['sensorGroups']['frame/column/casting']
        zmin=min(model['sensorNodes'][str(n)][2] for n in column)
        zmax=max(model['sensorNodes'][str(n)][2] for n in column)
        bottom=[n for n in column if model['sensorNodes'][str(n)][2]<zmin+.15*(zmax-zmin)]
        top=[n for n in column if model['sensorNodes'][str(n)][2]>zmax-.15*(zmax-zmin)]
        sensor_positions={int(n):p for n,p in model['sensorNodes'].items()}
        origin=[sum(sensor_positions[n][j] for n in bottom)/len(bottom) for j in range(3)]
        fitting=distributing_equations(sensor_positions,bottom,origin,-1,-2)
        motion=[-sum(u[n][d-1]*v for n,d,v in e[1:]) for e in fitting]
        from frame_solver import cross
        residual=[]
        for n in top:
            spin=cross(motion[3:],[sensor_positions[n][j]-origin[j] for j in range(3)])
            residual.append(math.sqrt(sum((u[n][j]-motion[j]-spin[j])**2 for j in range(3))))
        bending=max(residual)/max(scale,1e-30)
        signature=[]
        for member,ids in sorted(model['sensorGroups'].items()):
            lo=[min(sensor_positions[n][j] for n in ids) for j in range(3)]
            hi=[max(sensor_positions[n][j] for n in ids) for j in range(3)]
            for fx in (0,1):
                for fy in (0,1):
                    for fz in (0,.25,.5,.75,1):
                        target=[lo[j]+fraction*(hi[j]-lo[j]) for j,fraction in enumerate((fx,fy,fz))]
                        n=min(ids,key=lambda n:math.dist(sensor_positions[n],target))
                        signature.extend(u[n])
        rows.append({'shapeSignature':signature,'columnBendingResidualOverMaxCastingMotion':bending,'columnBendingProxy':bending>.1,'mode':i,'frequencyHz':hz,'relativeContactVector':relative,
            'relativeContactNormOverMaxCastingMotion':math.sqrt(sum(v*v for v in relative))/max(scale,1e-30),
            'sensorDisplacements':{n:u[int(n)] for n in model['sensorNodes']}})
    return rows


def modal_tables(text):
    result={}
    for key,title in [('participation','P A R T I C I P A T I O N   F A C T O R S'),('effectiveModalMass','E F F E C T I V E   M O D A L   M A S S')]:
        section=text.split(title,1)
        if len(section)!=2:raise ValueError('Missing modal participation/mass table')
        rows=[]
        for line in section[1].splitlines():
            raw=line.split()
            if len(raw)==7 and raw[0].isdigit():rows.append([float(v.replace('D','E')) for v in raw[1:]])
            elif rows and line.strip():break
        result[key]=rows
    return result


def shape_svg(model,modes,path):
    # Two orthographic views, normalized independently per mode; visual shape only.
    points=model['sensorNodes'];height=250*len(modes)
    lines=[f'<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="{height}" viewBox="0 0 1000 {height}">',
           '<rect width="100%" height="100%" fill="white"/>']
    for row,mode in enumerate(modes):
        vectors=mode['sensorDisplacements'];peak=max(math.sqrt(sum(v*v for v in u)) for u in vectors.values());gain=50/max(peak,1e-30)
        lines.append(f'<text x="10" y="{row*250+20}" font-size="14">Mode {mode["mode"]}: {mode["frequencyHz"]:.3f} Hz; XZ / YZ; displacement exaggerated</text>')
        for axis in (0,1):
            for n,p in points.items():
                x=axis*500+250+.18*p[axis];y=row*250+240-.18*p[2]
                u=vectors[n];dx=gain*u[axis];dy=-gain*u[2]
                lines.append(f'<line x1="{x:.3f}" y1="{y:.3f}" x2="{x+dx:.3f}" y2="{y+dy:.3f}" stroke="#1769aa" stroke-width="1"/>')
    path.write_text('\n'.join(lines+['</svg>']))


def markdown(report):
    lines=['# Undamped upper-bound frame modes','',report['status'],'',
        'Bonded interfaces and rigid axes raise frequencies. These are unprestressed modes, not forced-response amplitudes.',
        '',f'Frequency-order mesh change: {report["frequencyOrderRelativeChange"]}; converged: {report["converged"]}.',
        '',f'Pose (mm): {report["jointsMm"]}.', '',f'Manifest SHA256: {report["manifestSha256"]}.',
        '',f'Configuration SHA256: {report["configurationSha256"]}.','']
    for mesh in report['meshes']:
        for name,result in mesh['supports'].items():
            lines += [f'## {mesh["sizeMm"]:g} mm / {name}','',
                '| Mode | Frequency Hz | Column-bending proxy | Relative contact / max casting motion | Effective mass X/Y/Z kg |',
                '| --- | --- | --- | --- | --- |']
            for mode,mass in zip(result['modes'],result['participation']['effectiveModalMass']):
                masses='/'.join(f'{1000*v:.3f}' for v in mass[:3])
                lines.append(f'| {mode["mode"]} | {mode["frequencyHz"]:.4f} | {mode["columnBendingResidualOverMaxCastingMotion"]:.4f} | {mode["relativeContactNormOverMaxCastingMotion"]:.4f} | {masses} |')
            lines+=['',f'[Exaggerated mode shapes](mesh-{mesh["sizeMm"]:g}/{name}/mode-shapes.svg)','']
    lines+=['',report['modeTrackingNote'],'','The column-bending proxy removes a best-fit rigid motion at the column bottom, then measures top-section residual motion. Review shapes and crossings rather than treating this proxy as an acceptance limit.',
        '', 'Exact deck/result/source hashes, mass/inertia attachments, full vectors and participation tables are in report.json.']
    return '\n'.join(lines)+'\n'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest',type=Path);parser.add_argument('--ccx',required=True)
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('frame-study.json'))
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--sizes',nargs='+',type=float,default=[40,30])
    parser.add_argument('--modes',type=int,default=12);parser.add_argument('--timeout',type=int,default=1800)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'report.json').unlink(missing_ok=True)
    original=args.manifest.resolve();input_hash=digest(original)
    snapshot=args.output.resolve()/f'packet-{input_hash[:16]}'
    if not snapshot.exists():shutil.copytree(original.parent,snapshot)
    args.manifest=snapshot/'manifest.json'
    if digest(args.manifest)!=input_hash:raise ValueError('Packet changed during freeze')
    validated=validate(args.manifest);packet=json.loads(args.manifest.read_text());config=json.loads(args.config.read_text());validate_config(config)
    for part in packet['parts']:
        constellation(part['massKg'],xyz(part['centreOfMassMm']),tensor(part))
        if not part.get('inertiaSource'):raise ValueError('Missing inertia source')
    if len(args.sizes)<2 or any(s<=0 or not math.isfinite(s) for s in args.sizes) or any(a<=b for a,b in zip(args.sizes,args.sizes[1:])):
        raise ValueError('At least two decreasing positive mesh sizes required')
    ccx=Path(shutil.which(args.ccx)).resolve()
    version=subprocess.run([str(ccx),'-v'],capture_output=True,text=True,timeout=10)
    if version.returncode not in (0,201) or not re.search(r'Version\s+2\.21\b',version.stdout):raise ValueError('CalculiX 2.21 required')
    fixture=verify(ccx,args.output/'fixture')
    source_names=tuple(sorted(p.name for p in Path(__file__).parent.glob('*.py') if not p.name.startswith('test_')))
    hashes={n:digest(Path(__file__).with_name(n)) for n in source_names}
    input_hash=digest(args.manifest);config_hash=digest(args.config)
    sources=capture_sources(Path(__file__).resolve().parents[4],args.output.resolve()/'provenance')
    gmsh=importlib.import_module('gmsh')
    if gmsh.__version__!=GMSH_VERSION:raise ValueError('Pinned Gmsh version required')
    results=[]
    for size in args.sizes:
        output=args.output/f'mesh-{size:g}';output.mkdir(exist_ok=True)
        gmsh.initialize();gmsh.logger.start()
        try:mesh=cached_mesh(gmsh,packet,args.manifest,config,size,output)
        finally:
            messages=gmsh.logger.get();(output/'gmsh.log').write_text('\n'.join(messages)+'\n')
            gmsh.logger.stop();gmsh.finalize()
        if not mesh['throughThickness']['allSectionsResolved']:raise ValueError('Unresolved thin sections')
        supports={}
        for name in config['supportConfigurations']:
            directory=output/name;model=write_modal(mesh,packet,config,directory,name,args.modes)
            run_solver(ccx,directory,args.timeout)
            text=(directory/'frame.dat').read_text();modes=assess_modal(text,model)
            shape_svg(model,modes,directory/'mode-shapes.svg')
            supports[name]={'model':model,'modes':modes,'participation':modal_tables(text),'shapePlot':'mode-shapes.svg','resultSha256':digest(directory/'frame.dat')}
            print(f'{size:g} {name}: {[m["frequencyHz"] for m in modes]}',flush=True)
        results.append({'sizeMm':size,'supports':supports,'meshSha256':mesh['meshSha256'],'minimumJacobianDeterminant':mesh['minimumJacobianDeterminant'],'gmshLibrarySha256':mesh['gmshLibrarySha256'],'gmshWarnings':[line for line in messages if 'Warning' in line or 'Error' in line],'throughThickness':mesh['throughThickness']})
    change=None
    if len(results)>=2:
        change=max(abs(a['frequencyHz']/b['frequencyHz']-1) for name in config['supportConfigurations']
            for a,b in zip(results[-1]['supports'][name]['modes'],results[-2]['supports'][name]['modes']))
    if input_hash!=digest(args.manifest) or config_hash!=digest(args.config) or hashes!={n:digest(Path(__file__).with_name(n)) for n in source_names}:raise ValueError('Modal sources changed during solve')
    report={'status':'Undamped upper-bound modes; acceptance against excitation bands remains open',
        'converged':change is not None and change<=.05,'frequencyOrderRelativeChange':change,
        'modeTrackingNote':'Frequency-order convergence only; crossing/degenerate modes require subspace/MAC review across poses.',
        'analysisSources':sources,'sourceSha256':hashes,'manifestSha256':validated['manifestSha256'],
        'packetDensityMassChecks':validated['densityMassChecks'],'frozenManifestPath':str(args.manifest),'geometrySourceBuild':packet['provenance']['sourceBuild'],'configurationSha256':digest(args.config),
        'solverVersion':version.stdout.strip(),'gmshVersion':gmsh.__version__,'solverSha256':digest(ccx),'fixture':fixture,'jointsMm':packet['jointsMm'],'meshes':results}
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.output/'report.md').write_text(markdown(report))
    return 0 if report['converged'] else 2

if __name__=='__main__':raise SystemExit(main())
