"""SR6 native prepared-contact reactions, static envelopes and modal fatigue screen."""
import argparse,csv,hashlib,itertools,json,math,re,sys
from collections import defaultdict
from pathlib import Path
# The existing isolated environment supplies Gmsh; system Python supplies NumPy.
import numpy as np
sys.path.append(str(Path(__file__).resolve().parents[2]/'bench-mill/build/fem/venv/lib/python3.12/site-packages'))
from study import make_model,run_solver,tables,frequencies
from weld_model import WeldModel,centre_and_inertia,with_foot_welds,ExactOffsetModel
from modes import displacements
from modal_mass import constellation
from weld_effective import effective_stitches


def read_stitches(folder):
    return [dict(group=r['group'],seam=r['seam'],a=[float(r['start_'+a]) for a in 'xyz'],b=[float(r['end_'+a]) for a in 'xyz'],
        lengthMm=float(r['length_mm']),legMm=float(r['leg_mm']),fullSizeEnds=r.get('full_size_ends','false').lower()=='true') for r in csv.DictReader((folder/'weld-stitches.csv').open())]


def contacts(native,stitches):
    seams={s['id']:s for s in native['seams']};groups=defaultdict(list)
    for s in stitches:
        seam=seams[s['seam']];pair=tuple(sorted((seam['a']['member'],seam['b']['member'])))
        groups[(s['group'],pair)].append(s)
    result=[]
    for (group,pair),lines in sorted(groups.items()):
        length,c,J=centre_and_inertia(lines);J=np.array(J);inverse=np.linalg.pinv(J,rcond=1e-10)
        transforms=[]
        for line in lines:
            for end in ('a','b'):
                r=np.array(line[end])-c
                cross=np.array([[0,-r[2],r[1]],[r[2],0,-r[0]],[-r[1],r[0],0]])
                transforms.append(np.hstack((np.eye(3)/length,-cross@inverse))/(line.get('legMm',5)/math.sqrt(2)))
        result.append(dict(group=group,pieces=pair,lengthMm=length,legMm=lines[0].get('legMm',5),centre=c,J=J.tolist(),transform=np.vstack(transforms)))
    return result


def layout_signature(layout):
    rows=[dict(group=c['group'],pieces=c['pieces'],legMm=c['legMm'],lengthMm=c['lengthMm'],centre=c['centre'],J=c['J']) for c in layout]
    return hashlib.sha256(json.dumps(rows,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def wrench(model,u,layout):
    values=model.reactions(u);result=[]
    for c in layout:
        samples=[r for r in values[c['group']] if tuple(r['pieces'])==c['pieces']]
        force=np.sum([r['forceN'] for r in samples],axis=0)
        moment=np.sum([np.cross(np.array(r['at'])-c['centre'],r['forceN']) for r in samples],axis=0)
        # A singular weld line is allowed only when the actual recovered moment
        # lies in its resisting subspace. No invented rotational capacity.
        J=np.array(c['J']);unresisted=moment-J@np.linalg.pinv(J,rcond=1e-10)@moment
        result.append(np.r_[force,moment])
        if np.linalg.norm(unresisted)>.02+1e-4*np.linalg.norm(moment):raise ValueError(f'Unresisted recovered moment at {c["group"]}: {unresisted}')
    return np.array(result)


def fixture(ccx,folder):
    m=ExactOffsetModel(100);c=[0.,0.,0.];J=[[100000.,0,0],[0,100000.,0],[0,0,200000.]];ports=[]
    base=m.root('base',c,'base');moving=m.root('moving',c,'moving')
    for i,(weight,p) in enumerate(constellation(80,c,J)):
        roots=[]
        for space in ('base','moving'):
            m.rigid(c,p,space);roots.append(m.root(space+str(i),p,space))
        k=50000*weight;m.spring(*roots,[k,k,k,0,0,0],'weld');ports.append((roots,p,k))
    m.queries={'tip':(tuple(c),'moving')};m.mesh();r=m.roots[base];m.boundary=[(r['ref'],1,3),(r['rot'],1,3)]
    force=np.array([300.,400.,500.]);moment=np.array([10000.,20000.,30000.]);target=m.roots[moving]
    loads=[('wrench',{target['ref']:force,target['rot']:moment},False,None)]
    folder.mkdir(parents=True,exist_ok=True);(folder/'frame.inp').write_text(m.deck(loads,modal=False));run_solver(ccx,folder,180)
    u=tables((folder/'frame.dat').read_text())[(1,'U')];F=np.zeros(3);M=np.zeros(3)
    for roots,p,k in ports:
        a,b=[m.roots[x]['ref'] for x in roots];f=k*(np.array(u[a])-u[b]);F+=f;M+=np.cross(p,f)
    errorF=float(np.linalg.norm(F+force)/np.linalg.norm(force));errorM=float(np.linalg.norm(M+moment)/np.linalg.norm(moment))
    if max(errorF,errorM)>.001:raise ValueError('Native weld wrench fixture failed')
    result=dict(forceRelativeError=errorF,momentRelativeError=errorM,recoveredForceN=F.tolist(),recoveredMomentNmm=M.tolist())
    (folder/'verification.json').write_text(json.dumps(result,indent=2)+'\n');return result


def run_case(packet,native,stitches,pose,h,ccx,folder,mode_count=0,factor=1.,z_only=False):
    packet=dict(packet,modeCount=mode_count or 20)
    model=make_model(packet,pose,h,WeldModel(h,packet,native,stitches,factor));tip=model.query_nodes['tip'];bed=model.query_nodes['bed']
    loads=[]
    for i,name in enumerate('XYZ'):
        f=np.eye(3)[i]*300;loads.append(('cut'+name,{tip:f,bed:-f},False,None))
    loads += [('weight',{},True,None),('accelX',{},False,(0,('x','z'))),('accelY',{},False,(1,('x','y','z'))),('accelZ',{},False,(2,('z',)))]
    if z_only:loads=loads[-1:];mode_count=0
    folder.mkdir(parents=True,exist_ok=True);deck=model.deck(loads,modal=mode_count>0)
    cached=(folder/'frame.inp').exists() and (folder/'frame.inp').read_text()==deck and (folder/'frame.dat').exists() and (folder/'solver.log').exists()
    if cached:
        log=(folder/'solver.log').read_text();cached='Job finished' in log and '*ERROR' not in log.upper()
    if not cached:(folder/'frame.inp').write_text(deck);run_solver(ccx,folder,600)
    text=(folder/'frame.dat').read_text();tab=tables(text);layout=contacts(native,stitches);fields=[];basis=[];summaries=[]
    for step,(name,forces,gravity,acceleration) in enumerate(loads,1):
        u=tab[(step,'U')];fields.append(wrench(model,u,layout));net=np.zeros(3)
        for i in range(len(packet['supports'])):
            support_name=model.roots.aliases['foot'+str(i)];root=model.roots[support_name];ks=next(k for a,b,k,g in model.springs if a==support_name and b is None)
            net-=np.array(ks[:3])*u[root['ref']]
        expected=np.zeros(3)
        for f in forces.values():expected-=f
        for n,m,carrier in model.mass_loads:
            if gravity:expected[2]+=m*9.80665
            if acceleration and carrier in acceleration[1]:expected[acceleration[0]]+=3*m
        for _,conn,row in model.elements:
            _,rho=row['material']
            if not rho:continue
            _,width,height,*walls=row['sec'];area=width*height-(width-2*walls[0])*(height-2*walls[0]) if walls else width*height
            mass=area*math.dist(model.nodes[conn[0]],model.nodes[conn[2]])*rho
            if gravity:expected[2]+=mass*9.80665
            if acceleration and ((row['group']=='gantry' and 'y' in acceleration[1]) or (row['group']=='column' and 'x' in acceleration[1])):expected[acceleration[0]]+=3*mass
        error=float(np.linalg.norm(net-expected)/max(300,np.linalg.norm(expected)))
        if error>.002:raise ValueError(f'Global reaction balance failed {name}: {error}: {net} / {expected}')
        from modes import energy
        block=re.split(r'S T E P\s+\d+',text)[step];budget=energy(block,model.groups)
        artificial=budget.get('coupling',0);total=sum(budget.values())
        for n,m,i,k,group in model.spring_elements:
            e=.5*k*(u[n][i]-u[m][i])**2;total+=e
            if group=='loadBridges':artificial+=e
        fraction=abs(artificial)/total
        if fraction>.005:raise ValueError(f'Artificial offset energy too large {name}: {fraction}')
        summaries.append(dict(name=name,supportBalanceError=error,artificialOffsetEnergyFraction=fraction,tipRelativeMm=(np.array(u[tip])-u[bed]).tolist()))
    if mode_count:
        masses=list(model.mass_loads)
        for _,conn,row in model.elements:
            _,rho=row['material']
            if not rho:continue
            _,w,hg,*walls=row['sec'];area=w*hg-(w-2*walls[0])*(hg-2*walls[0]) if walls else w*hg
            mass=area*math.dist(model.nodes[conn[0]],model.nodes[conn[2]])*rho
            for n,weight in zip(conn,(1/6,4/6,1/6)):masses.append((n,mass*weight,'y' if row['group']=='gantry' else 'frame'))
        hz=frequencies(text);blocks=re.split(r'E I G E N V A L U E\s+N U M B E R\s+\d+',text)[1:]
        for f,block in zip(hz,blocks):
            u=displacements(block);mass=sum(m*sum(v*v for v in u[n]) for n,m,_ in masses)
            gamma=[sum(m*u[n][i] for n,m,carrier in masses if carrier in (('x','z') if i==0 else ('x','y','z')))/mass for i in range(2)]
            from modes import energy
            strain=sum(energy(block,model.groups).values())
            strain+=sum(.5*k*(u[n][i]-u[m][i])**2 for n,m,i,k,_ in model.spring_elements)
            energy_error=abs(strain/(.5*(2*math.pi*f)**2*mass/1000)-1)
            if energy_error>.05:raise ValueError(f'Weld-mode mass/strain energy mismatch {f}: {energy_error}')
            basis.append(dict(frequencyHz=f,gammaXY=gamma,wrench=wrench(model,u,layout).tolist(),modalEnergyRelativeError=energy_error,
                toolRelativeEigenvectorXY=[u[tip][i]-u[bed][i] for i in range(2)]))
    result=dict(pose={k:pose[k] for k in ('x','y','z')},meshMm=h,weldStiffnessFactor=factor,cases=summaries,staticWrenches=np.array(fields).tolist(),modes=basis,
        jointLayoutSha256=layout_signature(layout),solverDeckSha256=hashlib.sha256(deck.encode()).hexdigest(),
        contactOrder=[dict(group=c['group'],pieces=c['pieces'],centre=c['centre']) for c in layout])
    (folder/'reactions.json').write_text(json.dumps(result,separators=(',',':'))+'\n')
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--packets',type=Path,required=True);p.add_argument('--fabrication',type=Path,required=True);p.add_argument('--ccx',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--bed',type=int,choices=(2600,3150));p.add_argument('--trial',action='store_true');p.add_argument('--z-only',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    if not a.trial:print('fixture',fixture(a.ccx.resolve(),a.output/'fixture'),flush=True)
    rows=[]
    for bed in ([a.bed] if a.bed else (3150,2600)):
        packet=json.loads((a.packets/f'y{bed}/manifest.json').read_text());native=json.loads((a.fabrication/f'y{bed}/native.json').read_text());stitches=read_stitches(a.fabrication/f'y{bed}')
        priority=[(800,0 if bed==2600 else bed),(800,bed/2),(1600,0)]
        poses=sorted(packet['poses'],key=lambda q:priority.index((q['x'],q['y'])) if (q['x'],q['y']) in priority else 3)
        if a.trial:poses=poses[:3]
        for pose in poses:
            joined,physical=with_foot_welds(packet,native,stitches,pose);effective,rejected=effective_stitches(joined,physical)
            name=f'y{bed}-x{pose["x"]:g}-y{pose["y"]:g}';modal=40 if pose['x']==800 and pose['y']==(0 if bed==2600 else bed) else 0
            result=run_case(packet,joined,effective,pose,50,a.ccx.resolve(),a.output/name,modal,z_only=a.z_only)
            row=dict(workY=bed,path=name,**{k:result[k] for k in ('pose','meshMm','cases')});rows.append(row)
            (a.output/'index.json').write_text(json.dumps(rows,indent=2)+'\n');print(bed,row['pose'],'Z inertia' if a.z_only else 'XY',[] if a.z_only else [.3/result['cases'][i]['tipRelativeMm'][i] for i in (0,1)],'modes',len(result['modes']),flush=True)

if __name__=='__main__':main()
