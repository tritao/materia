"""Native modal energy and observed body motion; eigenvector amplitudes are arbitrary."""
from collections import defaultdict
import math
import re


GROUPS={
    'floor supports':('feet',),
    'legs/frame tubes':('frame',),
    'braces':('braces',),
    'Y rails/blocks':('Yblocks','Yguides_drive'),
    'Y drive':('Ydrive',),
    'gantry feet/welds':('feetplates','welds','bolts'),
    'beam':('gantry',),
    'X carriage/guides':('Xplate','Xblocks','Xdrive'),
    'Z ram/guides':('Zplate','Zribs','Zblocks','Zscrew','movingRails','railBolts','saddleBolts','column','flangeShelf'),
    'table mounts':('bedMounts',),
    'artificial rigid offsets':('rigid','coupling'),
}


def displacements(block):
    match=re.search(r'displacements \(vx,vy,vz\)[^\n]*\n',block)
    if not match:raise ValueError('Missing modal displacements')
    values={}
    for line in block[match.end():].splitlines():
        v=line.split()
        if len(v)==4 and v[0].isdigit():values[int(v[0])]=tuple(float(x.replace('D','E')) for x in v[1:])
        elif values:break
    return values


def energy(block,groups):
    result={}
    for group in groups:
        match=re.search(r'total internal energy.*?set\s+G'+re.escape(group)+r'\b[^\n]*\n\s*\n?\s*([+\-\d.EeDd]+)',block,re.I)
        if not match:raise ValueError('Missing native energy: '+group)
        result[group]=float(match[1].replace('D','E'))
    return result


def solve3(matrix,rhs):
    a=[list(row)+[v] for row,v in zip(matrix,rhs)]
    for i in range(3):
        pivot=max(range(i,3),key=lambda j:abs(a[j][i]));a[i],a[pivot]=a[pivot],a[i]
        if abs(a[i][i])<1e-16:return [0.,0.,0.]
        value=a[i][i];a[i]=[v/value for v in a[i]]
        for j in range(3):
            if j!=i:
                value=a[j][i];a[j]=[v-value*w for v,w in zip(a[j],a[i])]
    return [r[-1] for r in a]


def body_motion(model,u):
    byspace=defaultdict(set)
    for _,conn,row in model.elements:byspace[row['space']].update(conn)
    for root in model.roots.values():byspace[root['space']].add(root['ref'])
    result={};scale=max(math.sqrt(sum(x*x for x in u[n])) for ids in byspace.values() for n in ids)
    for space,ids in byspace.items():
        ids=sorted(ids);xyz=[model.nodes[n] for n in ids];disp=[[v/scale for v in u[n]] for n in ids]
        centre=[sum(p[i] for p in xyz)/len(ids) for i in range(3)];translation=[sum(p[i] for p in disp)/len(ids) for i in range(3)]
        offsets=[[p[i]-centre[i] for i in range(3)] for p in xyz]
        inertia=[[sum((sum(v*v for v in r) if i==j else 0)-r[i]*r[j] for r in offsets) for j in range(3)] for i in range(3)]
        rhs=[sum(r[(i+1)%3]*v[(i+2)%3]-r[(i+2)%3]*v[(i+1)%3] for r,v in zip(offsets,disp)) for i in range(3)]
        rotation=solve3(inertia,rhs)
        residual=sum((v[i]-translation[i]-(rotation[(i+1)%3]*r[(i+2)%3]-rotation[(i+2)%3]*r[(i+1)%3]))**2 for r,v in zip(offsets,disp) for i in range(3))
        rms=[math.sqrt(sum(v[i]**2 for v in disp)/len(ids)) for i in range(3)]
        result[space]=dict(translationAtCentreNormalized=translation,rotationRadPerMmMaxMotion=rotation,rmsXYZNormalized=rms,
            dominantDirection='XYZ'[max(range(3),key=lambda i:rms[i])],maxMotionNormalized=max(math.sqrt(sum(v*v for v in p)) for p in disp),
            rigidFitResidualNormalized=math.sqrt(residual/len(ids)/3),centreMm=centre)
    return result


def diagnostics(model,text,frequencies):
    blocks=re.split(r'E I G E N V A L U E\s+N U M B E R\s+\d+',text)[1:]
    if len(blocks)!=len(frequencies):raise ValueError('Missing modal energy blocks')
    rows=[]
    for index,(block,hz) in enumerate(zip(blocks,frequencies),1):
        u=displacements(block);budget=energy(block,model.groups)
        for n,m,d,k,group in model.spring_elements:budget[group]=budget.get(group,0)+.5*k*(u[n][d]-u[m][d])**2
        total=sum(budget.values())
        if total<=0 or min(budget.values())<-1e-8*total:raise ValueError('Invalid modal strain energy')
        shares={label:sum(budget.get(group,0) for group in groups)/total for label,groups in GROUPS.items()}
        if abs(sum(shares.values())-1)>1e-8:raise ValueError('Unmapped modal energy group')
        kinetic=defaultdict(float);xyz=defaultdict(lambda:[0.,0.,0.])
        for n,m,category in model.mass_loads:
            v=u[n];kinetic[category]+=m*sum(x*x for x in v)
            for i in range(3):xyz[category][i]+=m*v[i]**2
        for _,conn,row in model.elements:
            _,rho=row['material']
            if not rho:continue
            _,w,h,*walls=row['sec'];area=w*h-(w-2*walls[0])*(h-2*walls[0]);length=math.dist(model.nodes[conn[0]],model.nodes[conn[2]])
            category='y' if row['group']=='gantry' else 'frame'
            for n,weight in zip(conn,(1/6,4/6,1/6)):
                mass=area*length*rho*weight;v=u[n];kinetic[category]+=mass*sum(x*x for x in v)
                for i in range(3):xyz[category][i]+=mass*v[i]**2
        km=sum(kinetic.values());fractions={key:value/km for key,value in kinetic.items()}
        # Simpson integration is a check on native mass, not an independent consistent mass matrix.
        mass_energy=.5*(2*math.pi*hz)**2*km/1000
        error=abs(total/mass_energy-1)
        if error>.05:raise ValueError(f'Modal strain/kinetic energy mismatch: mode {index}, {error}')
        direction=[sum(v[i] for v in xyz.values())/km for i in range(3)]
        rows.append(dict(mode=index,frequencyHz=hz,fractions=fractions,gantryFraction=sum(fractions.get(k,0) for k in ('x','y','z')),
            directionalKineticFractionsXYZ=direction,dominantDirection='XYZ'[max(range(3),key=lambda i:direction[i])],
            relativeToolToBed=math.dist(u[model.query_nodes['tip']],u[model.query_nodes['bed']]),
            nativeAndSpringStrainEnergyNmm=total,strainEnergyFractions=shares,rawStrainEnergyByGroupNmm=budget,
            modalEnergyRelativeError=error,bodies=body_motion(model,u)))
    return rows
