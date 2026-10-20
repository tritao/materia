"""SR5 linear beam/grillage screen, mm/N/tonne/s. No contact or stress qualification."""
import argparse
import gzip
import shutil
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import re
import sys

MILL = Path(__file__).resolve().parents[2] / 'bench-mill' / 'fem'
sys.path.insert(0, str(MILL))
from ccx_deck import node_lines, nset_lines, equation_lines, rigid_equations, run_solver
from modal_mass import constellation
from modal_fixture import frequencies
import gmsh


def add(a,b): return tuple(x+y for x,y in zip(a,b))
def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def mul(a,s): return tuple(x*s for x in a)
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def norm(a): return math.sqrt(dot(a,a))
def project(p,a,b):
    d=sub(b,a);t=max(0,min(1,dot(sub(p,a),d)/dot(d,d)))
    return add(a,mul(d,t))
def section(row): return ('BOX',row['widthMm'],row['heightMm'],*[row['wallMm']]*4)
def point(p): return (p['x'],p['y'],p['z'])


class Model:
    def __init__(self,h):
        self.h=h;self.lines=[];self.roots={};self.springs=[];self.masses=[];self.boundary=[]
        self.native=[];self.queries={};self.parts=[];self.mode_count=20
    def beam(self,a,b,sec,normal=(0,0,1),material=(210000,0),group='rigid',space='frame'):
        if norm(sub(a,b))<1e-6:return
        d=mul(sub(b,a),1/norm(sub(b,a)))
        if abs(dot(d,normal))>.99:normal=(0,1,0) if abs(d[1])<.9 else (1,0,0)
        self.lines.append(dict(a=tuple(a),b=tuple(b),sec=sec,normal=normal,material=material,group=group,space=space))
    def rigid(self,a,b,space):
        self.beam(a,b,('RECT',100,100),material=(2100000000,0),space=space)
    def root(self,name,p,space):
        for existing,root in self.roots.items():
            if root['space']==space and norm(sub(p,root['p']))<1e-6:return existing
        if name not in self.roots:
            self.roots[name]=dict(p=tuple(p),space=space)
            # Two non-collinear arms make the beam end's rotation observable.
            for d in [(10,10,0),(-10,10,10)]:
                self.beam(p,add(p,d),('RECT',100,100),normal=(0,1,0),material=(2100000000,0),group='coupling',space=space)
        return name
    def spring(self,a,b,k,group): self.springs.append((a,b,tuple(k),group))
    def mesh(self):
        gmsh.initialize();gmsh.option.setNumber('General.Terminal',0);gmsh.model.add('router')
        tags={};curves=[];segments={}
        def ptag(p,space):
            key=(space,tuple(round(x,6) for x in p))
            if key not in tags:tags[key]=gmsh.model.geo.addPoint(*p)
            return tags[key]
        # Explicitly split stock at all welded branch and load attachment points.
        endpoints=defaultdict(list)
        for row in self.lines:
            endpoints[row['space']]+=[row['a'],row['b']]
        # Connect interior intersections of welded frame stock, not moving bodies or rigid arms.
        stock=[row for row in self.lines if row['group'] in ('frame','braces') and row['space']=='frame']
        for i,first in enumerate(stock):
            for second in stock[i+1:]:
                a,b=first['a'],second['a'];u=sub(first['b'],a);v=sub(second['b'],b);w=sub(a,b)
                uu,vv,uv=dot(u,u),dot(v,v),dot(u,v);den=uu*vv-uv*uv
                if den<=1e-12*uu*vv:continue
                t=(uv*dot(v,w)-vv*dot(u,w))/den
                q=(uu*dot(v,w)-uv*dot(u,w))/den
                if not (-1e-8<=t<=1+1e-8 and -1e-8<=q<=1+1e-8):continue
                p=add(a,mul(u,t))
                if norm(sub(p,add(b,mul(v,q))))<1e-5:endpoints['frame'].append(p)
        for row in self.lines:
            a,b=row['a'],row['b'];d=sub(b,a);l=norm(d)
            cuts={0.0,1.0}
            for p in endpoints[row['space']]:
                t=dot(sub(p,a),d)/(l*l)
                if 1e-8<t<1-1e-8 and norm(sub(p,add(a,mul(d,t))))<1e-5:cuts.add(t)
            merged=[]
            for t in sorted(cuts):
                if not merged or (t-merged[-1])*l>1e-5:merged.append(t)
            cuts=merged
            for t,u in zip(cuts,cuts[1:]):
                start=add(a,mul(d,t));end=add(a,mul(d,u))
                key=(row['space'],tuple(sorted((tuple(round(x,6) for x in start),tuple(round(x,6) for x in end)))))
                if key in segments:
                    previous=segments[key]
                    if row['group']!=previous['group'] or row['sec']!=previous['sec']:raise ValueError(f'Overlapping analysis members: {key}: {row["group"]}/{previous["group"]}')
                    continue
                segments[key]=row
                c=gmsh.model.geo.addLine(ptag(start,row['space']),ptag(end,row['space']))
                curves.append((c,row));gmsh.model.geo.mesh.setTransfiniteCurve(c,max(2,math.ceil(norm(sub(end,start))/(self.h/4 if row['group'] in ('Xplate','Zplate','feetplates') else self.h))+1))
        gmsh.model.geo.synchronize();gmsh.model.mesh.generate(1);gmsh.model.mesh.setOrder(2)
        ids,coords,_=gmsh.model.mesh.getNodes();nodes={int(n):tuple(float(x) for x in coords[3*i:3*i+3]) for i,n in enumerate(ids)}
        self.elements=[];self.groups=defaultdict(list)
        for c,row in curves:
            types,eids,conns=gmsh.model.mesh.getElements(1,c)
            for typ,es,cs in zip(types,eids,conns):
                if typ!=8:raise ValueError('Expected Gmsh quadratic line type 8')
                for i,eid in enumerate(es):
                    # Gmsh: start,end,mid; CalculiX: start,mid,end.
                    conn=[int(cs[3*i+j]) for j in (0,2,1)]
                    eid=len(self.elements)+1;self.elements.append((eid,conn,row));self.groups[row['group']].append(eid)
        def node(p,space):
            tag=tags[(space,tuple(round(x,6) for x in p))]
            ids,_,_=gmsh.model.mesh.getNodes(0,tag,includeBoundary=True)
            if len(ids)!=1:raise ValueError('Missing geometry point node')
            return int(ids[0])
        for root in self.roots.values():
            root['anchor']=node(root['p'],root['space'])
            root['members']=[root['anchor']]
            for d in [(10,10,0),(-10,10,10)]:
                q=add(root['p'],d)
                for eid,conn,row in self.elements:
                    if row['group']=='coupling' and row['space']==root['space'] and (row['a']==root['p'] and row['b']==q):root['members']+=conn
            root['members']=sorted(set(root['members']))
        self.query_nodes={name:node(p,space) for name,(p,space) in self.queries.items()}
        gmsh.finalize();self.nodes=nodes
        self.next_node=max(nodes)+1
        for root in self.roots.values():
            root['ref']=self.newnode(root['p']);root['rot']=self.newnode((0,0,0))
    def newnode(self,p):
        n=self.next_node;self.next_node+=1;self.nodes[n]=tuple(p);return n
    def deck(self,loads,modal=True):
        lines=['*HEADING','Sheet router SR5 reduced structural screen'];after=[];eid=len(self.elements)+1
        materials={row['material'] for _,_,row in self.elements}
        for i,(e,rho) in enumerate(sorted(materials)):
            lines += [f'*MATERIAL,NAME=M{i}','*ELASTIC',f'{e},0.3']
            if rho:lines+=['*DENSITY',f'{rho/1000:.12g}']
        matids={v:i for i,v in enumerate(sorted(materials))}
        for n,conn,row in self.elements:
            after += [f'*ELEMENT,TYPE=B32R,ELSET=E{n}',f'{n},'+','.join(map(str,conn)),
                f'*BEAM SECTION,ELSET=E{n},MATERIAL=M{matids[row["material"]]},SECTION={row["sec"][0]}',
                ','.join(str(x) for x in row['sec'][1:]),','.join(str(x) for x in row['normal'])]
        for name,root in self.roots.items():
            after+=nset_lines('R'+name,root['members'])+[f'*RIGID BODY,NSET=R{name},REF NODE={root["ref"]},ROT NODE={root["rot"]}']
        self.spring_elements=[]
        for a,b,ks,group in self.springs:
            ar,br=self.roots[a],self.roots[b] if b else None
            for i,k in enumerate(ks):
                if k<=0:continue
                n=ar['ref'] if i<3 else ar['rot'];m=(br['ref'] if i<3 else br['rot']) if br else self.newnode(self.nodes[n])
                if br is None:self.boundary.append((m,1,3))
                label='S'+str(eid);after += [f'*ELEMENT,TYPE=SPRING2,ELSET={label}',f'{eid},{n},{m}',f'*SPRING,ELSET={label}',f'{i%3+1},{i%3+1}',f'{float(k):.12f}']
                self.spring_elements.append((n,m,i%3,k,group));eid+=1
        equations=[];self.mass_loads=[]
        grouped=defaultdict(list)
        for entry in self.masses:grouped[(entry[0],entry[4])].append(entry)
        condensed=[]
        for (rootname,moving),entries in grouped.items():
            kg=sum(e[1] for e in entries)
            centre=[sum(e[1]*e[2][i] for e in entries)/kg for i in range(3)]
            inertia=[[0.]*3 for _ in range(3)]
            for _,m,c,t,_ in entries:
                r=sub(c,centre)
                for i in range(3):
                    for j in range(3):inertia[i][j]+=t[i][j]+m*((dot(r,r) if i==j else 0)-r[i]*r[j])
            condensed.append((rootname,kg,centre,inertia,moving))
        for rootname,kg,centre,inertia,moving in condensed:
            root=self.roots[rootname]
            for m,p in constellation(kg,list(centre),inertia):
                n=self.newnode(p);after += [f'*ELEMENT,TYPE=MASS,ELSET=MASS{eid}',f'{eid},{n}',f'*MASS,ELSET=MASS{eid}',f'{m/1000:.12g}'];eid+=1
                equations+=rigid_equations(self.nodes,[n],root['ref'],root['rot'],root['p'],(1,2,3))
                after += [f'*ELEMENT,TYPE=SPRINGA,ELSET=GRAPH{eid}',f'{eid},{n},{root["ref"]}',f'*SPRING,ELSET=GRAPH{eid}','','0.'];eid+=1
                self.mass_loads.append((n,m,moving))
        after+=equation_lines(equations)
        for group,ids in self.groups.items():after += ['*ELSET,ELSET=G'+group]+[','.join(map(str,ids[i:i+12])) for i in range(0,len(ids),12)]
        selected=list(self.nodes)
        for root in self.roots.values():selected += [root['ref'],root['rot']]
        selected += [n for n,_,_,_,_ in self.spring_elements]+[m for _,m,_,_,_ in self.spring_elements]
        after+=nset_lines('OBS',set(selected))+['*BOUNDARY']+[f'{n},{a},{b}' for n,a,b in self.boundary]
        for name,forces,gravity,acceleration in loads:
            after += ['*STEP','*STATIC','*CLOAD,OP=NEW',f'{self.query_nodes["tip"]},1,0.']
            after += [f'{n},{i+1},{v:.12g}' for n,f in forces.items() for i,v in enumerate(f) if abs(v)>1e-14]
            for n,m,moving in self.mass_loads:
                f=[0.,0.,-m*9.80665 if gravity else 0.]
                if acceleration and moving in acceleration[1]:f[acceleration[0]]-=3*m
                after += [f'{n},{i+1},{v:.12g}' for i,v in enumerate(f) if abs(v)>1e-14]
            after += ['*DLOAD,OP=NEW']
            for group in ['frame','braces','gantry','column']:
                if group not in self.groups:continue
                if gravity:after += [f'G{group},GRAV,9810,0,0,-1']
                if acceleration and ((group=='gantry' and 'y' in acceleration[1]) or (group=='column' and 'x' in acceleration[1])):
                    d=[0,0,0];d[acceleration[0]]=-1;after += [f'G{group},GRAV,3000,'+','.join(map(str,d))]
            after += ['*NODE PRINT,NSET=OBS','U','*NODE PRINT,NSET=OBS','RF']
            for group in self.groups:after += [f'*EL PRINT,ELSET=G{group},TOTALS=ONLY','ELSE']
            after += ['*END STEP']
        if modal:
            after += ['*STEP','*FREQUENCY',str(self.mode_count),'*NODE PRINT,NSET=OBS','U']
            for group in self.groups:after += [f'*EL PRINT,ELSET=G{group},TOTALS=ONLY','ELSE']
            after += ['*END STEP']
        return '\n'.join(lines+node_lines(self.nodes)+after)+'\n'


def make_model(packet,pose,h,model=None):
    model=Model(h) if model is None else model;model.mode_count=packet.get('modeCount',20);beams=packet['beams'];parts={p['id']:p for p in packet['parts']};axes={a['axis']:a for a in packet['axes']}
    frame=[b for b in beams if b['id'] not in ('beam','zColumn')]
    # Welded RHS meet on their outer faces. Rigid centreline offsets represent the weld regions.
    base_mass=sum((b['widthMm']*b['heightMm']-(b['widthMm']-2*b['wallMm'])*(b['heightMm']-2*b['wallMm']))*norm(sub(b['startMm'],b['endMm']))*b['densityKgPerMm3'] for b in frame if b['id'].startswith('base/'))
    for b in frame:
        rho=b['densityKgPerMm3']*(parts['base']['massKg']/base_mass if b['id'].startswith('base/') else 1)
        model.beam(b['startMm'],b['endMm'],section(b),b['sectionX'],(b['youngsMPa'],rho),'braces' if 'Brace' in b['id'] else 'frame')
    original=list(model.lines);links=set()
    for row in original:
        for p in (row['a'],row['b']):
            # Explicit architecture joints replace proximity guesses at these endpoints.
            if any(norm(sub(p,start))<1e-5 for start,_ in packet.get('analysisFrameLinksMm',[])):continue
            for other in original:
                if row is other:continue
                q=project(p,other['a'],other['b']);distance=norm(sub(p,q))
                if 1e-5<distance<=50.0001:
                    if any(norm(sub(p,project(p,r['a'],r['b'])))<1e-5 and norm(sub(q,project(q,r['a'],r['b'])))<1e-5 for r in original):continue
                    key=tuple(sorted((tuple(p),tuple(q))))
                    if key not in links:model.rigid(p,q,'frame');links.add(key)
    for start,end in packet.get('analysisFrameLinksMm',[]):model.rigid(start,end,'frame')
    def nearest(p,rows=original):
        return min((project(p,r['a'],r['b']) for r in rows),key=lambda q:norm(sub(p,q)))
    for i,support in enumerate(packet['supports']):
        p=support['pointMm'];q=nearest(p);model.rigid(p,q,'frame');root=model.root('foot'+str(i),p,'frame')
        anchored=packet.get('floorSupport','anchored')=='anchored'
        lateral=packet['interfaces'].get('footLateralNPerMm',50000)
        model.spring(root,None,[lateral if anchored or i==0 else 0,lateral if anchored or i in (0,len(packet['supports'])//2) else 0,packet['interfaces']['footVerticalNPerMm'],0,0,0],'feet')
    dy=pose['y']-packet['workY']/2;dx=pose['x']-800
    beam=next(b for b in beams if b['id']=='beam');a=add(beam['startMm'],(0,dy,0));b=add(beam['endMm'],(0,dy,0))
    model.beam(a,b,section(beam),beam['sectionX'],(beam['youngsMPa'],beam['densityKgPerMm3']),'gantry','gantry')
    g=packet['interfaces'];guidek=g['guideTranslationNPerMm'];rotk=g['guideRotationNmmPerRad'];bolt=g['boltRotationNmmPerRad']
    ydrive=axes['y']['compliance']['stiffnessNPerM']/1000
    for side in ['Left','Right']:
        foot=parts['beamFoot'+side];fp=point(foot['poseMm']);fp=add(fp,(0,dy,next(p['thicknessMm'] for p in packet['plates'] if p['id']=='beamFoot'+side)/2))
        plate=next(p for p in packet['plates'] if p['id']=='beamFoot'+side)
        # Full-width strip spanning the actual 140 mm between the two guide blocks.
        for offset,label in [(-packet.get('yBlockSpacing',140)/2,'Front'),(packet.get('yBlockSpacing',140)/2,'Rear')]:
            end=add(fp,(0,offset,0));model.beam(fp,end,('RECT',plate['widthMm'],plate['thicknessMm']),(1,0,0),(plate['youngsMPa'],0),'feetplates','foot'+side)
            contact=tuple(next(r['railPointMm'] for r in pose['blocks'] if r['id']=='blockY'+side+('Rear' if label=='Rear' else ''))) if packet.get('guideLayout') else end
            model.rigid(end,contact,'foot'+side)
            moving=model.root('y'+side+label,contact,'foot'+side)
            fixed=model.root('f'+side+label,contact,'frame');q=nearest(contact);model.rigid(q,contact,'frame')
            model.spring(moving,fixed,[g.get('yBlockLateralNPerMm',100000),0,g.get('yBlockRadialNPerMm',539000),0,0,0] if packet.get('guideLayout') else [guidek,ydrive/2,guidek,rotk,rotk,rotk],'Yblocks' if packet.get('guideLayout') else 'Yguides_drive')
        attachment=(fp[0],a[1],a[2]);model.rigid(fp,attachment,'foot'+side)
        r1=model.root('beam'+side,attachment,'gantry');r2=model.root('plate'+side,attachment,'foot'+side)
        if packet.get('beamFootJointWelded'):
            # The beam bears/welds across its full Y footprint, not at one point.
            for label,offset in [('Front',-beam['widthMm']/2),('Rear',beam['widthMm']/2)]:
                at=add(fp,(0,offset,plate['thicknessMm']/2))
                model.rigid(attachment,at,'gantry');model.rigid(add(fp,(0,offset,0)),at,'foot'+side)
                upper=model.root('weldBeam'+side+label,at,'gantry');lower=model.root('weldFoot'+side+label,at,'foot'+side)
                model.spring(upper,lower,[1e10*g.get('weldSpringFactor',1)]*3+[1e13*g.get('weldSpringFactor',1)]*3,'welds')
        elif g.get('beamFootMachinedContactNPerMm'):
            # Four real bearing quadrants; two locating dowels carry transverse shear.
            # Preloaded contact must remain closed; this linear screen does not prove it.
            normal=g['beamFootMachinedContactNPerMm'];shear=g.get('beamFootDowelShearNPerMm',200000)
            for ix in (-1,1):
                for iy in (-1,1):
                    at=add(fp,(ix*23,iy*46,plate['thicknessMm']/2));label=side+str(ix)+str(iy)
                    model.rigid(attachment,at,'gantry');model.rigid(add(fp,(0,iy*46,0)),at,'foot'+side)
                    upper=model.root('contactBeam'+label,at,'gantry');lower=model.root('contactFoot'+label,at,'foot'+side)
                    model.spring(upper,lower,[shear,shear,normal,0,0,0],'bolts')
        else:model.spring(r1,r2,[g['boltTranslationNPerMm']]*3+[bolt]*3,'bolts')
    if packet.get('guideLayout'):
        for side in ['Left','Right']:
            d=next(d for d in pose['drives'] if d['id']=='pinionY'+side);q=tuple(d['pointMm'])
            model.rigid(nearest(q),q,'frame');fp=add(point(parts['beamFoot'+side]['poseMm']),(0,dy,next(p['thicknessMm'] for p in packet['plates'] if p['id']=='beamFoot'+side)/2));model.rigid(fp,q,'foot'+side)
            fixed=model.root('YdriveFixed'+side,q,'frame');moving=model.root('YdriveMoving'+side,q,'foot'+side)
            model.spring(moving,fixed,[0,ydrive,0,0,0,0],'Ydrive')
        from front_guides import front_carriage
        xroot,zroot=front_carriage(model,packet,pose,a,b,axes)
        tip=tuple(pose['toolTipMm'])
    else:
        bp=pose['blockXMm'];beamload=project(bp,a,b);model.rigid(beamload,bp,'gantry')
        fixed=model.root('Xfixed',bp,'gantry');moving=model.root('Xmoving',bp,'x')
        model.spring(moving,fixed,[axes['x']['compliance']['stiffnessNPerM']/1000,guidek,guidek,rotk,rotk,rotk],'Xguide_drive')
        xp=next(p for p in packet['plates'] if p['id']=='xCarriage');zcol=next(b for b in beams if b['id']=='zColumn')
        za=add(zcol['startMm'],(dx,dy,0));zb=add(zcol['endMm'],(dx,dy,0))
        platez=xp['poseMm']['z']+xp['thicknessMm']/2
        start=(bp[0],bp[1],platez);end=(zb[0],zb[1],platez)
        model.rigid(bp,start,'x');model.beam(start,end,('RECT',xp['widthMm'],xp['thicknessMm']),(1,0,0),(xp['youngsMPa'],0),'Xplate','x')
        model.rigid(end,zb,'x');xroot=model.root('Xmass',end,'x')
        ztop=model.root('Ztop',zb,'column');xtop=model.root('Xtop',zb,'x')
        model.spring(ztop,xtop,[g['boltTranslationNPerMm']]*3+[bolt]*3,'bolts')
        model.beam(za,zb,section(zcol),zcol['sectionX'],(zcol['youngsMPa'],zcol['densityKgPerMm3']),'column','column')
        zp=pose['blockZMm'];q=project(zp,za,zb);model.rigid(q,zp,'column')
        zf=model.root('Zfixed',zp,'column');zm=model.root('Zmoving',zp,'z')
        model.spring(zm,zf,[guidek,guidek,axes['z']['screw']['stiffnessNPerM']/1000,rotk,rotk,rotk],'Zguide_screw')
        zplate=next(p for p in packet['plates'] if p['id']=='zCarriage');tip=pose['toolTipMm']
        end=(zp[0],tip[1],zp[2]);model.beam(zp,end,('RECT',zplate['widthMm'],zplate['thicknessMm']),(1,0,0),(zplate['youngsMPa'],0),'Zplate','z')
        model.rigid(end,tip,'z');zroot=model.root('Zmass',end,'z')
    # Rigid table body on six spatial, finite translational mounting interfaces.
    # Avoid cantilevering its entire mass and inertia from a single floor foot.
    bed=tuple(tip)
    if packet.get('guideLayout'):
        table=parts['vacuumTable'];bedcentre=point(table['centreOfMassMm'])
        bedroot=model.root('bedMass',bedcentre,'bed')
        for i,stock in enumerate(r for r in frame if r['id'] in ('base/tableCross0','base/tableCross3','base/tableCross6')):
            for j,key in enumerate(('startMm','endMm')):
                q=tuple(stock[key]);seat=add(q,(0,0,stock['heightMm']/2))
                model.rigid(q,seat,'frame');model.rigid(bedcentre,seat,'bed')
                fixed=model.root('bedFrame'+str(i)+str(j),seat,'frame');moving=model.root('bedTable'+str(i)+str(j),seat,'bed')
                model.spring(moving,fixed,[g['boltTranslationNPerMm']]*3+[0]*3,'bedMounts')
        model.rigid(bedcentre,bed,'bed');model.queries={'tip':(tip,getattr(model,'tip_space','z')),'bed':(bed,'bed')}
    else:
        q=nearest(bed);model.rigid(q,bed,'frame');model.queries={'tip':(tip,getattr(model,'tip_space','z')),'bed':(bed,'frame')}
    for name,p in [('lf',original[0]['a']),('lr',original[0]['b']),('rf',original[1]['a']),('rr',original[1]['b'])]:model.queries[name]=(p,'frame')
    yroot=model.root('Ymass',project((800,pose['y'],a[2]),a,b),'gantry')
    native={b['id'] for b in beams};native.add('base')
    zids={p['id'].split(' (whole chain upper bound)')[0] for p in axes['z']['movingMembers']};xids={p['id'].split(' (whole chain upper bound)')[0] for p in axes['x']['movingMembers']};yids={p['id'].split(' (whole chain upper bound)')[0] for p in axes['y']['movingMembers']}
    model.independent_floor_mass=0.0
    floor_members={'vacuumBlower','cabinet','vfd','brakeResistor','mainBreaker','mainContactor','safetyRelay','emergencyStop'} if packet.get('guideLayout') else set()
    # All real components not already represented by stock beam density retain CAD mass and inertia.
    for p in packet['parts']:
        if p['id'] in native:continue
        if p['id'] in floor_members:
            model.independent_floor_mass+=p['massKg'];continue
        category='z' if p['id'] in zids else 'x' if p['id'] in xids else 'y' if p['id'] in yids else 'frame'
        centre=point(p['centreOfMassMm']);delta=(dx,dy,0) if category in ('z','x') else (0,dy,0) if category=='y' else (0,0,0)
        centre=add(centre,delta)
        if packet.get('guideLayout') and category=='frame':
            if p['id'] in ('vacuumTable','spoilboard'):
                root=bedroot;category='bed'
            else:
                row=min(original,key=lambda r:norm(sub(centre,project(centre,r['a'],r['b']))))
                direction=sub(row['b'],row['a']);length=norm(direction);at=project(centre,row['a'],row['b'])
                distance=min(length,max(0,round(norm(sub(at,row['a']))/50)*50))
                anchor=add(row['a'],mul(direction,distance/length))
                root=model.root('fixedMass'+str(len(model.masses)),anchor,'frame')
        else:root=zroot if category=='z' else xroot if category=='x' else yroot if category=='y' else min(('foot'+str(i) for i in range(len(packet['supports']))),key=lambda r:norm(sub(centre,model.roots[r]['p'])))
        if category=='z' and hasattr(model,'spindle_root') and p['id'] in ('spindle','holder','collet','longestTool','spindleRelease','shoe','shoeLiftRod'):root=model.spindle_root
        t=p['centroidalInertiaKgMm2World'];inertia=[[t['xx'],t['xy'],t['xz']],[t['xy'],t['yy'],t['yz']],[t['xz'],t['yz'],t['zz']]]
        model.masses.append((root,p['massKg'],centre,inertia,category))
    model.mesh();return model


def tables(text):
    result={};step=0;kind=None;rows={}
    for line in text.splitlines()+['END']:
        match=re.search(r'S T E P\s+(\d+)',line)
        if match:step=int(match[1])
        if kind:
            v=line.split()
            if len(v)==4 and v[0].isdigit():rows[int(v[0])]=tuple(float(x.replace('D','E')) for x in v[1:]);continue
            if rows:result[(step,kind)]=rows;kind=None;rows={}
        if re.match(r'\s*displacements \(',line):kind='U';rows={}
        if re.match(r'\s*forces \(',line):kind='RF';rows={}
    return result


def solve(packet,pose,h,ccx,folder):
    model=make_model(packet,pose,h);tip=model.query_nodes['tip'];bed=model.query_nodes['bed']
    loads=[]
    for i,name in enumerate('XYZ'):
        f=[0.,0.,0.];f[i]=300;loads.append(('cut'+name,{tip:f,bed:mul(f,-1)},False,None))
    loads += [('weight',{},True,None),('accelX',{},False,(0,('x','z'))),('accelY',{},False,(1,('x','y','z')))]
    folder.mkdir(parents=True,exist_ok=True);(folder/'frame.inp').write_text(model.deck(loads))
    run_solver(ccx,folder,180);text=(folder/'frame.dat').read_text();tab=tables(text)
    cases=[]
    for step,(name,forces,_,_) in enumerate(loads,1):
        u=tab[(step,'U')];rf=tab[(step,'RF')];relative=sub(u[tip],u[bed]);budget=defaultdict(float)
        for n,m,i,k,group in model.spring_elements:budget[group]+=k*(u[n][i]-u[m][i])**2
        # Native beam energy is printed as totals, one per named group and step.
        block=re.split(r'S T E P\s+\d+',text)[step]
        for group in model.groups:
            match=re.search(r'total internal energy.*?set\s+G'+re.escape(group)+r'\b[^\n]*\n\s*\n?\s*([+\-\d.EeDd]+)',block,re.I)
            if match:budget[group]+=2*float(match[1].replace('D','E'))
        if name.startswith('cut'):
            direction='XYZ'.index(name[-1]);work=300*relative[direction]
            if work<=0 or not math.isfinite(work):raise ValueError('Non-positive cutting compliance')
            energy_error=abs(sum(budget.values())/work-1)
            if energy_error>.03:raise ValueError(f'Energy accounting mismatch {energy_error}; inspect {folder}')
            budget={k:v/300/300*1000 for k,v in budget.items()}
        else:energy_error=None
        reactions={}
        for i in range(len(packet['supports'])):
            root=model.roots['foot'+str(i)];ks=next(k for a,b,k,g in model.springs if a=='foot'+str(i) and b is None);reactions[str(i)]=[-ks[j]*u[root['ref']][j] for j in range(3)]
        net=[sum(r[j] for r in reactions.values()) for j in range(3)]
        if name.startswith('cut'):
            force_error=norm(net)/300
        elif name=='weight':
            force_error=norm(sub(net,(0,0,(packet['totalMassKg']-model.independent_floor_mass)*9.80665)))/((packet['totalMassKg']-model.independent_floor_mass)*9.80665)
        else:
            axis=0 if name=='accelX' else 1
            target=[0.,0.,0.];target[axis]=next(a['movingMassKg'] for a in packet['axes'] if a['axis']==name[-1].lower())*3
            force_error=norm(sub(net,target))/norm(target)
        if force_error>.001:raise ValueError(f'Support force balance error {force_error}: {folder}/{name}')
        cases.append(dict(name=name,supportForceRelativeError=force_error,toolRelativeMm=relative,complianceUmPerN=[x/300*1000 for x in relative] if name.startswith('cut') else None,
            budgetUmPerN=budget if name.startswith('cut') else None,energyRelativeError=energy_error,supportReactionsN=reactions,
            frameTwistMm=(u[model.query_nodes['lf']][2]-u[model.query_nodes['lr']][2]-u[model.query_nodes['rf']][2]+u[model.query_nodes['rr']][2])))
    for suffix in ('.frd','.12d'):
        artifact=folder/('frame'+suffix)
        if artifact.exists():
            raw=artifact.read_bytes();packed=gzip.compress(raw,compresslevel=3,mtime=0)
            artifact.with_suffix(artifact.suffix+'.gz').write_bytes(packed)
            if gzip.decompress(packed)!=raw:raise ValueError('Solver archive integrity failed')
            artifact.unlink()
    from modes import diagnostics
    mode_rows=diagnostics(model,text,frequencies(text))
    return dict(modalParticipation=mode_rows,independentFloorMassKg=model.independent_floor_mass,pose=pose,meshMm=h,nodes=len(model.nodes),beamElements=len(model.elements),firstModesHz=frequencies(text),cases=cases)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--packets',type=Path,required=True);parser.add_argument('--ccx',type=Path,required=True);parser.add_argument('--one',action='store_true');parser.add_argument('--floor',choices=('anchored','free'),default='anchored');args=parser.parse_args()
    reports=[]
    snapshot=args.packets/'source';snapshot.mkdir(exist_ok=True)
    for source in Path(__file__).parent.glob('*.py'):shutil.copy2(source,snapshot/source.name)
    shared=snapshot/'mill-fem';shared.mkdir(exist_ok=True)
    for source in MILL.glob('*.py'):shutil.copy2(source,shared/source.name)
    (args.packets/'results.json').unlink(missing_ok=True)
    hashes={}
    for length in [2600,3150]:
        manifest=args.packets/f'y{length}'/'manifest.json'
        hashes[str(length)]=hashlib.sha256(manifest.read_bytes()).hexdigest()
        packet=json.loads(manifest.read_text())
        if packet['units']!={'length':'mm','force':'N','mass':'kg','youngsModulus':'MPa'}:raise ValueError('Unsupported packet units')
        for part in packet['parts']:
            if part['stepFile'] and hashlib.sha256((manifest.parent/part['stepFile']).read_bytes()).hexdigest()!=part['stepSha256']:raise ValueError('STEP integrity failed: '+part['id'])
    identity=dict(modalDiagnosticsSha256=hashlib.sha256(Path(__file__).with_name('modes.py').read_bytes()).hexdigest(),frontGuideSha256=hashlib.sha256(Path(__file__).with_name('front_guides.py').read_bytes()).hexdigest(),packetSha256=hashes,studySha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        floorSupport=args.floor,footLateralNPerMm=50000,footVerticalNPerMm=50000,gmshVersion=gmsh.__version__,ccxSha256=hashlib.sha256(args.ccx.read_bytes()).hexdigest())
    for length in [2600,3150]:
        packet=json.loads((args.packets/f'y{length}'/'manifest.json').read_text());packet['floorSupport']=args.floor
        poses=packet['poses'][4:5] if args.one else packet['poses']
        for i,pose in enumerate(poses):
            for h in ([200] if args.one else [100,50]):
                folder=args.packets/f'solve-y{length}-p{i}-h{h}'
                if shutil.disk_usage(args.packets).free<128*1024*1024:raise ValueError('Less than 128 MiB free; preserving partial results before another solve')
                result=solve(packet,pose,h,args.ccx.resolve(),folder);reports.append(dict(workY=length,**result))
                print(f'SR5 Y={length} pose={i} h={h}: f1={result["firstModesHz"][0]:.3f} Hz',flush=True)
                (args.packets/'results.partial.json').write_text(json.dumps(dict(status='in progress',identity=identity,results=reports),indent=2)+'\n')
        if args.one:break
    if not args.one:
        grouped=defaultdict(dict)
        for r in reports:grouped[(r['workY'],r['pose']['x'],r['pose']['y'])][r['meshMm']]=r
        errors=[]
        for rows in grouped.values():
            a,b=rows[100],rows[50]
            static=max(abs(a['cases'][i]['complianceUmPerN'][i]/b['cases'][i]['complianceUmPerN'][i]-1) for i in range(3))
            modal=max(abs(x/y-1) for x,y in zip(a['firstModesHz'][:6],b['firstModesHz'][:6]))
            if static>.03 or modal>.05:raise ValueError(f'Mesh convergence failed: static={static}, modal={modal}')
            errors.append(dict(workY=b['workY'],pose=b['pose'],staticRelativeChange=static,modalRelativeChange=modal))
        (args.packets/'results.json').write_text(json.dumps(dict(status='complete reduced screening; see summary.json for target assessment',identity=identity,
            meshConvergence=errors,results=reports),indent=2)+'\n')

if __name__=='__main__':main()
