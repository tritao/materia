"""Prepared, separate frame stocks joined only through spatial weld springs.

The six-point line measure preserves each weld group's full translational and
rotational stiffness, including eccentricity, without guessing a local rotation
spring. Existing moving-body/drive/guide modelling remains in study.make_model.
"""
from collections import defaultdict
from pathlib import Path
import json,math,sys
from study import Model,project,norm,sub,section,equation_lines,rigid_equations
from modal_mass import constellation
from copy import deepcopy


class RootStore(dict):
    def __init__(self):super().__init__();self.aliases={}
    def __getitem__(self,key):
        return super().__getitem__(key if super().__contains__(key) else self.aliases[key])


class ExactOffsetModel(Model):
    """Work-conjugate rigid offsets, without fictitious offset beam stiffness."""
    def __init__(self,h):
        super().__init__(h);self.roots=RootStore();self.rigid_links=[];self.offset_count=0
    def root(self,name,p,space):
        canonical=super().root(name,p,space);self.roots.aliases[name]=canonical;return canonical
    def rigid(self,a,b,space):
        i=self.offset_count;self.offset_count+=1
        left=self.root('offset'+str(i)+'a',a,space);right=self.root('offset'+str(i)+'b',b,space)
        if left!=right:self.rigid_links.append((left,right))
    def deck(self,loads,modal=True):
        text=super().deck(loads,modal);parent={name:name for pair in self.rigid_links for name in pair}
        def find(name):
            while parent[name]!=name:parent[name]=parent[parent[name]];name=parent[name]
            return name
        for a,b in self.rigid_links:parent[find(b)]=find(a)
        components=defaultdict(list)
        for name in self.roots:
            if name in parent:components[find(name)].append(name)
        equations=[]
        for names in components.values():
            master=self.roots[names[0]]
            for name in names[1:]:
                slave=self.roots[name]
                equations+=rigid_equations(self.nodes,[slave['ref']],master['ref'],master['rot'],master['p'],(1,2,3))
                equations += [[(slave['rot'],i,1),(master['rot'],i,-1)] for i in (1,2,3)]
        return text.replace('*STEP\n','\n'.join(equation_lines(equations))+'\n*STEP\n',1)


def with_foot_welds(packet,native,stitches,pose):
    """Two accessible 8 mm fillets at each RHS sidewall/foot intersection.

    No credit for the beam-end side, whose 4 mm overhang cannot accommodate
    an assumed all-around fillet. Run-on/off tabs develop the full throat at
    both ends; remove tabs and blend without cutting the 92 mm finished run.
    """
    native=deepcopy(native);stitches=deepcopy(stitches);dy=pose['y']-packet['workY']/2
    beam=next(b for b in packet['beams'] if b['id']=='beam');y=beam['startMm'][1]+dy
    for side,group in [('Left','GFL'),('Right','GFR')]:
        part=next(p for p in packet['parts'] if p['id']=='beamFoot'+side);plate=next(p for p in packet['plates'] if p['id']=='beamFoot'+side)
        x=part['poseMm']['x'];z=part['poseMm']['z']+plate['thicknessMm']
        for sign in (-1,1):
            id=group+str(sign);a=[x-plate['widthMm']/2,y+sign*beam['widthMm']/2,z];b=[x+plate['widthMm']/2,a[1],z]
            native['seams'].append(dict(id=id,a=dict(member='beam'),b=dict(member='beamFoot'+side),type='Fillet'))
            stitches.append(dict(group=group,seam=id,a=a,b=b,lengthMm=plate['widthMm'],legMm=8,fullSizeEnds=True))
    return native,stitches


def centre_and_inertia(lines):
    length=sum(s['lengthMm'] for s in lines)
    centre=[sum(s['lengthMm']*(s['a'][i]+s['b'][i])/2 for s in lines)/length for i in range(3)]
    q=[[0.]*3 for _ in range(3)]
    for s in lines:
        d=sub(s['b'],s['a']);r=[(s['a'][i]+s['b'][i])/2-centre[i] for i in range(3)]
        for i in range(3):
            for j in range(3):q[i][j]+=s['lengthMm']*(r[i]*r[j]+d[i]*d[j]/12)
    trace=sum(q[i][i] for i in range(3));inertia=[[(trace if i==j else 0)-q[i][j] for j in range(3)] for i in range(3)]
    return length,centre,inertia


class WeldModel(ExactOffsetModel):
    def __init__(self,h,packet,native,stitches,stiffness_factor=1.):
        super().__init__(h);self.native=native;self.weld_ports=[];self.bridge_ports=[];self.stiffness_factor=stiffness_factor
        self.stitches=stitches;self.by_stock=defaultdict(list)
        self.stock_beams={b['id'].removeprefix('base/'):b for b in packet['beams'] if b['id']!='beam'}
        for m in native['members']:self.by_stock[m['sourceStock']].append(m)
        self.axes={m['id']:(tuple(m['startMm']),tuple(m['endMm'])) for m in native['members']}
        self.members={m['id']:m for m in native['members']}
        self.raw_keys={tuple(b['startMm']+b['endMm']):name for name,b in self.stock_beams.items()}
    def spring(self,a,b,k,group):
        if group!='welds':super().spring(a,b,k,group)  # Replace the old guessed beam-foot joint.
    def beam(self,a,b,sec,normal=(0,0,1),material=(210000,0),group='rigid',space='frame'):
        if space=='frame' and group in ('frame','braces'):
            stock=self.raw_keys[tuple(list(a)+list(b))]
            for piece in self.by_stock[stock]:
                length=math.dist(piece['startMm'],piece['endMm']);area=sec[1]*sec[2]-(sec[1]-2*sec[3])*(sec[2]-2*sec[3])
                density=piece['volumeMm3']*7850e-9/(area*length)
                super().beam(piece['startMm'],piece['endMm'],sec,normal,(material[0],density),group,'stock/'+piece['id'])
        else:super().beam(a,b,sec,normal,material,group,space)
    def on_stock(self,p):
        choices=[(norm(sub(p,project(p,a,b))),id) for id,(a,b) in self.axes.items()]
        return min(choices)
    def rigid(self,a,b,space):
        if space=='frame' and self.on_stock(a)[0]<1e-5 and self.on_stock(b)[0]<1e-5:
            return  # Never bypass the actual prepared seam network with an old rigid joint.
        super().rigid(a,b,space)
    def mesh(self):
        frame_points={tuple(p) for row in self.lines if row['space']=='frame' for p in (row['a'],row['b'])}
        frame_points.update(tuple(p) for p,space in self.queries.values() if space=='frame')
        for i,p in enumerate(sorted(frame_points)):
            distance,id=self.on_stock(p)
            if distance>1e-5:continue
            load=self.root('frameLoad'+str(i),p,'frame');stock=self.root('stockLoad'+str(i),p,'stock/'+id)
            self.spring(load,stock,[1e10]*3+[1e13]*3,'loadBridges');self.bridge_ports.append((load,stock))
        # Candidate seams are associated with specific physical piece pairs, not
        # merely the two original saw blanks.
        native_seams={s['id']:s for s in self.native['seams']};groups=defaultdict(list)
        for s in self.stitches:
            seam=native_seams[s['seam']];pair=tuple(sorted((seam['a']['member'],seam['b']['member'])))
            groups[(s['group'],pair)].append(s)
        links={id:set() for id in self.axes}
        for _,pair in groups:
            if pair[0] not in links or pair[1] not in links:continue
            links[pair[0]].add(pair[1]);links[pair[1]].add(pair[0])
        seen=set();todo=[next(iter(links))]
        while todo:
            id=todo.pop()
            if id in seen:continue
            seen.add(id);todo.extend(links[id]-seen)
        if len(seen)!=len(links):raise ValueError('Prepared pieces without a qualified-length welded load path: '+', '.join(sorted(set(links)-seen)))
        for index,((group,pair),lines) in enumerate(sorted(groups.items())):
            length,centre,inertia=centre_and_inertia(lines)
            for j,(weight,at) in enumerate(constellation(length,centre,inertia)):
                ends=[]
                for side,id in enumerate(pair):
                    if id in self.axes:p=project(at,*self.axes[id]);space='stock/'+id
                    else:
                        space='gantry' if id=='beam' else 'foot'+id.removeprefix('beamFoot')
                        rows=[r for r in self.lines if r['space']==space and r['group'] in ('gantry','feetplates')]
                        p=min((project(at,r['a'],r['b']) for r in rows),key=lambda q:norm(sub(at,q)))
                    self.rigid(p,at,space);ends.append(self.root(f'weld{index}_{j}_{side}',at,space))
                # G*a*L / leg: an explicitly assumed weld shear compliance.
                # Rigid-piece kinematics at the weld reconstruct the complete
                # 6x6 connection matrix from these spatial translations.
                leg=lines[0].get('legMm',5)
                k=210000/(2*1.3)*(leg/math.sqrt(2))*weight/leg*self.stiffness_factor
                self.spring(*ends,[k,k,k,0,0,0],'frameWelds')
                self.weld_ports.append(dict(group=group,pieces=pair,at=at,k=k,ends=ends))
        for row in self.lines:
            if row['group']=='rigid':row['group']='coupling'
        super().mesh()
    def reactions(self,u):
        rows=defaultdict(list)
        for port in self.weld_ports:
            a,b=[self.roots[id]['ref'] for id in port['ends']]
            f=[port['k']*(u[a][i]-u[b][i]) for i in range(3)]
            rows[port['group']].append(dict(at=port['at'],forceN=f,pieces=port['pieces']))
        return dict(rows)
