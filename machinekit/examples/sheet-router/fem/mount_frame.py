"""Small local 3-D Timoshenko frame solver, N/mm/rad.

Rigid offsets are eliminated exactly. Compliance comes from deformable sections
and explicit physical springs, never a fitted end-anchor rotational stiffness.
This is a section/connection screen, not a solid/contact stress qualification.
"""
from dataclasses import dataclass
import numpy as np
from scipy.sparse import coo_matrix, bmat, csc_matrix
from scipy.sparse.linalg import spsolve


@dataclass(frozen=True)
class Section:
    area: float
    iv: float
    iw: float
    torsion: float
    shear_v: float
    shear_w: float

    def validate(self):
        values=np.array([self.area,self.iv,self.iw,self.torsion,self.shear_v,self.shear_w])
        if not np.all(np.isfinite(values)) or np.min(values)<=0:
            raise ValueError('Positive finite section properties required')


def rectangle(width,height):
    """Local v width, local w height; rectangular Saint-Venant approximation."""
    a,b=max(width,height),min(width,height)
    if b<=0:raise ValueError('Positive rectangle required')
    j=a*b**3*(1/3-.21*(b/a)*(1-b**4/(12*a**4)))
    return Section(width*height,width*height**3/12,height*width**3/12,j,
                   width*height*5/6,width*height*5/6)


def cross_matrix(r):
    x,y,z=r
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def offset(r):
    result=np.eye(6);result[:3,3:]=-cross_matrix(r)
    return result


def beam_matrix(a,b,section,normal,youngs=210000.,poisson=.3):
    section.validate();a=np.asarray(a,float);b=np.asarray(b,float)
    length=np.linalg.norm(b-a)
    if length<=1e-8 or not np.isfinite(length):raise ValueError('Finite nonzero beam required')
    axis=(b-a)/length;v=np.asarray(normal,float);v=v-axis*np.dot(v,axis)
    if np.linalg.norm(v)<1e-8:raise ValueError('Section normal parallel to beam')
    v/=np.linalg.norm(v);w=np.cross(axis,v);rotation=np.array([axis,v,w])
    shear=youngs/(2*(1+poisson));local=np.zeros((12,12))
    def block(ids,values):local[np.ix_(ids,ids)]+=values
    block([0,6],youngs*section.area/length*np.array([[1.,-1],[-1,1]]))
    block([3,9],shear*section.torsion/length*np.array([[1.,-1],[-1,1]]))
    for ids,inertia,area,sign in [([1,5,7,11],section.iw,section.shear_v,1),
                                ([2,4,8,10],section.iv,section.shear_w,-1)]:
        phi=12*youngs*inertia/(shear*area*length**2);l=length
        values=np.array([[12.,6*l,-12,6*l],[6*l,(4+phi)*l*l,-6*l,(2-phi)*l*l],
                         [-12.,-6*l,12,-6*l],[6*l,(2-phi)*l*l,-6*l,(4+phi)*l*l]])
        signs=np.diag([1.,sign,1.,sign]);values=signs@values@signs
        block(ids,youngs*inertia/(l**3*(1+phi))*values)
    transform=np.zeros((12,12))
    for i in range(4):transform[3*i:3*i+3,3*i:3*i+3]=rotation
    return transform.T@local@transform


class Frame:
    def __init__(self):
        self.points={};self.parents={};self.elements=[];self.fixed={}

    def node(self,name,point):
        p=np.asarray(point,float)
        if p.shape!=(3,) or not np.all(np.isfinite(p)):raise ValueError('Finite 3-D node required')
        if name in self.points and np.linalg.norm(self.points[name]-p)>1e-8:
            raise ValueError('Node identity changed: '+name)
        self.points[name]=p
        return name

    def rigid(self,master,slave):
        if master==slave or slave in self.parents:raise ValueError('Duplicate/cyclic rigid ownership')
        self.parents[slave]=master
        self._owner(slave)

    def _owner(self,name):
        if name not in self.points:raise ValueError('Missing node '+name)
        original=name;seen=set()
        while name in self.parents:
            if name in seen:raise ValueError('Rigid offset cycle')
            seen.add(name);name=self.parents[name]
            if name not in self.points:raise ValueError('Missing rigid master')
        return name,offset(self.points[original]-self.points[name])

    def beam(self,a,b,section,normal,group,youngs=210000.,poisson=.3):
        k=beam_matrix(self.points[a],self.points[b],section,normal,youngs,poisson)
        self.elements.append(((a,b),k,group))

    def spring(self,a,b,stiffness,group):
        values=np.asarray(stiffness,float)
        if values.shape!=(6,) or not np.all(np.isfinite(values)) or np.min(values)<0:
            raise ValueError('Six nonnegative spring terms required')
        k=np.diag(values)
        if b is None:self.elements.append(((a,),k,group))
        else:self.elements.append(((a,b),np.block([[k,-k],[-k,k]]),group))

    def fix(self,node,dofs=range(6)):
        owner,_=self._owner(node)
        if owner!=node:raise ValueError('Fix an independent node, not its rigid offset')
        self.fixed.setdefault(node,set()).update(dofs)

    def solve(self,loads,reference=None):
        roots=sorted({self._owner(n)[0] for n in self.points});indices={n:i for i,n in enumerate(roots)}
        size=6*len(roots);rows=[];cols=[];values=[];records=[]
        for nodes,k,group in self.elements:
            mappings=[self._owner(n) for n in nodes]
            t=np.zeros_like(k);ids=[]
            for i,(owner,r) in enumerate(mappings):
                t[6*i:6*i+6,6*i:6*i+6]=r;ids+=list(range(6*indices[owner],6*indices[owner]+6))
            km=t.T@k@t
            for i,global_i in enumerate(ids):
                for j,global_j in enumerate(ids):
                    if km[i,j]:rows.append(global_i);cols.append(global_j);values.append(km[i,j])
            records.append((nodes,k,group))
        matrix=coo_matrix((values,(rows,cols)),shape=(size,size)).tocsc();force=np.zeros(size)
        for node,value in loads.items():
            owner,r=self._owner(node);f=np.asarray(value,float)
            if f.shape!=(6,) or not np.all(np.isfinite(f)):raise ValueError('Finite force/moment vector required')
            start=6*indices[owner];force[start:start+6]+=r.T@f
        fixed={6*indices[n]+d for n,ds in self.fixed.items() for d in ds}
        free=np.array([i for i in range(size) if i not in fixed],int)
        if not len(free):raise ValueError('No free displacement coordinate')
        reference_reaction=None;reference_centre=None
        if reference is not None:
            if self.fixed or len(reference)<3 or len(set(reference))!=len(reference):
                raise ValueError('Mean reference needs distinct spatial points and no extra clamps')
            reference_centre=np.mean([self.points[n] for n in reference],axis=0)
            weights=1/len(reference)
            offsets=[self.points[n]-reference_centre for n in reference]
            inertia=sum(weights*(np.dot(r,r)*np.eye(3)-np.outer(r,r)) for r in offsets)
            if min(np.linalg.eigvalsh(inertia))<=1e-12:
                raise ValueError('Mean reference cannot be collinear')
            inverse=np.linalg.inv(inertia);constraint=np.zeros((6,size))
            for n,r in zip(reference,offsets):
                owner,transform=self._owner(n);start=6*indices[owner]
                constraint[:3,start:start+6]+=weights*transform[:3,:]
                constraint[3:,start:start+6]+=weights*inverse@cross_matrix(r)@transform[:3,:]
            c=csc_matrix(constraint)
            answer=spsolve(bmat([[matrix,c.T],[c,None]],format='csc'),np.r_[force,np.zeros(6)])
            if not np.all(np.isfinite(answer)):raise ValueError('Singular mean-reference frame')
            u=answer[:size];multipliers=answer[size:]
            if np.max(np.abs(constraint@u))>1e-8*max(1.,np.linalg.norm(u)):
                raise ValueError('Mean-reference closure failed')
            reference_reaction=-multipliers
            residual=matrix@u-force+c.T@multipliers
        else:
            u=np.zeros(size);u[free]=spsolve(matrix[free,:][:,free],force[free])
            residual=matrix@u-force
        if not np.all(np.isfinite(u)):raise ValueError('Singular/invalid local frame')
        if np.linalg.norm(residual[free])>1e-7*max(1.,np.linalg.norm(force[free])):
            raise ValueError('Local equilibrium residual exceeds tolerance')
        displacement={}
        for n in self.points:
            owner,r=self._owner(n);start=6*indices[owner];displacement[n]=r@u[start:start+6]
        energy={}
        for nodes,k,group in records:
            ue=np.concatenate([displacement[n] for n in nodes]);e=float(ue@k@ue/2)
            if e < -1e-8:raise ValueError('Negative element energy')
            energy[group]=energy.get(group,0.)+e
        work=float(force@u);stored=sum(energy.values())
        if work<=0 or abs(2*stored/work-1)>1e-6:raise ValueError('Local work/energy mismatch')
        return dict(displacement=displacement,work=work,energy=energy,
                    energyShare={k:v/stored for k,v in energy.items()},
                    freeResidualNorm=float(np.linalg.norm(residual[free])),dofs=size,
                    referenceReactionNAndNmm=None if reference_reaction is None else reference_reaction.tolist(),
                    referenceOriginMm=None if reference_centre is None else reference_centre.tolist(),
                    reactions={n:residual[6*i:6*i+6] for n,i in indices.items() if n in self.fixed})
