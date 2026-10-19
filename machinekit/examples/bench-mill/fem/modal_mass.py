"""Mass/inertia-preserving point masses and work-conjugate distributing motion."""
import math
from compliance import principal
from frame_solver import cross, distribute_wrench


def tensor(part):
    v=part.get('centroidalInertiaKgMm2World')
    if v is None:
        raise ValueError('Re-export modal packet: missing inertia for '+part['id'])
    return [[v['xx'],v['xy'],v['xz']],[v['xy'],v['yy'],v['yz']],[v['xz'],v['yz'],v['zz']]]


def aggregate(parts):
    mass=sum(p['massKg'] for p in parts)
    centre=[sum(p['massKg']*p['centreOfMassMm'][a] for p in parts)/mass for a in ('x','y','z')]
    inertia=[[0.0]*3 for _ in range(3)]
    for p in parts:
        j=tensor(p);r=[p['centreOfMassMm'][a]-centre[i] for i,a in enumerate(('x','y','z'))]
        for i in range(3):
            for k in range(3):
                inertia[i][k]+=j[i][k]+p['massKg']*((sum(x*x for x in r) if i==k else 0)-r[i]*r[k])
    return mass,centre,inertia


def constellation(mass, centre, inertia):
    """Six positive point masses reproduce arbitrary physical centroidal inertia.

    CalculiX MASS is translational. Symmetric pairs on principal axes reproduce
    the full inertia without a fictitious rotary spring or omitted products.
    """
    modes=principal(inertia)
    values=[v['complianceUmPerN'] for v in modes]
    if mass<=0 or min(values)<-1e-8*max(values):
        raise ValueError('Nonphysical mass/inertia')
    points=[]
    for i,mode in enumerate(modes):
        radius2=3*(sum(values)-2*values[i])/(2*mass)
        if radius2 < -1e-8*max(1,max(values)/mass):
            raise ValueError('Inertia violates triangle inequality')
        for sign in (-1,1):
            points.append((mass/6,[centre[k]+sign*math.sqrt(max(0,radius2))*mode['worldDirection'][k] for k in range(3)]))
    return points


def distributing_equations(nodes, ids, point, translation, rotation):
    """Dual of least-norm force transfer: six average rigid motions, no patch lock."""
    equations=[]
    for axis in range(3):
        force=[float(i==axis) for i in range(3)]
        loads=distribute_wrench(nodes,ids,point,force)
        equations.append([(translation,axis+1,1)]+[(n,d,-v) for n,d,v in loads if abs(v)>1e-16])
    for axis in range(3):
        r=[float(i==(axis+1)%3) for i in range(3)]
        force=[float(i==(axis+2)%3) for i in range(3)]
        plus=distribute_wrench(nodes,ids,[point[i]+.5*r[i] for i in range(3)],force)
        minus=distribute_wrench(nodes,ids,[point[i]-.5*r[i] for i in range(3)],[-v for v in force])
        coefficients={}
        for n,d,v in plus+minus: coefficients[n,d]=coefficients.get((n,d),0)+v
        equations.append([(rotation,axis+1,1)]+[(n,d,-v) for (n,d),v in coefficients.items() if abs(v)>1e-16])
    return equations
