"""Frozen reference dynamics: complete module lumps, SI units, no exterior-shell mass allocation."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent


def load_json(path): return json.loads(path.read_text())


def engineering_module():
    spec=importlib.util.spec_from_file_location('frozen_engineering',ROOT/'engineering-study.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def central_inertia(mass,cog,at_origin):
    return at_origin-mass*((cog@cog)*np.eye(3)-np.outer(cog,cog))


def reference_study(payload_kg=5.,cog_mm=150.):
    frozen=load_json(ROOT/'reference-baseline.json')
    assert hashlib.sha256((ROOT/'reference-layout.json').read_bytes()).hexdigest()==frozen['layoutSha256']
    geometry=load_json(ROOT/'build/engineering-input.json');layout=load_json(ROOT/'reference-layout.json')
    assert geometry['layoutSha256']==frozen['layoutSha256']
    config=load_json(ROOT/'original-comparison-config.json');config.pop('missionStudy',None)
    config['application'].update(toolKg=0,payloadKg=payload_kg,combinedCogAlongToolNormalMm=cog_mm)
    materials={'base':2810,'baseCover':2700,'upperArm':2810,'forearm':2700,
               'elbowOutputAdapter':2810,'elbowHousingAdapter':2810,'toolFlange':2810}
    config['partDensitiesKgM3']=materials
    study=engineering_module().Study(geometry,config)
    study.bodies=[b for b in study.bodies if not b[0].startswith('joint')]
    for index,family in enumerate(frozen['modules']):
        spec=frozen['wholeModuleLumps'][family];mass=spec['massKg'];cog=np.array(spec['cogMm'])/1000
        xx,yy,zz,xy,xz,yz=np.array(spec['inertiaAtOutputGmm2'])*1e-9
        inertia=central_inertia(mass,cog,np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]]))
        assert np.linalg.eigvalsh(inertia).min()>=-1e-10,'Invalid central module inertia'
        occurrence=next(o for o in layout['occurrences'] if o['id']=='joint'+str(index+1))
        pose=np.array(occurrence['matrix']).reshape(3,4)
        rotation=pose[:,:3]@np.array(spec['manualToAssetRotation'])
        origin=pose[:,:3]@(np.array(spec['outputOriginAssetMm'])/1000)+pose[:,3]/1000
        # J1/J2 housing downstream; J3-J6 housing upstream in supplier assembly.
        active=list(range(index+1 if index<2 else index))
        study.bodies.append(('module'+str(index+1),mass,origin+rotation@cog,
                             rotation@inertia@rotation.T,active))
    return study


def terms(study,q):
    """Vectorized serial-chain rigid-body gravity and mass matrix."""
    rotations=[np.eye(3)];translations=[np.zeros(3)]
    for axis,origin,angle in zip(study.axes,study.origins,q):
        x,y,z=axis;k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        r=np.eye(3)+np.sin(angle)*k+(1-np.cos(angle))*(k@k)
        rotations.append(rotations[-1]@r)
        translations.append(translations[-1]+rotations[-2]@(origin-r@origin))
    axes=np.array([rotations[i]@study.axes[i] for i in range(6)])
    origins=np.array([rotations[i]@study.origins[i]+translations[i] for i in range(6)])
    depths=np.array([len(b[4]) for b in study.bodies])
    assert all(b[4]==list(range(len(b[4]))) for b in study.bodies)
    r=np.array(rotations)[depths];p=np.einsum('nij,nj->ni',r,np.array([b[2] for b in study.bodies]))+np.array(translations)[depths]
    mass=np.array([b[1] for b in study.bodies]);inertia=np.array([b[3] for b in study.bodies])
    active=np.arange(6)[None,:]<depths[:,None]
    jv=np.cross(axes[None,:,:],p[:,None,:]-origins[None,:,:])*active[:,:,None]
    jw=axes[None,:,:]*active[:,:,None]
    matrix=np.einsum('n,nic,njc->ij',mass,jv,jv)+np.einsum('nic,ncd,njd->ij',jw,r@inertia@np.swapaxes(r,1,2),jw)
    return np.einsum('n,ni->i',mass*9.80665,jv[:,:,2]),matrix+np.diag(study.reflected)


def inverse_dynamics(study,q,velocity,acceleration):
    g,m=terms(study,q)
    derivatives=[]
    for i in range(6):
        step=np.eye(6)[i]*1e-5
        derivatives.append((terms(study,q+step)[1]-terms(study,q-step)[1])/(2e-5))
    d=np.array(derivatives)
    coriolis=np.einsum('k,kij,j->i',velocity,d,velocity)-.5*np.einsum('kij,i,j->k',d,velocity,velocity)
    return g+m@acceleration+coriolis


def transformed_bodies(study,q):
    result=[]
    for name,mass,cog,inertia,active in study.bodies:
        p=cog.copy()
        for i in reversed(active):
            r=engineering_module().rotation(study.axes[i],q[i]);p=study.origins[i]+r@(p-study.origins[i])
        result.append((name,mass,p,active))
    return result
