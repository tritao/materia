"""Bolts-only C3D10 X carriage check: actual ground deck, four bars and four
replicated pads. No adhesive stiffness credit. Small holes <=20 mm are filled
for stiffness; CAD integrity retains them. Run through heavy-run, slots4.
Usage: carriage_fem.py EXPORT_DIR OUTPUT_JSON [SIZE_MM=10] [METHOD=amg]
"""
import json,sys
from pathlib import Path
import numpy as np
import fe
PADS=[(x,s*220.) for x in (102.,300.) for s in (-1,1)]
PAD_HALF=(41.5,24.)

def surface_nodes(nodes, ids, z, box):
    p = nodes[ids]
    sel = (np.abs(p[:, 2] - z) < 1e-6) & (p[:, 0] >= box[0] - 1e-6) & (p[:, 0] <= box[1] + 1e-6) & (p[:, 1] >= box[2] - 1e-6) & (p[:, 1] <= box[3] + 1e-6)
    return [int(i) for i in ids[sel]]


def face_weights(conn, nodes, sel_nodes, z):
    """Area weights (6-node triangle faces on the plane z) of the nodes in sel_nodes."""
    sel = set(sel_nodes)
    seen = {}
    for e in conn:
        for f in ((0, 1, 2, 4, 5, 6), (0, 1, 3, 4, 9, 7), (1, 2, 3, 5, 8, 9), (0, 2, 3, 6, 8, 7)):
            ids = e[list(f)]
            if np.all(np.abs(nodes[ids, 2] - z) < 1e-6):
                seen[tuple(sorted(ids[:3]))] = ids
    w = {}
    for ids in seen.values():
        if not all(i in sel for i in ids):
            continue
        p = nodes[ids[:3]]
        a = 0.5 * np.linalg.norm(np.cross(p[1] - p[0], p[2] - p[0]))
        for n in ids[3:]:
            w[n] = w.get(n, 0.0) + a / 3.0
    return w



def prepare(root, parts, names):
    gmsh = fe.import_gmsh()
    gmsh.initialize()
    paths, volumes, pad_bounds = [], {}, {}
    origin = np.asarray(parts[names[0]]['pose']['origin'])
    try:
        gmsh.option.setNumber('General.Terminal', 0)
        for name in names:
            if not name.startswith('Xpad-'):
                continue
            gmsh.clear()
            tags = gmsh.model.occ.importShapes(str(root / (name + '.step')))
            gmsh.model.occ.synchronize()
            # OCC boxes include its geometric tolerance; nominal coordinates in
            # this export are mm to six decimals. Retain one micron precision.
            box = [round(v, 6) for v in gmsh.model.getBoundingBox(-1, -1)]
            pad_bounds[name] = box
        for name in names:
            gmsh.clear()
            occ = gmsh.model.occ
            tags = [v for v in occ.importShapes(str(root / (name + '.step'))) if v[0] == 3]
            before = sum(occ.getMass(*tag) for tag in tags)
            delta = np.asarray(parts[name]['pose']['origin']) - origin
            if not np.allclose(parts[name]['pose']['axes'], parts[names[0]]['pose']['axes']):
                raise ValueError('rotated XC member ' + name)
            if name in pad_bounds:
                # These declared layers are rectangular stock with through
                # clearances only. Suppress those holes, not their compliance.
                occ.remove(tags, recursive=True)
                b = pad_bounds[name]
                tags = [(3, occ.addBox(*b[:3], *(np.array(b[3:]) - b[:3])))]
            else:
                tools = []
                for hole in parts[name]['holes']:
                    if not 0 < hole['diameter'] <= 20:
                        continue
                    p = hole['position']; d = np.asarray(hole['direction']) * hole['depth']
                    tools.append((3, occ.addCylinder(*p, *d, hole['diameter'] / 2)))
                if tools:
                    tags, _ = occ.fuse(tags, tools)
                if name == 'GE-X-carriage':
                    # Filling clearance holes must not make steel pillars in
                    # the replicated pad pockets. Restore each actual pocket.
                    pockets = []
                    for pad, b in pad_bounds.items():
                        shift = np.asarray(parts[pad]['pose']['origin']) - origin
                        low = np.array(b[:3]) + shift
                        span = np.array(b[3:]) - b[:3]
                        pockets.append((3, occ.addBox(*low, *span)))
                    tags, _ = occ.cut(tags, pockets)
            after = sum(occ.getMass(*tag) for tag in tags)
            volumes[name] = dict(originalMm3=before, smallHolesFilledMm3=after)
            occ.translate(tags, *delta)
            occ.synchronize()
            path = root / (name + '-stiffness.step')
            gmsh.write(str(path)); paths.append(str(path))
    finally:
        gmsh.finalize()
    return paths, volumes


def run(export_dir,size=10.,method='amg'):
    root=Path(export_dir)
    data=json.loads((root/'hobber-fem-export.json').read_text())
    parts={p['name']:p for p in data['parts']}
    names=['GE-X-carriage']+sorted(n for n in parts if n.startswith('Xrib-'))+sorted(n for n in parts if n.startswith('Xpad-'))
    if len(names)!=9:raise ValueError('expected deck, four ribs and four replication pads')
    paths,_=prepare(root,parts,names)
    pts,cs=fe.mesh_parts(paths,size,min_size=size/3,curvature=0)
    pts,cs,pairs=fe.split_interface(pts,cs)
    ids=fe.used_nodes(cs[0])
    joint=data['columnSoleJoint'];origin=np.asarray(joint['contactCentreLocal'])
    z=origin[2];pad_z=z-joint['deckThicknessMm']
    patches=[surface_nodes(pts,ids,z,(p['x0'],p['x1'],p['y0'],p['y1'])) for p in joint['contactFootprints']]
    if not patches or any(not p for p in patches):raise ValueError('missing column corner contact')
    sole=sorted(set(n for patch in patches for n in patch))
    pc=np.vstack([c for n,c in zip(names,cs) if n.startswith('Xpad-')]);pi=fe.used_nodes(pc)
    pads=[surface_nodes(pts,pi,pad_z,(x-PAD_HALF[0],x+PAD_HALF[0],y-PAD_HALF[1],y+PAD_HALF[1])) for x,y in PADS]
    if any(not p for p in pads):raise ValueError('missing guide contact')
    rib_for_node={int(node):n for n,c in zip(names,cs) if n.startswith('Xrib-') for node in np.unique(c)}
    bolts={n:[h['position'][:2] for h in parts[n]['holes'] if h['kind']=='clearance'] for n in names if n.startswith('Xrib-')}
    def model():
        m=fe.Model();m.add_nodes(pts)
        for n,c in zip(names,cs):m.add_body(n,c,dict(E=4000.,nu=.35,rho=1.8e-9) if n.startswith('Xpad-') else fe.STEEL)
        for a,b in pairs:
            rib=rib_for_node.get(b)
            if rib is None:m.tie(b,a);continue
            at_bolt=any(np.hypot(pts[a][0]-x,pts[a][1]-y)<=6 for x,y in bolts[rib])
            m.tie(b,a) if at_bolt else m.tie(b,a,(2,))
        return m
    residuals=[]
    def solve(m,load):
        K,_,_=m.system();T,free=m.reduction()
        u=m.static_solver(K,T,free,method=method)(load)
        f=np.zeros(m.ndof())
        for d,v in load.items():f[d]=v
        residual=float(np.linalg.norm(T.T@(K@u-f))/max(np.linalg.norm(T.T@f),1.))
        if not np.isfinite(residual) or residual>1e-6:raise ValueError('unconverged carriage solve')
        residuals.append(residual);return u
    m=model()
    for n in sole:m.fix(n,(2,))
    a,b=sorted(sole,key=lambda n:pts[n][0])[::len(sole)-1];m.fix(a,(0,1));m.fix(b,(1,))
    load={};weights=[]
    for p in pads[:2]:
        w=face_weights(pc,pts,p,pad_z);total=sum(w.values());weights.append(w)
        for n,area in w.items():load[3*n+2]=1000*area/total
    u=solve(m,load);delta=np.mean([sum(u[3*n+2]*a for n,a in w.items())/sum(w.values()) for w in weights])
    front=1/abs(delta)
    m=model()
    for p in pads:
        for n in p:m.fix(n,(2,))
    m.fix(pads[0][0],(0,1));m.fix(pads[2][0],(1,))
    master=m.add_master('sole',origin);m.rigid(master,sole)
    for d in (0,1,3,5):m.fix_dof(m.master_dof(master,d))
    u=solve(m,{m.master_dof(master,4):1e6})
    return dict(size=size,elements=[len(c) for c in cs],members=names,
        front_tie_N_per_um=front,sole_tilt_kNm_per_rad_pads_uz=1/abs(u[m.master_dof(master,4)]),
        relativeResidualFront=residuals[0],relativeResidualTilt=residuals[1],
        assumptions=['Bolts-only: radius6 mm tied patches, normal contact closed elsewhere; no bond credit',
            'Replication E4 GPa, nu0.35 ASSUMED; small holes<=20 mm filled',
            'Front pair 1 kN each; column contact held z; tilt 1 kN m, block pads held z'])

if __name__=='__main__':
    result=run(sys.argv[1],float(sys.argv[3]) if len(sys.argv)>3 else 10.,sys.argv[4] if len(sys.argv)>4 else 'amg')
    Path(sys.argv[2]).write_text(json.dumps([result],indent=2)+'\n')
    print(json.dumps(result,indent=2))
