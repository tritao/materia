"""One C3D10 Y plate-path stiffness screen from exported production geometry.

Rigid rail/riser/block links and a rigid head are an UPPER stiffness bound.
Small joint bores are filled; mounting patches remain joint-derived. The
actuator's bearing, bolt/contact compliance and guide compliance are excluded.
Run once through heavy-run --slots 4; /usr/bin/time -v records the peak RSS.
Usage: y_module_fem.py EXPORT_DIR OUTPUT_JSON
"""
import json
import sys
from pathlib import Path
import numpy as np
import fe

root, output = Path(sys.argv[1]), Path(sys.argv[2])
data = json.loads((root / 'y-module.json').read_text())
paths, filled = [], {}
for part in data['parts']:
    name = part['name']
    target = root / ('Y-' + name + '-filled.step')
    volumes = fe.filled_step(str(root / ('Y-' + name + '.step')), part['holes'], str(target))
    filled[name] = {'filled_mm3': volumes[0], 'CAD_mm3': volumes[1]}
    paths.append(str(target))
centre = data['centre']
refine = [([centre-60, centre+60, -60, 60, 0, data['baseThickness']], 4.0)]
nodes, conns = fe.mesh_parts(paths, 10.0, min_size=4.0, curvature=3, fragment=False, refine=refine)
print("Meshed",len(nodes),"nodes",sum(len(c) for c in conns),"tet10 elements",flush=True)
m = fe.Model()
m.add_nodes(nodes)
for part, conn in zip(data['parts'], conns):
    m.add_body(part['name'], conn, fe.STEEL)
ids = [fe.used_nodes(conn) for conn in conns]

def plane(which, z):
    idx = ids[which]
    return idx[np.abs(nodes[idx, 2] - z) < 1e-6]

def patches(which, points):
    found = set()
    for item in points:
        p, r = np.asarray(item['point']), item['radius']
        idx = plane(which, p[2])
        chosen = idx[np.linalg.norm(nodes[idx, :2]-p[:2], axis=1) <= r+1e-6]
        if len(chosen) < 3:
            raise ValueError('Under-resolved joint patch: ' + str((p.tolist(), r, len(chosen))))
        found.update(int(n) for n in chosen)
    return sorted(found)

held = patches(0, [{'point': p, 'radius': 6.0} for p in data['held']])
for n in held:
    m.fix(n)
rail_nodes = []
for side in (-1, 1):
    y = side*data['railSpacing']/2
    idx = plane(0, data['baseThickness'])
    pts = nodes[idx]
    band = idx[(pts[:, 0] >= data['railStart']-1e-6) &
               (pts[:, 0] <= data['railStart']+data['railLength']+1e-6) &
               (np.abs(pts[:, 1]-data['riserCentres'][0 if side < 0 else 1]) <= data['riserWidth']/2+1e-6)]
    block = patches(1, [p for p in data['blocks'] if side*p['point'][1] > 0])
    if len(band) < 10:
        raise ValueError('Under-resolved rigid rail land')
    master = m.add_master('rigid-guide-'+str(side), (centre, y, data['baseThickness']))
    m.rigid(master, list(band)+block)
    rail_nodes.append({'base': len(band), 'carriage': len(block)})
head = m.add_master('rigid-head-at-hob', data['hob'])
head_nodes = patches(1, data['head'])
m.rigid(head, head_nodes)
K, mass_matrix, volumes = m.system()
del mass_matrix
T, free = m.reduction()
Kr = (T.T @ K @ T).tocsc()
ml = m.amg(Kr, free)
print("Assembled",Kr.shape[0],"free DOF; six RHS on the same model",flush=True)
C, residuals = np.zeros((6, 6)), []
for k in range(6):
    dof = m.master_dof(head, k)
    f = np.zeros(K.shape[0]); f[dof] = 1.0
    u = T @ ml.solve(T.T@f, tol=1e-11, accel='cg', maxiter=2000)
    C[:, k] = [u[m.master_dof(head, j)] for j in range(6)]
    residuals.append(float(np.linalg.norm(T.T@(K@u-f))/np.linalg.norm(T.T@f)))
    print('RHS',k,'relative residual',residuals[-1],flush=True)
    if not np.isfinite(residuals[-1]) or residuals[-1] > 1e-7:
        raise ValueError('FE convergence failed at RHS '+str(k)+': '+str(residuals[-1])+'; remaining RHS not run')
if np.max(np.abs(C-C.T)) > 1e-7*np.max(np.abs(C)):
    raise ValueError('FE convergence/reciprocity failed: '+str(residuals))
trans = (C[:3, :3]+C[:3, :3].T)/2
values = np.linalg.eigvalsh(trans)*1000
if values[0] <= 0:
    raise ValueError('Non-positive plate-path compliance')
result = {'model': 'C3D10 size10; filled small bores; rigid guides/risers/head; joint-derived patches',
          'bound': 'Upper stiffness / lower compliance: excludes actuator, guides, fasteners and contact compliance',
          'assumptions': ['Linear steel E=210 GPa, nu=0.3', 'Closed joints; no bond credit',
                          'Output bolt patches held in all directions; no bearing compliance'],
          'nodes': len(nodes), 'elements': sum(len(c) for c in conns),
          'held_nodes': len(held), 'head_nodes': len(head_nodes), 'guide_nodes': rail_nodes,
          'volumes': filled, 'C_mm_N_rad_Nmm': C.tolist(),
          'translational_compliance_um_per_N_eigenvalues': values.tolist(),
          'plate_tip_compliance_um_per_N': float(values[-1]),
          'plate_tip_stiffness_N_per_um': float(1/values[-1]),
          'relative_residuals': residuals}
output.write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
