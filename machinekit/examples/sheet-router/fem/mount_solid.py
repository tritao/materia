"""Local elastic tetrahedra with finite connector springs and rigid load ports.

Reconstruction must match the native CAD volume before meshing. Tetrahedra are
linear: mesh convergence is required. Port patches are idealized bearing faces,
not a nonlinear contact/preload or local stress qualification. Units N/mm/rad.
"""
import sys
import inspect
from pathlib import Path
import numpy as np
from scipy.sparse import coo_matrix, diags
from scipy.sparse.linalg import cg
from mount_frame import cross_matrix, offset
from mount_local import rotation, position


def mesh_stock(row, native_volume, size):
    # Read the retained environment only; callers set PYTHONDONTWRITEBYTECODE=1.
    sys.path.append(str(Path(__file__).resolve().parents[2]/'bench-mill/build/fem/venv/lib/python3.12/site-packages'))
    import gmsh
    gmsh.initialize()
    try:
        gmsh.option.setNumber('General.NumThreads', 1)
        gmsh.option.setNumber('General.Terminal', 0)
        gmsh.model.add(row['id'])
        occ = gmsh.model.occ
        anchor = row.get('pivotAnchor')
        carrier = row.get('carrierStock')
        if anchor:
            regions = anchor['stockBoxBounds']
        elif carrier:
            regions = carrier['stockBoxBounds']
        else:
            raise ValueError('Explicit solid stock required')
        stock = [(3, occ.addBox(r[0], r[2], r[4], r[1]-r[0], r[3]-r[2], r[5]-r[4])) for r in regions]
        if carrier:
            stock.append((3, occ.addCylinder(0, 0, 0, 0, 0, carrier['thicknessMm'], carrier['bossRadiusMm'])))
        merged, _ = occ.fuse(stock[:1], stock[1:])
        tools = []
        if anchor:
            for hole in anchor['bores']:
                axis = rotation(hole['frame'])[:, 2]
                at = position(hole['frame'])-.1*axis
                delta = (hole['depth']+.2)*axis
                tools.append((3, occ.addCylinder(*at, *delta, hole['diameter']/2)))
        else:
            thickness = carrier['thicknessMm']
            tools.append((3, occ.addCylinder(0, 0, -1, 0, 0, thickness+2, carrier['boreRadiusMm'])))
            flange = row['carrierMotorFlange']
            tools.append((3, occ.addCylinder(80, 0, -1, 0, 0, thickness+2, (flange['pilotDiameter']+.2)/2)))
            for hole in flange['holes']:
                tools.append((3, occ.addCylinder(80+hole['x'], hole['y'], -1, 0, 0, thickness+2, flange['holeDiameter']/2)))
        result, _ = occ.cut(merged, tools)
        occ.synchronize()
        volume = sum(occ.getMass(dim, tag) for dim, tag in result)
        if len(result) != 1 or abs(volume/native_volume-1) > 1e-7:
            raise ValueError('Reconstructed solid differs from native CAD: '+row['id']+' / '+str(volume/native_volume))
        gmsh.option.setNumber('Mesh.MeshSizeMin', size)
        gmsh.option.setNumber('Mesh.MeshSizeMax', size)
        gmsh.option.setNumber('Mesh.ElementOrder', 1)
        gmsh.model.mesh.generate(3)
        tags, coordinates, _ = gmsh.model.mesh.getNodes()
        xyz = np.asarray(coordinates).reshape(-1, 3)
        lookup = {int(tag): i for i, tag in enumerate(tags)}
        types, _, connectivity = gmsh.model.mesh.getElements(3)
        blocks = []
        for typ, nodes in zip(types, connectivity):
            if typ != 4:
                raise ValueError('Only first-order tetrahedra admitted')
            blocks.append(np.asarray([lookup[int(n)] for n in nodes], np.int32).reshape(-1, 4))
        surface_tags, _, _ = gmsh.model.mesh.getNodes(2, -1, includeBoundary=True)
        surface = np.zeros(len(xyz), bool)
        surface[[lookup[int(tag)] for tag in surface_tags]] = True
        return xyz, np.vstack(blocks), surface, dict(nativeVolumeMm3=native_volume, reconstructedVolumeMm3=volume,
                                           sizeMm=size, nodes=len(xyz), tetrahedra=sum(len(b) for b in blocks))
    finally:
        gmsh.finalize()


def tetra_stiffness(xyz, tets, youngs=210000., poisson=.3):
    """Vectorized batches bound temporary storage. Engineering shear strains."""
    lam = youngs*poisson/((1+poisson)*(1-2*poisson))
    mu = youngs/(2*(1+poisson))
    elastic = np.zeros((6, 6))
    elastic[:3, :3] = lam
    for i in range(3): elastic[i, i] += 2*mu
    for i in range(3, 6): elastic[i, i] = mu
    count = len(tets)
    values = np.empty((count, 12, 12))
    volumes = np.empty(count)
    for start in range(0, count, 1000):
        elements = tets[start:start+1000]
        points = xyz[elements]
        matrix = np.concatenate([np.ones((len(elements), 4, 1)), points], axis=2)
        determinant = np.linalg.det(matrix)
        if np.min(np.abs(determinant)) < 1e-12:
            raise ValueError('Degenerate tetrahedron')
        gradients = np.linalg.inv(matrix)[:, 1:, :]
        b = np.zeros((len(elements), 6, 12))
        for node in range(4):
            x, y, z = (gradients[:, j, node] for j in range(3))
            j = 3*node
            b[:, 0, j] = x; b[:, 1, j+1] = y; b[:, 2, j+2] = z
            b[:, 3, j] = y; b[:, 3, j+1] = x
            b[:, 4, j+1] = z; b[:, 4, j+2] = y
            b[:, 5, j] = z; b[:, 5, j+2] = x
        volume = np.abs(determinant)/6
        values[start:start+len(elements)] = np.einsum('eia,ij,ejb,e->eab', b, elastic, b, volume, optimize=True)
        volumes[start:start+len(elements)] = volume
    ids = (3*tets[:, :, None]+np.arange(3)).reshape(-1, 12)
    rows = np.broadcast_to(ids[:, :, None], values.shape).ravel()
    columns = np.broadcast_to(ids[:, None, :], values.shape).ravel()
    stiffness = coo_matrix((values.ravel(), (rows, columns)), shape=(3*len(xyz),)*2).tocsc()
    return stiffness, float(sum(volumes))


class SolidFixture:
    def __init__(self):
        self.parts = []; self.ports = {}; self.springs = []

    def port(self, name, point):
        if name in self.ports:
            raise ValueError('Duplicate load port')
        self.ports[name] = np.asarray(point, float)

    def solid(self, name, xyz, tets, patches, youngs=210000., poisson=.3, surface=None):
        assignments = {}
        for port, selector in patches.items():
            selected = np.flatnonzero(selector(xyz) & (np.ones(len(xyz), bool) if surface is None else surface))
            if len(selected) < 3:
                raise ValueError('Insufficient bearing-face nodes: '+port)
            if np.linalg.matrix_rank(xyz[selected]-xyz[selected].mean(axis=0), tol=1e-7) < 2:
                raise ValueError('Collinear bearing patch: '+port)
            for node in selected:
                if int(node) in assignments:
                    raise ValueError('Overlapping rigid bearing patches')
                assignments[int(node)] = port
        stiffness, volume = tetra_stiffness(xyz, tets, youngs, poisson)
        self.parts.append(dict(name=name, xyz=xyz, k=stiffness, assignments=assignments, volume=volume))

    def spring(self, a, b, stiffness, group, point=None):
        k = np.asarray(stiffness, float)
        if k.shape != (6,) or min(k) < 0 or not np.all(np.isfinite(k)):
            raise ValueError('Six finite nonnegative connector terms required')
        self.springs.append((a, b, k, group, self.ports[a] if point is None else np.asarray(point, float)))

    def solve(self, loads):
        ports = {name: 6*i for i, name in enumerate(self.ports)}
        size = 6*len(ports)
        for part in self.parts:
            indices = {}
            for node in range(len(part['xyz'])):
                if node not in part['assignments']:
                    indices[node] = size; size += 3
            part['internal'] = indices
        matrix = coo_matrix((size, size)).tocsc()
        for part in self.parts:
            rows = []; columns = []; values = []
            for node, point in enumerate(part['xyz']):
                owner = part['assignments'].get(node)
                if owner is None:
                    for j in range(3):
                        rows.append(3*node+j); columns.append(part['internal'][node]+j); values.append(1.)
                else:
                    transform = np.column_stack([np.eye(3), -cross_matrix(point-self.ports[owner])])
                    for i, j in zip(*np.nonzero(transform)):
                        rows.append(3*node+int(i)); columns.append(ports[owner]+int(j)); values.append(transform[i, j])
            transform = coo_matrix((values, (rows, columns)), shape=(3*len(part['xyz']), size)).tocsc()
            part['transform'] = transform
            matrix += transform.T@part['k']@transform
        sr = []; sc = []; sv = []
        spring_records = []
        for a, b, stiffness, group, point in self.springs:
            ids = list(range(ports[a], ports[a]+6))
            transform = offset(point-self.ports[a])
            if b is not None:
                ids += list(range(ports[b], ports[b]+6))
                transform = np.column_stack([transform, -offset(point-self.ports[b])])
            k = transform.T@np.diag(stiffness)@transform
            for i, j in zip(*np.nonzero(k)):
                sr.append(ids[i]); sc.append(ids[j]); sv.append(k[i,j])
            spring_records.append((ids, k, group))
        matrix += coo_matrix((sv, (sr, sc)), shape=(size, size)).tocsc()
        diagonal = matrix.diagonal()
        if min(diagonal) <= 0:
            raise ValueError('Unrestrained fixture coordinate')
        force = np.zeros(size)
        for name, load in loads.items(): force[ports[name]:ports[name]+6] += load
        iterations = [0]
        def iteration(_): iterations[0] += 1
        tolerance = {'rtol':1e-9} if 'rtol' in inspect.signature(cg).parameters else {'tol':1e-9}
        u, status = cg(matrix, force, M=diags(1/diagonal), atol=1e-11,
                       maxiter=30000, callback=iteration, **tolerance)
        residual = float(np.linalg.norm(matrix@u-force)/max(1., np.linalg.norm(force)))
        if status != 0 or residual > 1e-7:
            raise ValueError('Unconverged local solid solve: '+str((status, residual)))
        energy = {}
        for part in self.parts:
            displacement = part['transform']@u
            energy[part['name']] = float(displacement@part['k']@displacement/2)
        for ids, k, group in spring_records:
            delta = u[ids]
            energy[group] = energy.get(group, 0.)+float(delta@k@delta/2)
        work = float(force@u)
        if work <= 0 or abs(2*sum(energy.values())/work-1) > 1e-6:
            raise ValueError('Local solid work/energy mismatch')
        return dict(work=work, energy=energy, energyShare={k:v/(work/2) for k, v in energy.items()},
                    displacement={name:u[start:start+6].tolist() for name, start in ports.items()},
                    iterations=iterations[0], residualRelative=residual, dofs=size)
