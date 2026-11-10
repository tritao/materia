"""Small quadratic-tetrahedron (C3D10) FE kernel for the hobber checks: scipy only, N / mm / tonne-free units (N, mm, kg via 1e-9 factor in `rho`).

Mesh by gmsh (the bench-mill's pinned 4.12.2 in its retained venv), isotropic linear elasticity, consistent mass, and linear constraints
(fixed, tied, rigid-body groups with a 6-DOF master that can carry a point mass) eliminated exactly through a transformation matrix T.
Units: length mm, force N, E in N/mm^2, density in tonne/mm^3 (steel 7.85e-9) so that M is in tonne and frequencies come out in Hz.
Verified against a beam (`selftest`) before use; this is a screening model, not a stress qualification (linear tets, no contact: see each case).
"""
import os
import sys
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spl

GMSH_SITES = [
    os.environ.get("GMSH_SITE", ""),
    "/home/joao/dev/materia/machinekit/examples/bench-mill/build/fem/venv/lib/python3.12/site-packages",
]


PYAMG_SITES = [os.environ.get("PYAMG_SITE", ""), "/media/joao/DATA1/claude-hobber-tools/tmp/pylib"]


def import_pyamg():
    """pyamg 4.2.3 (small pure wheel, installed with `pip install --target`; not part of the system python)."""
    for site in PYAMG_SITES:
        if site and os.path.isdir(site) and site not in sys.path:
            sys.path.append(site)
    import pyamg
    return pyamg


def import_gmsh():
    for site in GMSH_SITES:
        if site and os.path.isdir(site) and site not in sys.path:
            sys.path.append(site)
    import gmsh
    return gmsh


STEEL = dict(E=210000.0, nu=0.30, rho=7.85e-9)

EDGES = [(0, 1), (1, 2), (2, 0), (3, 0), (3, 2), (3, 1)]  # gmsh type-11 mid-edge nodes
DL = np.array([[-1., -1, -1], [1, 0, 0], [0, 1, 0], [0, 0, 1]])


def shape(L):
    """N (10,), dN/dxi (10,3) at barycentric L (4,)."""
    N = np.zeros(10)
    dN = np.zeros((10, 3))
    for i in range(4):
        N[i] = L[i] * (2 * L[i] - 1)
        dN[i] = (4 * L[i] - 1) * DL[i]
    for k, (i, j) in enumerate(EDGES):
        N[4 + k] = 4 * L[i] * L[j]
        dN[4 + k] = 4 * (L[j] * DL[i] + L[i] * DL[j])
    return N, dN


def duffy_rule(n):
    g, w = np.polynomial.legendre.leggauss(n)
    g, w = (g + 1) / 2, w / 2
    pts, wts = [], []
    for a, wa in zip(g, w):
        for b, wb in zip(g, w):
            for c, wc in zip(g, w):
                xi, eta, zeta = a, b * (1 - a), c * (1 - a) * (1 - b)
                pts.append((xi, eta, zeta))
                wts.append(wa * wb * wc * (1 - a) ** 2 * (1 - b))
    return np.array(pts), np.array(wts)  # weights sum to 1/6


def rules():
    a, b = 0.1381966011250105, 0.5854101966249685
    k4 = np.array([[a, a, a], [b, a, a], [a, b, a], [a, a, b]]), np.full(4, 1 / 24)
    return k4, duffy_rule(3)


def tabulate(pts):
    out = []
    for p in pts:
        L = np.array([1 - p.sum(), p[0], p[1], p[2]])
        out.append(shape(L))
    return out


def constitutive(E, nu):
    c = E / ((1 + nu) * (1 - 2 * nu))
    D = np.zeros((6, 6))
    D[:3, :3] = c * nu
    for i in range(3):
        D[i, i] = c * (1 - nu)
        D[3 + i, 3 + i] = c * (1 - 2 * nu) / 2
    return D


def element_dofs(conn):
    return (3 * conn[:, :, None] + np.arange(3)[None, None, :]).reshape(len(conn), 30)


def assemble(nodes, conn, mat, ndof, chunk=4000, element_mat=None):
    """K (stiffness) and M (consistent mass) as csr of size ndof (>= 3 nodes)."""
    (kp, kw), (mp, mw) = rules()
    ktab, mtab = tabulate(kp), tabulate(mp)
    D = constitutive(mat["E"], mat["nu"])
    K = sp.csr_matrix((ndof, ndof))
    M = sp.csr_matrix((ndof, ndof))
    volume = 0.0
    for s in range(0, len(conn), chunk):
        c = conn[s:s + chunk]
        X = nodes[c]  # (e,10,3)
        e = len(c)
        Ke = np.zeros((e, 30, 30))
        Me = np.zeros((e, 10, 10))
        for (N, dN), w in zip(ktab, kw):
            J = np.einsum("eai,aj->eij", X, dN)
            det = np.linalg.det(J)
            if np.any(det <= 0):
                raise ValueError("inverted element")
            dNx = np.einsum("aj,eji->eai", dN, np.linalg.inv(J))
            B = np.zeros((e, 6, 30))
            for a in range(10):
                B[:, 0, 3 * a] = dNx[:, a, 0]
                B[:, 1, 3 * a + 1] = dNx[:, a, 1]
                B[:, 2, 3 * a + 2] = dNx[:, a, 2]
                B[:, 3, 3 * a + 1] = dNx[:, a, 2]
                B[:, 3, 3 * a + 2] = dNx[:, a, 1]
                B[:, 4, 3 * a] = dNx[:, a, 2]
                B[:, 4, 3 * a + 2] = dNx[:, a, 0]
                B[:, 5, 3 * a] = dNx[:, a, 1]
                B[:, 5, 3 * a + 1] = dNx[:, a, 0]
            Ke += w * det[:, None, None] * np.einsum("eik,ij,ejl->ekl", B, D, B)
            volume += w * det.sum()
        for (N, dN), w in zip(mtab, mw):
            J = np.einsum("eai,aj->eij", X, dN)
            det = np.linalg.det(J)
            Me += w * det[:, None, None] * np.outer(N, N)[None]
        dofs = element_dofs(c)
        r = np.repeat(dofs, 30, axis=1).ravel()
        cc = np.tile(dofs, (1, 30)).ravel()
        K = K + sp.coo_matrix((Ke.ravel(), (r, cc)), shape=(ndof, ndof)).tocsr()
        n3 = (3 * c[:, :, None] + np.arange(3)[None, None, :])  # (e,10,3)
        Mfull = np.zeros((e, 30, 30))
        for d in range(3):
            Mfull[:, d::3, d::3] = Me
        M = M + sp.coo_matrix((Mfull.ravel() * mat["rho"], (r, cc)), shape=(ndof, ndof)).tocsr()
    return K, M, volume


class Model:
    """Several meshes (bodies) in one DOF space, plus 6-DOF masters, springs and linear constraints."""

    def __init__(self):
        self.nodes = np.zeros((0, 3))
        self.bodies = []  # (name, conn, material)
        self.masters = []  # (name, point, mass, inertia(3x3) in tonne mm^2)
        self.cons = {}  # slave dof -> list[(master dof, coeff)]
        self.springs = []  # (dof_a, dof_b or None, k)
        self.loads = {}

    def add_nodes(self, pts):
        start = len(self.nodes)
        self.nodes = np.vstack([self.nodes, pts])
        return start

    def add_body(self, name, conn, mat):
        self.bodies.append((name, conn, mat))

    @property
    def nn(self):
        return len(self.nodes)

    def add_master(self, name, point, mass=0.0, inertia=None):
        self.masters.append([name, np.asarray(point, float), mass, np.zeros((3, 3)) if inertia is None else np.asarray(inertia, float)])
        return len(self.masters) - 1

    def master_dof(self, m, k):
        return 3 * self.nn + 6 * m + k

    def fix(self, node, dirs=(0, 1, 2)):
        for d in dirs:
            self.cons[3 * node + d] = []

    def fix_dof(self, dof):
        self.cons[dof] = []

    def tie(self, slave, master, dirs=(0, 1, 2)):
        """Equal displacement of two nodes in the given directions."""
        for d in dirs:
            self.cons[3 * slave + d] = [(3 * master + d, 1.0)]

    def rigid(self, m, nodes, dirs=(0, 1, 2)):
        """u(node) = u_m + theta_m x (x_node - x_m) for the listed nodes (components in dirs)."""
        p = self.masters[m][1]
        for n in nodes:
            r = self.nodes[n] - p
            cross = [(0, -r[2], r[1]), (r[2], 0, -r[0]), (-r[1], r[0], 0)]
            for d in dirs:
                terms = [(self.master_dof(m, d), 1.0)]
                for k in range(3):
                    if abs(cross[d][k]) > 1e-12:
                        terms.append((self.master_dof(m, 3 + k), cross[d][k]))
                self.cons[3 * n + d] = terms

    def spring(self, a, b, k):
        self.springs.append((a, b, k))

    def ndof(self):
        return 3 * self.nn + 6 * len(self.masters)

    def system(self):
        nd = self.ndof()
        K = sp.csr_matrix((nd, nd))
        M = sp.csr_matrix((nd, nd))
        vols = {}
        for name, conn, mat in self.bodies:
            k, m, v = assemble(self.nodes, conn, mat, nd)
            K, M = K + k, M + m
            vols[name] = v
        rows, cols, vals = [], [], []
        for a, b, k in self.springs:
            rows += [a]
            cols += [a]
            vals += [k]
            if b is not None:
                rows += [b, a, b]
                cols += [b, b, a]
                vals += [k, -k, -k]
        if rows:
            K = K + sp.coo_matrix((vals, (rows, cols)), shape=(nd, nd)).tocsr()
        r, c, v = [], [], []
        for i, (name, p, mass, I) in enumerate(self.masters):
            for d in range(3):
                r.append(self.master_dof(i, d))
                c.append(self.master_dof(i, d))
                v.append(mass)
            for a in range(3):
                for b in range(3):
                    if I[a, b] != 0:
                        r.append(self.master_dof(i, 3 + a))
                        c.append(self.master_dof(i, 3 + b))
                        v.append(I[a, b])
        M = M + sp.coo_matrix((v, (r, c)), shape=(nd, nd)).tocsr()
        return K, M, vols

    def reduction(self):
        nd = self.ndof()
        slaves = set(self.cons)
        for s_, terms in list(self.cons.items()):
            kept = []
            for md, cf in terms:
                if md in slaves:
                    if self.cons[md]:
                        raise ValueError("constraint chain: master dof %d is itself constrained" % md)
                    continue  # master held at zero
                kept.append((md, cf))
            self.cons[s_] = kept
        free = np.array([d for d in range(nd) if d not in slaves])
        col = np.full(nd, -1)
        col[free] = np.arange(len(free))
        rows, cols, vals = list(free), list(range(len(free))), [1.0] * len(free)
        for s, terms in self.cons.items():
            for md, cf in terms:
                rows.append(s)
                cols.append(col[md])
                vals.append(cf)
        T = sp.coo_matrix((vals, (rows, cols)), shape=(nd, len(free))).tocsr()
        return T, free

    def rigid_vectors(self, free):
        """Rigid-body near-null space on the reduced free dofs (for the AMG hierarchy)."""
        n3 = 3 * self.nn
        B = np.zeros((len(free), 6))
        for i, d in enumerate(free):
            if d < n3:
                node, k = divmod(d, 3)
                x = self.nodes[node]
                B[i, k] = 1.0
                if k == 0:
                    B[i, 4], B[i, 5] = x[2], -x[1]
                elif k == 1:
                    B[i, 3], B[i, 5] = -x[2], x[0]
                else:
                    B[i, 3], B[i, 4] = x[1], -x[0]
            else:
                k = (d - n3) % 6
                if k < 3:
                    B[i, k] = 1.0
        return B

    def amg(self, Kr, free):
        pyamg = import_pyamg()
        return pyamg.smoothed_aggregation_solver(Kr.tocsr(), B=self.rigid_vectors(free), max_coarse=400, strength="evolution", smooth="energy")

    def static_solver(self, K, T, free, method="auto"):
        """Factor (direct) or build the AMG hierarchy once; returns solve(loads dict) -> full displacement."""
        Kr = (T.T @ K @ T).tocsc()
        n = Kr.shape[0]
        if method == "direct" or (method == "auto" and n < 30000):
            lu = spl.splu(Kr, permc_spec="MMD_AT_PLUS_A")
            red = lambda b: lu.solve(b)
        else:
            ml = self.amg(Kr, free)
            red = lambda b: ml.solve(b, tol=1e-10, accel="cg", maxiter=400)

        def solve(loads):
            f = np.zeros(K.shape[0])
            for d, v in loads.items():
                f[d] += v
            return T @ red(T.T @ f)
        return solve

    def solve_static(self, loads, K=None, T=None, method="auto"):
        """loads: dict dof -> force. Returns full displacement vector."""
        if K is None:
            K, _, _ = self.system()
        free = None
        if T is None:
            T, free = self.reduction()
        elif method != "direct":
            _, free = self.reduction()
        f = np.zeros(K.shape[0])
        for d, v in loads.items():
            f[d] += v
        Kr = (T.T @ K @ T).tocsc()
        if method == "direct" or (method == "auto" and Kr.shape[0] < 30000):
            return T @ spl.splu(Kr).solve(T.T @ f)
        ml = self.amg(Kr, free)
        res = []
        x = ml.solve(T.T @ f, tol=1e-10, accel="cg", residuals=res, maxiter=400)
        self.last_amg_iterations = len(res)
        return T @ x

    def modes(self, count=8, K=None, M=None, T=None, method="auto", tol=1e-7):
        if K is None:
            K, M, _ = self.system()
        T, free = self.reduction() if T is None else (T, self.reduction()[1])
        Kr = (T.T @ K @ T).tocsc()
        Mr = (T.T @ M @ T).tocsc()
        if method == "amg" or (method == "auto" and Kr.shape[0] >= 30000):
            ml = self.amg(Kr, free)
            rng = np.random.default_rng(1)
            X = rng.standard_normal((Kr.shape[0], count + 4))
            vals, vecs = spl.lobpcg(Kr, X, B=Mr, M=ml.aspreconditioner(), largest=False, tol=tol, maxiter=400)
            vals, vecs = vals[:count], vecs[:, :count]
            order = np.argsort(vals)
            return np.sqrt(np.abs(vals[order])) / (2 * np.pi), T @ vecs[:, order]
        vals, vecs = spl.eigsh(Kr, k=count, M=Mr, sigma=-1.0, which="LM")
        order = np.argsort(vals)
        vals, vecs = vals[order], vecs[:, order]
        return np.sqrt(np.abs(vals)) / (2 * np.pi), T @ vecs


# ---- meshing ---------------------------------------------------------------------------------------------------------------------

def mesh_parts(steps, size, min_size=None, curvature=8, fragment=True, refine=None):
    """Mesh STEP files (several = fused conformally at shared faces). Returns nodes (n,3), list of tet10 connectivity per input part
    (0-based into nodes) and the OCC volumes. refine: list of (box [x0,x1,y0,y1,z0,z1], size) local size boxes."""
    gmsh = import_gmsh()
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.option.setNumber("General.NumThreads", 1)
        gmsh.model.add("m")
        tags = []
        for s in steps:
            ents = gmsh.model.occ.importShapes(s)
            vol = [e for e in ents if e[0] == 3]
            tags.append(vol)
        if fragment and len(steps) > 1:
            objs = tags[0]
            tools = [v for t in tags[1:] for v in t]
            _, mp = gmsh.model.occ.fragment(objs, tools)
            groups = [mp[0]] + [m for m in mp[1:]]
            tags = groups
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMax", size)
        gmsh.option.setNumber("Mesh.MeshSizeMin", min_size or size / 3)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", curvature)
        gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 1)
        gmsh.option.setNumber("Mesh.ElementOrder", 2)
        gmsh.option.setNumber("Mesh.HighOrderOptimize", 0)
        gmsh.option.setNumber("Mesh.Algorithm3D", 1)
        if refine:
            fields = []
            for i, (b, sz) in enumerate(refine):
                f = gmsh.model.mesh.field.add("Box")
                gmsh.model.mesh.field.setNumber(f, "VIn", sz)
                gmsh.model.mesh.field.setNumber(f, "VOut", size)
                for k, n in enumerate(["XMin", "XMax", "YMin", "YMax", "ZMin", "ZMax"]):
                    gmsh.model.mesh.field.setNumber(f, n, b[k])
                gmsh.model.mesh.field.setNumber(f, "Thickness", size)
                fields.append(f)
            m = gmsh.model.mesh.field.add("Min")
            gmsh.model.mesh.field.setNumbers(m, "FieldsList", fields)
            gmsh.model.mesh.field.setAsBackgroundMesh(m)
            gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
        gmsh.model.mesh.generate(3)
        ntags, raw, _ = gmsh.model.mesh.getNodes()
        index = {int(t): i for i, t in enumerate(ntags)}
        pts = np.array(raw).reshape(-1, 3)
        conns = []
        for vol in tags:
            blocks = []
            for dim, t in vol:
                types, ids, con = gmsh.model.mesh.getElements(3, t)
                for ty, cn in zip(types, con):
                    if ty != 11:
                        raise ValueError("expected tet10, got type %d" % ty)
                    blocks.append(np.array([index[int(x)] for x in cn]).reshape(-1, 10))
            conns.append(np.vstack(blocks))
        allc = np.vstack(conns)
        pts, touched = repair_inverted(pts, allc)
        mesh_parts.repaired = touched
        return pts, conns
    finally:
        gmsh.finalize()


def repair_inverted(nodes, conn):
    """Straighten the mid-edge nodes of any element whose Jacobian is not positive at the 4 quadrature points (curved boundary edges of coarse
    meshes); returns the repaired node array and the number of elements touched."""
    (kp, _), _ = rules()
    tab = tabulate(kp)
    nodes = nodes.copy()
    touched = 0
    for _ in range(3):
        X = nodes[conn]
        bad = np.zeros(len(conn), bool)
        for N, dN in tab:
            bad |= np.linalg.det(np.einsum("eai,aj->eij", X, dN)) <= 1e-12
        if not bad.any():
            break
        touched += int(bad.sum())
        for e in conn[bad]:
            for k, (i, j) in enumerate(EDGES):
                nodes[e[4 + k]] = 0.5 * (nodes[e[i]] + nodes[e[j]])
    return nodes, touched


def split_interface(nodes, conns, keep_first=0):
    """Give each body after the first its own copy of the nodes it shares with earlier bodies. Returns (nodes, conns, pairs[(a,b)])
    where a belongs to the first body, b is the later body's copy."""
    pairs = []
    nodes = nodes.copy()
    first = set(conns[0].ravel().tolist())
    out = [conns[0]]
    extra = []
    mapping = {}
    for c in conns[1:]:
        shared = sorted(set(c.ravel().tolist()) & first)
        local = {}
        for n in shared:
            local[n] = len(nodes) + len(extra)
            extra.append(nodes[n])
            pairs.append((n, local[n]))
        c2 = c.copy()
        flat = c2.ravel()
        for i, n in enumerate(flat):
            if n in local:
                flat[i] = local[n]
        out.append(flat.reshape(c.shape))
    if extra:
        nodes = np.vstack([nodes, np.array(extra)])
    return nodes, out, pairs


def nodes_where(nodes, pred, subset=None):
    idx = np.arange(len(nodes)) if subset is None else np.asarray(sorted(subset))
    return [int(i) for i in idx[pred(nodes[idx])]]


def corner_nodes(conn):
    return np.unique(conn[:, :4])


def used_nodes(conn):
    return np.unique(conn.ravel())


# ---- self test -------------------------------------------------------------------------------------------------------------------

def selftest(size=6.0):
    """Cantilever 100 x 10 x 10 mm steel: tip deflection under a tip force and first bending mode against Timoshenko theory."""
    import tempfile
    gmsh = import_gmsh()
    d = tempfile.mkdtemp(dir=os.environ.get("TMPDIR"))
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.add("b")
    gmsh.model.occ.addBox(0, 0, 0, 100, 10, 10)
    gmsh.model.occ.synchronize()
    path = os.path.join(d, "beam.step")
    gmsh.write(path)
    gmsh.finalize()
    pts, (conn,) = mesh_parts([path], size)
    m = Model()
    m.add_nodes(pts)
    m.add_body("beam", conn, STEEL)
    for n in nodes_where(pts, lambda p: p[:, 0] < 1e-6):
        m.fix(n)
    tip = nodes_where(pts, lambda p: p[:, 0] > 100 - 1e-6)
    K, M, vols = m.system()
    T, _ = m.reduction()
    load = {3 * n + 2: 100.0 / len(tip) for n in tip}
    u = m.solve_static(load, K, T)
    delta = np.mean([u[3 * n + 2] for n in tip])
    E, nu, L, b, h = 210000.0, 0.3, 100.0, 10.0, 10.0
    I, A, G = b * h ** 3 / 12, b * h, E / 2.6
    theory = 100 * L ** 3 / (3 * E * I) + 100 * L / (5 / 6 * G * A)
    f, _ = m.modes(4, K, M, T)
    mu = 1.8751
    f1 = mu ** 2 / (2 * np.pi * L ** 2) * np.sqrt(E * I / (7.85e-9 * A)) * 1.0
    return {"volume_error": vols["beam"] / 1e4 - 1, "tip_fe": delta, "tip_theory": theory, "tip_ratio": delta / theory,
            "f1_fe": float(f[0]), "f1_euler": f1, "f1_ratio": float(f[0]) / f1}


if __name__ == "__main__":
    print(selftest())


def filled_step(step, holes, out, max_diameter=20.0, extend=0.0):
    """Fill the small holes of a STEP part (tapped, clearance, dowel and counterbore bores listed in the export, up to `max_diameter`) so the mesh
    is not driven by them. Returns (filled volume, original volume). The holes are the exporter's `holes` (position, direction, diameter, depth, mm)."""
    gmsh = import_gmsh()
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("f")
        o = gmsh.model.occ
        ents = [e for e in o.importShapes(step) if e[0] == 3]
        v0 = sum(o.getMass(3, t) for _, t in ents)
        tools = []
        for h in holes:
            if h["diameter"] > max_diameter or h["diameter"] <= 0:
                continue
            d = h["direction"]
            p = h["position"]
            tools.append((3, o.addCylinder(p[0] - extend * d[0], p[1] - extend * d[1], p[2] - extend * d[2], d[0] * (h["depth"] + 2 * extend), d[1] * (h["depth"] + 2 * extend),
                                           d[2] * (h["depth"] + 2 * extend), h["diameter"] / 2)))
        out_tags, _ = o.fuse(ents, tools)
        o.synchronize()
        gmsh.write(out)
        return sum(o.getMass(3, t) for _, t in out_tags), v0
    finally:
        gmsh.finalize()
