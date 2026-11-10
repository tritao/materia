#!/usr/bin/env python3
"""Rough plate-bending stiffness of the hobber X carriage, before and after H1 (FRAME_PLAN.md).

A Mindlin-Reissner plate (4-node, selective reduced shear integration) on a 10 mm grid, thickness per cell, so the deck,
its window and any ribs are cells of different thickness. Ribs are lumped as extra thickness (neutral-axis offset ignored),
which is rough but the same for every variant. Rails and blocks are taken rigid in the vertical direction; the column is a
rigid body on its sole. Dimensions mm, force N, E in N/mm^2.

Two numbers per variant:
  front tie   vertical stiffness at the front block pads (x = 102, y = +-220) with the column sole held: how well the front
              blocks are tied to the column (N/um, per pad pair loaded together);
  sole tilt   moment per radian the column sole needs to tilt about y with all four block pads held vertically (kN m/rad):
              the plate's share of the column's tilt compliance.

Usage: frame_stiffness.py [--json]
"""
import json
import sys

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spl

H = 10.0  # grid pitch
X0, X1, Y0, Y1 = 60.0, 386.0, -245.0, 245.0
SOLE = (276.0, 376.0, -190.0, 190.0)  # column sole footprint on the deck
PADS = [(x, s * 220.0) for x in (102.0, 300.0) for s in (-1, 1)]  # block centres
PAD_HALF = (41.5, 24.0)  # HGH25CA block 83 x 48


def thickness_map(spec):
    nx, ny = int(round((X1 - X0) / H)), int(round((Y1 - Y0) / H))
    t = np.zeros((nx, ny))
    for i in range(nx):
        for j in range(ny):
            x, y = X0 + (i + 0.5) * H, Y0 + (j + 0.5) * H
            t[i, j] = spec(x, y)
    return t


def before(x, y):
    """Welded steel: rear plate, two 50 mm legs, the rib under the nut bracket (all 20 mm)."""
    if x >= 240 or abs(y) >= 195 or (202.5 <= x <= 260 and abs(y) <= 35):
        return 20.0
    return 0.0


def after_flat(thickness):
    def spec(x, y):
        window = x <= 140 and abs(y) <= 150
        return 0.0 if window else thickness
    return spec


def after_ribbed(deck, rib_depth, rib_y=(130.0, 170.0)):
    def spec(x, y):
        if x <= 140 and abs(y) <= 150:
            return 0.0
        rib = x >= 140 and any(abs(abs(y) - r) <= 10 for r in rib_y)
        return deck + rib_depth if rib else deck
    return spec


def element_matrices(t, E, nu):
    """Mindlin Q4 stiffness for a square element of side H, dofs (w, tx, ty) per node."""
    G = E / (2 * (1 + nu))
    kappa = 5.0 / 6.0
    Db = E * t ** 3 / (12 * (1 - nu ** 2)) * np.array([[1, nu, 0], [nu, 1, 0], [0, 0, (1 - nu) / 2]])
    Ds = kappa * G * t * np.eye(2)
    a = H / 2
    gp = [-1 / np.sqrt(3), 1 / np.sqrt(3)]
    Ke = np.zeros((12, 12))
    xi_n = [-1, 1, 1, -1]
    eta_n = [-1, -1, 1, 1]

    def shape(xi, eta):
        N = np.array([(1 + xi_n[k] * xi) * (1 + eta_n[k] * eta) / 4 for k in range(4)])
        dNxi = np.array([xi_n[k] * (1 + eta_n[k] * eta) / 4 for k in range(4)])
        dNeta = np.array([eta_n[k] * (1 + xi_n[k] * xi) / 4 for k in range(4)])
        return N, dNxi / a, dNeta / a

    for xi in gp:
        for eta in gp:
            N, dNx, dNy = shape(xi, eta)
            B = np.zeros((3, 12))
            for k in range(4):
                B[0, 3 * k + 2] = dNx[k]  # kappa_x = d(ty)/dx   (ty = rotation about y -> w slope sign handled below)
                B[1, 3 * k + 1] = -dNy[k]
                B[2, 3 * k + 2] = dNy[k]
                B[2, 3 * k + 1] = -dNx[k]
            Ke += B.T @ Db @ B * a * a
    # one-point shear integration
    N, dNx, dNy = shape(0.0, 0.0)
    Bs = np.zeros((2, 12))
    for k in range(4):
        Bs[0, 3 * k] = dNx[k]
        Bs[0, 3 * k + 2] = N[k]
        Bs[1, 3 * k] = dNy[k]
        Bs[1, 3 * k + 1] = -N[k]
    Ke += Bs.T @ Ds @ Bs * (2 * a) ** 2
    return Ke


def solve(tmap, E, nu):
    nx, ny = tmap.shape
    node = lambda i, j: j * (nx + 1) + i
    nn = (nx + 1) * (ny + 1)
    cache = {}
    rows, cols, vals = [], [], []
    for i in range(nx):
        for j in range(ny):
            t = tmap[i, j]
            if t <= 0:
                continue
            if t not in cache:
                cache[t] = element_matrices(t, E, nu)
            Ke = cache[t]
            ids = [node(i, j), node(i + 1, j), node(i + 1, j + 1), node(i, j + 1)]
            dof = np.array([3 * n + d for n in ids for d in range(3)])
            r, c = np.meshgrid(dof, dof, indexing="ij")
            rows.append(r.ravel())
            cols.append(c.ravel())
            vals.append(Ke.ravel())
    K = sp.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(3 * nn, 3 * nn))
    used = np.zeros(nn, bool)
    for i in range(nx):
        for j in range(ny):
            if tmap[i, j] > 0:
                for n in (node(i, j), node(i + 1, j), node(i + 1, j + 1), node(i, j + 1)):
                    used[n] = True
    xy = lambda n: (X0 + (n % (nx + 1)) * H, Y0 + (n // (nx + 1)) * H)
    pads = [[n for n in range(nn) if used[n] and abs(xy(n)[0] - px) <= PAD_HALF[0] and abs(xy(n)[1] - py) <= PAD_HALF[1]]
            for px, py in PADS]
    sole = [n for n in range(nn) if used[n] and SOLE[0] <= xy(n)[0] <= SOLE[1] and SOLE[2] <= xy(n)[1] <= SOLE[3]]
    big = 1e9
    free = np.array([n for n in range(nn) if used[n]])
    unused_dofs = np.array([3 * n + d for n in range(nn) if not used[n] for d in range(3)])

    def with_constraints(fixed_w, tied_sole, loads_w, moment=None):
        Kc = K.tolil()
        for n in fixed_w:
            Kc[3 * n, 3 * n] += big
        F = np.zeros(3 * nn)
        for n, f in loads_w:
            F[3 * n] += f
        keep = np.ones(3 * nn, bool)
        keep[unused_dofs] = False
        Kc = Kc.tocsr()
        if tied_sole is None:
            sel = np.where(keep)[0]
            u = np.zeros(3 * nn)
            u[sel] = spl.spsolve(Kc[sel][:, sel].tocsc(), F[sel])
            return u, None
        # rigid sole: w = a + th (x - x0), ty rotation = th about y
        x0 = (SOLE[0] + SOLE[1]) / 2
        dofs_sole = {3 * n for n in tied_sole} | {3 * n + 1 for n in tied_sole} | {3 * n + 2 for n in tied_sole}
        other = [d for d in np.where(keep)[0] if d not in dofs_sole]
        m = len(other)
        T = sp.lil_matrix((3 * nn, m + 2))
        for c, d in enumerate(other):
            T[d, c] = 1.0
        for n in tied_sole:
            T[3 * n, m] = 1.0
            T[3 * n, m + 1] = xy(n)[0] - x0
            T[3 * n + 2, m + 1] = -1.0  # ty = -dw/dx: the element's shear strain is w,x + ty (fixed 2026-11-06: the tie was +1, which put a shear strain 2 theta in the whole sole)
        T = T.tocsr()
        Kr = (T.T @ Kc @ T).tocsc()
        Fr = np.zeros(m + 2)
        Fr[m + 1] = moment if moment else 0.0
        z = spl.spsolve(Kr, Fr)
        return None, z[m + 1]

    result = {}
    # front tie: pads 0,1 (x=102) loaded with 1 kN each, sole held
    front = [pads[i] for i, (px, _) in enumerate(PADS) if px == 102.0]
    load_nodes = [(n, 1000.0 / len(p)) for p in front for n in p]
    u, _ = with_constraints(sole, None, load_nodes)
    delta = np.mean([u[3 * n] for p in front for n in p])
    result["front_tie_N_per_um"] = 1000.0 / (delta * 1000.0) if delta != 0 else float("inf")
    # sole tilt: rigid sole, all pads held
    allpads = [n for p in pads for n in p]
    _, th = with_constraints(allpads, sole, [], moment=1e6)  # 1 kN m
    result["sole_tilt_kNm_per_rad"] = 1.0 / th if th else float("inf")
    return result


def main():
    steel = (210e3, 0.30, 7.85e-6)
    alu = (69e3, 0.33, 2.70e-6)
    variants = [
        ("before: welded steel U, 20 mm", before, steel),
        ("after: Al 20 mm flat, windowed", after_flat(20.0), alu),
        ("after: Al 20 mm deck, 25 mm ribs", after_ribbed(20.0, 25.0), alu),
        ("after: steel 20 mm flat, windowed", after_flat(20.0), steel),
        ("after: steel 20 mm deck, 25 mm ribs", after_ribbed(20.0, 25.0), steel),
    ]
    out = {}
    for name, spec, (E, nu, rho) in variants:
        tmap = thickness_map(spec)
        res = solve(tmap, E, nu)
        res["mass_kg"] = float(tmap.sum() * H * H * rho)
        out[name] = res
    if "--json" in sys.argv:
        print(json.dumps(out, indent=2))
    else:
        for name, res in out.items():
            print(f"{name:40s} front tie {res['front_tie_N_per_um']:8.2f} N/um   sole tilt {res['sole_tilt_kNm_per_rad']:9.1f} kN m/rad   mass {res['mass_kg']:5.1f} kg")


if __name__ == "__main__":
    main()
