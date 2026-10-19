"""Shared C3D10 input, linear rigid coupling and execution path for fixtures and frame."""

import os
import subprocess


def number(value):
    # CCX free-format fields still have a 20-character limit.
    text = f"{value:.12g}"
    if len(text) > 20:
        raise ValueError("CalculiX numeric field exceeds 20 characters")
    return text


def node_lines(nodes):
    return ["*NODE"] + [str(n) + ", " + ", ".join(number(v) for v in point) for n, point in sorted(nodes.items())]


def element_lines(rows, label):
    lines = [f"*ELEMENT,TYPE=C3D10,ELSET={label}"]
    for n, conn in rows:
        if len(conn) != 10 or len(set(conn)) != 10:
            raise ValueError("C3D10 requires ten distinct nodes")
        lines.append(str(n) + ", " + ", ".join(map(str, conn)))
    return lines


def nset_lines(label, nodes):
    ids = sorted(nodes)
    return [f"*NSET,NSET={label}"] + [", ".join(map(str, ids[i:i+12])) for i in range(0, len(ids), 12)]


def equation_lines(equations):
    lines = ["*EQUATION"]
    for terms in equations:
        lines.append(str(len(terms)))
        for start in range(0, len(terms), 4):
            lines.append(", ".join(f"{n}, {dof}, {number(value)}" for n, dof, value in terms[start:start+4]))
    return lines


def rigid_equations(nodes, ids, reference, rotation, origin, dofs):
    """u(node) = u(ref) + theta cross (node - ref), for selected world components."""
    equations = []
    for n in ids:
        r = tuple(nodes[n][i] - origin[i] for i in range(3))
        coefficients = ((0, -r[2], r[1]), (r[2], 0, -r[0]), (-r[1], r[0], 0))
        for dof in dofs:
            terms = [(n, dof, 1.0), (reference, dof, -1.0)]
            terms += [(rotation, i+1, value) for i, value in enumerate(coefficients[dof-1]) if abs(value) > 1e-12]
            equations.append(terms)
    return equations


def tet10_order(gmsh):
    """Map reference-edge coordinates to CalculiX's C3D10 edge ordering."""
    _, _, _, count, raw, _ = gmsh.model.mesh.getElementProperties(11)
    local = [tuple(raw[i:i+3]) for i in range(0, len(raw), 3)]
    if count != 10:
        raise ValueError("Expected ten-node tetrahedra")
    order = list(range(4))
    for a, b in ((0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)):
        midpoint = tuple((local[a][i] + local[b][i])/2 for i in range(3))
        order.append(next(i for i in range(4, 10) if local[i] == midpoint))
    return order


def run_solver(ccx, output, timeout):
    for suffix in (".dat", ".frd", ".sta", ".cvg"):
        (output / ("frame" + suffix)).unlink(missing_ok=True)
    env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    with (output / "solver.log").open("w") as log:
        result = subprocess.run([str(ccx), "-i", "frame"], cwd=output, env=env,
                                stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
    text = (output / "solver.log").read_text()
    if result.returncode or "*ERROR" in text.upper() or "Job finished" not in text:
        raise ValueError(f"Static solver failed; see {output / 'solver.log'}")
    return text
