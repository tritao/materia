"""CalculiX installation/unit checks; these fixtures do not analyze the mill."""

import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess

from packet import SOLVER_UNITS, digest, elastic_input

CCX_VERSION = "2.21"


def brick_mesh(nx, ny, nz, quadratic):
    """Structured 100 x 10 x 10 mm bar, standard C3D8/C3D20 connectivity."""
    offsets = [(0, 0, 0), (2, 0, 0), (2, 2, 0), (0, 2, 0),
               (0, 0, 2), (2, 0, 2), (2, 2, 2), (0, 2, 2)]
    if quadratic:
        offsets += [(1, 0, 0), (2, 1, 0), (1, 2, 0), (0, 1, 0),
                    (1, 0, 2), (2, 1, 2), (1, 2, 2), (0, 1, 2),
                    (0, 0, 1), (2, 0, 1), (2, 2, 1), (0, 2, 1)]
    lattice, nodes, elements = {}, {}, []
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                element = []
                for a, b, c in offsets:
                    key = (2 * i + a, 2 * j + b, 2 * k + c)
                    if key not in lattice:
                        node = len(lattice) + 1
                        lattice[key] = node
                        nodes[node] = (100 * key[0] / (2 * nx),
                                       10 * key[1] / (2 * ny), 10 * key[2] / (2 * nz))
                    element.append(lattice[key])
                elements.append(element)
    return nodes, elements


def fixture(nx=1, ny=1, nz=1, bending=False):
    nodes, elements = brick_mesh(nx, ny, nz, bending)
    fixed = [n for n, p in nodes.items() if p[0] == 0]
    tip = [n for n, p in nodes.items() if p[0] == 100]
    # nu=0 gives a simple analytical tension solution even with the end clamped.
    material = elastic_input(210e9, 0, 7850)
    force, direction = (1.0, 2) if bending else (1000.0, 1)
    lines = ["*HEADING", "Bench mill FEM toolchain fixture; mm N tonne s", "*NODE"]
    lines += [f"{n}, {x:.12g}, {y:.12g}, {z:.12g}" for n, (x, y, z) in nodes.items()]
    lines.append("*ELEMENT, TYPE=" + ("C3D20" if bending else "C3D8") + ", ELSET=BAR")
    for n, element in enumerate(elements, 1):
        # CalculiX permits at most 16 entries per element input line.
        first = [n] + element[:15]
        lines.append(", ".join(map(str, first)) + ("," if len(element) > 15 else ""))
        if len(element) > 15:
            lines.append(", ".join(map(str, element[15:])))
    for name, ids in (("FIXED", fixed), ("TIP", tip)):
        lines.append(f"*NSET, NSET={name}")
        for start in range(0, len(ids), 12):
            lines.append(", ".join(map(str, ids[start:start + 12])))
    lines += ["*MATERIAL, NAME=REFERENCE", "*ELASTIC",
              f"{material['youngsMPa']:.12g}, {material['poissonRatio']}", "*DENSITY",
              f"{material['densityTonnePerMm3']:.12g}",
              "*SOLID SECTION, ELSET=BAR, MATERIAL=REFERENCE", "*BOUNDARY", "FIXED, 1, 3",
              "*STEP", "*STATIC", "*CLOAD"]
    lines += [f"{n}, {direction}, {force / len(tip):.16g}" for n in tip]
    lines += ["*NODE PRINT, NSET=TIP", "U", "*NODE PRINT, NSET=FIXED", "RF", "*END STEP"]
    expected = force * 100**3 / (3 * 210000 * (10 * 10**3 / 12)) if bending else force * 100 / (210000 * 100)
    return "\n".join(lines) + "\n", nodes, fixed, tip, direction - 1, force, expected


def read_table(text, field):
    """Read one fixture's nodal output, refusing missing/duplicate tables."""
    title = "displacements" if field == "U" else "forces"
    result, active, tables = {}, False, 0
    for line in text.splitlines():
        if re.match(rf"\s*{title}\s+\(", line):
            active = True
            tables += 1
            continue
        if active:
            columns = line.split()
            if len(columns) == 4 and columns[0].isdigit():
                node = int(columns[0])
                if node in result:
                    raise ValueError("Duplicate nodal result")
                result[node] = tuple(float(v.replace("D", "E")) for v in columns[1:])
            elif result:
                active = False
    if tables != 1 or not result:
        raise ValueError(f"Expected one nonempty {field} result table, found {tables}")
    return result


def run_case(ccx, directory, name, **settings):
    directory = directory / name
    directory.mkdir(parents=True, exist_ok=True)
    text, nodes, fixed, tip, axis, force, expected = fixture(**settings)
    job = directory / "fixture.inp"
    job.write_text(text)
    # Never consume output left by an earlier solver invocation.
    for suffix in (".dat", ".sta", ".cvg", ".frd"):
        job.with_suffix(suffix).unlink(missing_ok=True)
    env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    run = subprocess.run([str(ccx), "-i", "fixture"], cwd=directory,
                         env=env, text=True, capture_output=True, timeout=120)
    log = run.stdout + run.stderr
    (directory / "solver.log").write_text(log)
    if run.returncode or "*ERROR" in log.upper() or "Job finished" not in log:
        raise ValueError(f"CalculiX failed; see {directory / 'solver.log'}")
    output = job.with_suffix(".dat").read_text()
    u, rf = read_table(output, "U"), read_table(output, "RF")
    if set(u) != set(tip) or set(rf) != set(fixed):
        raise ValueError("Incomplete fixture nodal results")
    displacement = sum(u[n][axis] for n in tip) / len(tip)
    reaction = sum(rf[n][axis] for n in fixed)
    error = abs(displacement - expected) / abs(expected)
    reaction_error = abs(reaction + force) / abs(force)
    if not all(math.isfinite(v) for row in (*u.values(), *rf.values()) for v in row):
        raise ValueError("Non-finite solver result")
    if reaction_error > 1e-5:
        raise ValueError(f"Reaction balance failed for {name}: {reaction_error}")
    applied = {n: tuple(force / len(tip) if i == axis else 0.0 for i in range(3)) for n in tip}
    force_sum, moment_sum = [0.0] * 3, [0.0] * 3
    for node, load in list(rf.items()) + list(applied.items()):
        x, y, z = nodes[node]
        for i in range(3):
            force_sum[i] += load[i]
        for i, moment in enumerate((y * load[2] - z * load[1], z * load[0] - x * load[2],
                                    x * load[1] - y * load[0])):
            moment_sum[i] += moment
    moment_error = math.sqrt(sum(v * v for v in moment_sum)) / (force * 100)
    if math.sqrt(sum(v * v for v in force_sum)) / force > 1e-5 or moment_error > 1e-5:
        raise ValueError(f"Force/moment equilibrium failed for {name}")
    return {"case": name, "elements": settings.get("nx", 1) * settings.get("ny", 1) * settings.get("nz", 1),
            "displacementMm": displacement, "analyticalMm": expected, "relativeError": error,
            "reactionN": reaction, "reactionRelativeError": reaction_error,
            "momentRelativeError": moment_error,
            "inputSha256": digest(job), "outputSha256": digest(job.with_suffix(".dat")),
            "command": [str(ccx), "-i", "fixture"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ccx", default="ccx", help=f"CalculiX {CCX_VERSION} executable")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "build/fem/fixtures")
    args = parser.parse_args()
    report = args.output / "report.json"
    report.unlink(missing_ok=True)
    try:
        executable = shutil.which(args.ccx)
        if not executable:
            raise ValueError(f"CalculiX {CCX_VERSION} not found; pass --ccx /path/to/ccx")
        ccx = Path(executable).resolve()
        version = subprocess.run([str(ccx), "-v"], text=True, capture_output=True, timeout=10)
        # The packaged 2.21 executable returns 201 for its successful -v path.
        if version.returncode not in (0, 201) or not re.search(rf"\bVersion\s+{re.escape(CCX_VERSION)}\b", version.stdout):
            raise ValueError(f"Expected CalculiX {CCX_VERSION}; got {version.stdout + version.stderr}")
        tension = run_case(ccx, args.output, "tension")
        beams = [run_case(ccx, args.output, f"cantilever-{n}", nx=n, ny=2, nz=2, bending=True) for n in (4, 8, 16)]
        if tension["relativeError"] > .01 or beams[-1]["relativeError"] > .05:
            raise ValueError("Fixture displacement exceeds analytical tolerance")
        change = abs(beams[-1]["displacementMm"] - beams[-2]["displacementMm"]) / abs(beams[-1]["displacementMm"])
        if change > .05:
            raise ValueError("Cantilever mesh convergence exceeds 5%")
        result = {"status": "fixture checks passed; mill not solved", "units": SOLVER_UNITS,
                  "solverVersion": version.stdout.strip(), "solverSha256": digest(ccx),
                  "fixtureGeneratorSha256": digest(Path(__file__)), "tension": tension,
                  "cantilever": beams, "lastRefinementRelativeChange": change}
        report.write_text(json.dumps(result, indent=2) + "\n")
        print(f"Fixture checks passed; report: {report}")
    except (ValueError, OSError, subprocess.TimeoutExpired) as error:
        parser.exit(1, f"FEM solver check failed: {error}\n")


if __name__ == "__main__":
    main()
