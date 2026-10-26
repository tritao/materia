"""Conditional step-fillet comparison through the existing gmsh/CalculiX path.

The source sole pads are bonded to the casting and the sole is clamped, as an
ideal support bound. Preload is the source's class-8.8 policy value. The bending
force comes from frame-study.json; elastic inputs match the torsion study.
Peaks are integration-point stresses in elements bordering the actual fillets;
no compound strength, contact, shrinkage or thermal-cycle capacity is inferred.
"""

import argparse
import gc
import itertools
import json
import math
from pathlib import Path
import re

from ccx_deck import (element_lines, equation_lines, node_lines, nset_lines,
                      rigid_equations, run_solver, tet10_order)
from compliance import symmetric_eigensystem
from frame_solver import read_energies, read_results
from packet import digest, elastic_input


def stress_matrix(values):
    xx, yy, zz, xy, xz, yz = values
    return [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]


def read_stresses(text):
    tables, active = {}, None
    pattern = re.compile(r"\s*stresses\s+\((.*?)\) for set CORNER and time\s+(\S+)", re.I)
    for line in text.splitlines():
        match = pattern.match(line)
        if match:
            if re.findall(r"s[xyz]{2}", match[1].lower()) != ["sxx", "syy", "szz", "sxy", "sxz", "syz"]:
                raise ValueError("Unknown stress tensor column order")
            step = int(round(float(match[2].replace("D", "E"))))
            if step in tables:
                raise ValueError("Duplicate stress result table")
            active = tables[step] = {}
        elif active is not None:
            row = line.split()
            if len(row) == 8 and row[0].isdigit() and row[1].isdigit():
                key = int(row[0]), int(row[1])
                values = tuple(float(v.replace("D", "E")) for v in row[2:])
                if key in active or not all(map(math.isfinite, values)):
                    raise ValueError("Invalid integration-point stress")
                active[key] = values
            elif active:
                active = None
    if set(tables) != {1, 2, 3} or not tables[1] or any(set(tables[n]) != set(tables[1]) for n in (2, 3)):
        raise ValueError("Incomplete three-basis stress results")
    return tables


def mesh(gmsh, packet_path, ratio, bulk_size):
    packet = json.loads(packet_path.read_text())
    radius = packet["radiusMm"]
    pose = packet["sourcePoseMm"]
    if any(abs(pose[k] - v) > 1e-9 for k, v in dict(x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1).items()):
        raise ValueError("Corner packet must use the source-local upright frame")
    gmsh.clear()
    gmsh.model.add("column-foot-corners")
    for option, value in {"General.NumThreads": 1, "Mesh.MeshSizeMax": bulk_size,
                          "Mesh.MeshSizeMin": min(radius / ratio, 1.5), "Mesh.ElementOrder": 2,
                          "Mesh.SecondOrderLinear": 1, "Mesh.MinimumCirclePoints": 32,
                          "Mesh.HighOrderOptimize": 0, "Mesh.MeshSizeExtendFromBoundary": 0,
                          "Mesh.MeshSizeFromPoints": 0, "Mesh.MeshSizeFromCurvature": 0}.items():
        gmsh.option.setNumber(option, value)
    bodies = [packet["source"]] + packet["pads"]
    imported = []
    for body in bodies:
        path = packet_path.parent / body["stepFile"]
        if digest(path) != body["stepSha256"]:
            raise ValueError("STEP fingerprint mismatch: " + body["id"])
        volumes = [pair for pair in gmsh.model.occ.importShapes(str(path)) if pair[0] == 3]
        if len(volumes) != 1:
            raise ValueError("Expected one connected source body per STEP")
        imported += volumes
    _, mapped = gmsh.model.occ.fragment(imported[:1], imported[1:])
    gmsh.model.occ.synchronize()
    assigned = [tag for group in mapped for dim, tag in group if dim == 3]
    if len(set(assigned)) != len(assigned):
        raise ValueError("Casting and pad interiors overlap")
    groups = {"CASTING": [tag for dim, tag in mapped[0] if dim == 3],
              "STEEL": [tag for group in mapped[1:] for dim, tag in group if dim == 3]}
    pad_volumes = {body["id"]: [tag for dim, tag in group if dim == 3]
                   for body, group in zip(packet["pads"], mapped[1:])}
    # Select the two actual cylindrical re-entrant faces, not a proxy stress zone.
    fillets = []
    for dim, tag in gmsh.model.getBoundary([(3, tag) for tag in groups["CASTING"]], oriented=False):
        if dim != 2 or gmsh.model.getType(dim, tag) != "Cylinder":
            continue
        x0, y0, z0, x1, y1, z1 = gmsh.model.getBoundingBox(dim, tag)
        half, height = packet["stemHalfWidthMm"], packet["footHeightMm"]
        if abs(z0-height) < 1e-4 and abs(z1-height-radius) < 1e-4 \
                and abs(min(abs(x0), abs(x1))-half) < 1e-4 \
                and abs(max(abs(x0), abs(x1))-half-radius) < 1e-4:
            fillets.append(tag)
    if len(fillets) != 2:
        raise ValueError(f"Expected two exact step-fillet faces, found {len(fillets)}")
    arcs = {tag for face in fillets for dim, tag in gmsh.model.getBoundary([(2, face)], oriented=False)
            if dim == 1 and gmsh.model.getType(dim, tag) == "Circle"}
    if len(arcs) != 4:
        raise ValueError("Expected the four source quarter-circle edges")
    # Refine the curved boundary too, not only elements along a fixed polygon.
    if ratio > 3:
        segments = math.ceil(8 * 2**(ratio-3))
        for tag in arcs:
            gmsh.model.mesh.setTransfiniteCurve(tag, segments+1)
    fields = []
    def ball(point, extent, size):
        field = gmsh.model.mesh.field.add("Ball")
        for option, value in dict(XCenter=point[0], YCenter=point[1], ZCenter=point[2],
                                  Radius=extent, VIn=size, VOut=bulk_size).items():
            gmsh.model.mesh.field.setNumber(field, option, value)
        fields.append(field)
    low, high = packet["source"]["boundsMm"]["minY"], packet["source"]["boundsMm"]["maxY"]
    count = math.ceil((high-low)/radius)
    for side in (-1, 1):
        for i in range(count+1):
            ball((side*(packet["stemHalfWidthMm"]+radius), low+(high-low)*i/count,
                  packet["footHeightMm"]+radius), 1.5*radius, radius/ratio)
    # Source pads contain cross-drilled fixture and jacking holes; constrain the
    # whole steel volume, not only the bolt-head patches, to resolve that steel.
    for pad in packet["pads"]:
        box = pad["boundsMm"]
        field = gmsh.model.mesh.field.add("Box")
        for option, value in {"VIn": 3., "VOut": bulk_size,
                              "XMin": box["minX"], "XMax": box["maxX"],
                              "YMin": box["minY"], "YMax": box["maxY"],
                              "ZMin": box["minZ"], "ZMax": box["maxZ"]}.items():
            gmsh.model.mesh.field.setNumber(field, option, value)
        fields.append(field)
    for clamp in packet["clamps"]:
        p = clamp["bearingPointMm"]
        ball(tuple(p[k] for k in "xyz"), 1.5*clamp["patchRadiusMm"], 1.5)
    background = gmsh.model.mesh.field.add("Min")
    gmsh.model.mesh.field.setNumbers(background, "FieldsList", fields)
    gmsh.model.mesh.field.setAsBackgroundMesh(background)
    gmsh.model.mesh.generate(3)
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    nodes = {int(tag): tuple(float(v) for v in xyz[3*i:3*i+3]) for i, tag in enumerate(tags)}
    triangles = set()
    for tag in fillets:
        types, _, connectivity = gmsh.model.mesh.getElements(2, tag)
        if list(types) != [9]:
            raise ValueError("Expected quadratic triangular fillet surfaces")
        for i in range(0, len(connectivity[0]), 6):
            triangles.add(tuple(sorted(map(int, connectivity[0][i:i+3]))))
    elements, ids, roi, centers, volume_errors = {}, {}, [], {}, {}
    minimum_jacobian = math.inf
    order = tet10_order(gmsh)
    for label, volumes in groups.items():
        rows, volume = [], 0.0
        for tag in volumes:
            types, tags, connectivity = gmsh.model.mesh.getElements(3, tag)
            if list(types) != [11]:
                raise ValueError("Expected C3D10 tetrahedra")
            rows += [(int(el), [int(connectivity[0][10*i+j]) for j in order]) for i, el in enumerate(tags[0])]
            minimum_jacobian = min(minimum_jacobian, min(gmsh.model.mesh.getElementQualities(tags[0], "minDetJac")))
            volume += sum(gmsh.model.mesh.getElementQualities(tags[0], "volume"))
        exact = sum(gmsh.model.occ.getMass(3, tag) for tag in volumes)
        volume_errors[label] = abs(volume/exact-1)
        if volume_errors[label] > .01:
            raise ValueError(f"Material-volume error exceeds 1%: {label}, mesh={volume}, exact={exact}, relative={volume_errors[label]}")
        elements[label], ids[label] = rows, {n for _, conn in rows for n in conn}
        if label == "CASTING":
            for el, conn in rows:
                if any(tuple(sorted(face)) in triangles for face in itertools.combinations(conn[:4], 3)):
                    roi.append(el)
                    centers[el] = tuple(sum(nodes[n][a] for n in conn[:4])/4 for a in range(3))
    if not roi or minimum_jacobian <= 0:
        raise ValueError("Missing fillet elements or inverted mesh")
    pad_nodes = {}
    for label, volumes in pad_volumes.items():
        pad_nodes[label] = {int(n) for tag in volumes for n in gmsh.model.mesh.getNodes(3, tag, includeBoundary=True)[0]}
    return packet, nodes, elements, ids, pad_nodes, roi, centers, {
        "radiusMm": radius, "filletSizeMm": radius/ratio, "radiusToMeshSize": ratio,
        "bulkSizeMm": bulk_size, "steelSizeMm": 3., "elements": sum(map(len, elements.values())),
        "filletElements": len(roi), "filletFaces": fillets,
        "filletArcSegments": sorted(len(gmsh.model.mesh.getElements(1, tag)[1][0]) for tag in arcs),
        "minimumJacobian": minimum_jacobian,
        "materialVolumeRelativeErrors": volume_errors}


def solve(gmsh, packet_path, config, ratio, bulk_size, ccx, output):
    packet, nodes, elements, ids, pad_nodes, roi, centers, report = mesh(gmsh, packet_path, ratio, bulk_size)
    equations = []
    def reference(selected, point):
        translation, rotation = max(nodes)+1, max(nodes)+2
        nodes[translation] = nodes[rotation] = tuple(point)
        equations.extend(rigid_equations(nodes, sorted(selected), translation, rotation, point, (1, 2, 3)))
        return translation, rotation
    bounds = packet["source"]["boundsMm"]
    top = {n for n in ids["CASTING"] if abs(nodes[n][2]-bounds["maxZ"]) < 1e-5}
    bottom = {n for group in ids.values() for n in group if abs(nodes[n][2]-bounds["minZ"]) < 1e-5}
    if len(top) < 3 or len(bottom) < 3:
        raise ValueError("Missing source sole or top diaphragm")
    root, root_rotation = max(nodes)+1, max(nodes)+2
    nodes[root] = nodes[root_rotation] = (0., 0., bounds["minZ"])
    tip, _ = reference(top, (0., 0., bounds["maxZ"]))
    clamp_refs = []
    for clamp in packet["clamps"]:
        p = tuple(clamp["bearingPointMm"][k] for k in "xyz")
        selected = {n for n in pad_nodes[clamp["memberId"]] if abs(nodes[n][2]-p[2]) < 1e-5
                    and math.hypot(nodes[n][0]-p[0], nodes[n][1]-p[1]) <= clamp["patchRadiusMm"]}
        if len(selected) < 6:
            raise ValueError("Resolve the fixed physical bolt-head patch")
        translation, _ = reference(selected, p)
        clamp_refs.append((translation, clamp["preloadN"]))
    lines = ["*HEADING", "Conditional column step fillet; mm N tonne s"] + node_lines(nodes)
    for label, rows in elements.items():
        material = config["materials"]["mineral-cast" if label == "CASTING" else "machined-steel"]
        elastic = elastic_input(material["youngsPa"], material["poissonRatio"], material["densityKgPerM3"])
        lines += element_lines(rows, label) + [f"*MATERIAL,NAME={label}", "*ELASTIC",
                 f"{elastic['youngsMPa']},{elastic['poissonRatio']}", f"*SOLID SECTION,ELSET={label},MATERIAL={label}"]
    lines += ["*ELSET,ELSET=CORNER"] + [", ".join(map(str, roi[i:i+12])) for i in range(0, len(roi), 12)]
    lines += equation_lines(equations) + nset_lines("SOLE", bottom)
    lines += [f"*RIGID BODY,NSET=SOLE,REF NODE={root},ROT NODE={root_rotation}"]
    loaded = [tip] + [n for n, _ in clamp_refs]
    lines += nset_lines("OBS", loaded) + nset_lines("SUPPORTS", [root, root_rotation])
    lines += ["*BOUNDARY", f"{root},1,3", f"{root_rotation},1,3"]
    cases = [[(n, 3, -value) for n, value in clamp_refs], [(tip, 1, 1.)], [(tip, 2, 1.)]]
    for loads in cases:
        lines += ["*STEP", "*STATIC,SOLVER=SPOOLES", "*CLOAD,OP=NEW"] + [f"{n},{dof},{value:.12g}" for n, dof, value in loads]
        lines += ["*NODE PRINT,NSET=OBS", "U", "*NODE PRINT,NSET=SUPPORTS", "U,RF",
                  "*EL PRINT,ELSET=CORNER", "S"]
        for label in elements:
            lines += [f"*EL PRINT,ELSET={label},TOTALS=ONLY", "ELSE"]
        lines += ["*END STEP"]
    output.mkdir(parents=True, exist_ok=True)
    deck = output / "frame.inp"
    deck.write_text("\n".join(lines)+"\n")
    labels = tuple(elements)
    reference_points = {n: nodes[n] for n in [root, tip]+[ref for ref, _ in clamp_refs]}
    # The external factorization needs memory; retain only the small result
    # locations and load references, not a second in-memory copy of the mesh.
    gmsh.clear()
    del lines, nodes, elements, ids, pad_nodes, roi, top, bottom, equations, loaded
    gc.collect()
    run_solver(ccx, output, 1800)
    text = (output / "frame.dat").read_text()
    tables, energies, stresses = read_results(text, 3), read_energies(text, 3, list(labels)), read_stresses(text)
    balances = []
    for step, loads in enumerate(cases, 1):
        work = sum(value*tables[(step, "displacements", "OBS")][n][dof-1] for n, dof, value in loads)
        energy = sum(energies[(step, label)] for label in labels)
        force, moment = [0.]*3, [0.]*3
        force_scale, moment_scale = 0., 0.
        for n, dof, value in loads:
            f = [value if i == dof-1 else 0. for i in range(3)]
            x, y, z = (reference_points[n][i]-reference_points[root][i] for i in range(3))
            m = [y*f[2]-z*f[1], z*f[0]-x*f[2], x*f[1]-y*f[0]]
            force = [force[i]+f[i] for i in range(3)]; moment = [moment[i]+m[i] for i in range(3)]
            force_scale += abs(value); moment_scale += math.sqrt(sum(v*v for v in m))
        reactions = tables[(step, "forces", "SUPPORTS")]
        f_error = math.sqrt(sum((force[i]+reactions[root][i])**2 for i in range(3)))/max(1., force_scale)
        m_error = math.sqrt(sum((moment[i]+reactions[root_rotation][i])**2 for i in range(3)))/max(1., moment_scale)
        e_error = abs(2*energy/work-1) if work > 0 else math.inf
        if max(f_error, m_error, e_error) > config["equilibriumRelativeTolerance"]:
            raise ValueError(f"Force/moment/energy balance failed: {f_error}, {m_error}, {e_error}")
        balances.append(dict(forceRelativeError=f_error, momentRelativeError=m_error, energyRelativeError=e_error))
    peaks = []
    force = config["cuttingForceN"]
    for fx, fy in [(0., 0.), (force, 0.), (-force, 0.), (0., force), (0., -force)]:
        maximum, minimum, location = -math.inf, math.inf, None
        for key, preload in stresses[1].items():
            values = [preload[i]+fx*stresses[2][key][i]+fy*stresses[3][key][i] for i in range(6)]
            modes = symmetric_eigensystem(stress_matrix(values))
            if modes[0]["value"] > maximum:
                maximum, location = modes[0]["value"], key
            minimum = min(minimum, modes[-1]["value"])
        peaks.append({"bendingForceN": [fx, fy, 0.], "maximumPrincipalMPa": maximum,
                      "minimumPrincipalMPa": minimum, "peakElement": location[0],
                      "peakIntegrationPoint": location[1], "peakElementCentroidMm": centers[location[0]]})
    report.update(cases=peaks, balances=balances, packetSha256=digest(packet_path),
                  deckSha256=digest(deck), resultSha256=digest(output/"frame.dat"))
    (output/"result.json").write_text(json.dumps(report, indent=2)+"\n")
    return report


def write_comparison(output, reports, config):
    refinements = []
    for radius in sorted({row["radiusMm"] for row in reports}):
        levels = sorted((row for row in reports if row["radiusMm"] == radius), key=lambda row: row["radiusToMeshSize"])
        peaks = [max(case["maximumPrincipalMPa"] for case in row["cases"]) for row in levels]
        change = abs(peaks[-1]-peaks[-2])/max(abs(peaks[-1]), 1e-12) if len(peaks)>1 else None
        refinements.append({"radiusMm": radius, "peakPrincipalMPaByLevel": peaks,
                            "relativePeakChange": change,
                            "withinStatedMeshTolerance": change is not None and change <= config["meshRelativeTolerance"]})
    result = {"status": "conditional elastic comparison; physical qualification Pending",
              "solver": "CalculiX SPOOLES, one thread",
              "materials": config["materials"], "bendingForceN": config["cuttingForceN"],
              "stressMeasure": "Integration-point first principal stress in elements bordering the actual step fillets",
              "assumptions": ["clamped sole, bonded source pads and rigid bolt-head patches",
                              "policy assembly preload; linear superposition of preload and +/-X, +/-Y top-diaphragm bending",
                              "other inserts and anchors omit stiffness; source voids retained",
                              "no material strength, shrinkage, thermal-cycle or aggregate-scale qualification"],
              "meshRelativeTolerance": config["meshRelativeTolerance"],
              "refinement": refinements, "meshes": reports}
    output.mkdir(parents=True, exist_ok=True)
    (output/"comparison.json").write_text(json.dumps(result, indent=2)+"\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("packets", type=Path)
    parser.add_argument("--ccx", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("frame-study.json"))
    parser.add_argument("--ratios", type=float, nargs="+", default=[3., 4.])
    parser.add_argument("--bulk-size", type=float, default=25.)
    args = parser.parse_args()
    import gmsh
    config = json.loads(args.config.read_text())
    packets = sorted(args.packets.glob("*/column.json"), key=lambda path: json.loads(path.read_text())["radiusMm"])
    records = [json.loads(path.read_text()) for path in packets]
    if [record["radiusMm"] for record in records] != [3.2, 12]:
        raise ValueError("Expected the approved 3.2 mm and 12 mm source cases")
    if records[0]["clamps"] != records[1]["clamps"]:
        raise ValueError("Clamp holes or policy preload changed between the source cases")
    reports = []
    gmsh.initialize()
    try:
        for packet in packets:
            for ratio in args.ratios:
                target = args.output/packet.parent.name/("ratio-"+str(ratio))
                print("START", packet.parent.name, "radius/mesh", ratio, flush=True)
                report = solve(gmsh, packet, config, ratio, args.bulk_size, args.ccx, target)
                reports.append(report)
                print("DONE", report["radiusMm"], "peak", max(row["maximumPrincipalMPa"] for row in report["cases"]), flush=True)
    finally:
        gmsh.finalize()
    write_comparison(args.output, reports, config)


if __name__ == "__main__":
    main()
