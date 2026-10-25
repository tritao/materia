"""Conditional isolated-column comparison; uses the existing FEM export/deck path.

The rear plate has ideal rigid bolt patches, not a bonded face. Neither the
compound modulus nor bolt slip has been qualified, so this cannot select a
production rear plate or certify machine accuracy.
"""

import argparse
import json
import math
from pathlib import Path

from ccx_deck import (element_lines, equation_lines, node_lines, nset_lines,
                      rigid_equations, run_solver, tet10_order)
from frame_solver import read_energies
from packet import digest, elastic_input
from solver_fixtures import read_table


def solve(gmsh, packet_path, config, size, plated, ccx, output):
    packet = json.loads(packet_path.read_text())
    source, plate = packet["source"], packet["plate"]
    gmsh.clear()
    gmsh.model.add("column-torsion")
    gmsh.option.setNumber("General.NumThreads", 1)
    gmsh.option.setNumber("Mesh.MeshSizeMax", size)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 3)
    gmsh.option.setNumber("Mesh.ElementOrder", 2)
    # Straight-sided quadratic elements avoid curved mid-edge inversions in
    # the small nominal anchor/receiver details. Boundary vertices still come
    # from the exact STEP; circle resolution and material-volume error are
    # reported independently of displacement refinement.
    gmsh.option.setNumber("Mesh.SecondOrderLinear", 1)
    gmsh.option.setNumber("Mesh.MinimumCirclePoints", 24)
    gmsh.option.setNumber("Mesh.HighOrderOptimize", 0)
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
    path = packet_path.parent / source["stepFile"]
    if digest(path) != source["stepSha256"]:
        raise ValueError("Column STEP fingerprint mismatch")
    bodies = {"CASTING": [tag for dim, tag in gmsh.model.occ.importShapes(str(path)) if dim == 3]}
    if len(bodies["CASTING"]) != 1:
        raise ValueError("Expected the exact connected source casting")
    if plated:
        bodies["PLATE"] = [gmsh.model.occ.addBox(
            -plate["widthMm"] / 2, plate["rearYMm"], plate["lowerZMm"],
            plate["widthMm"], plate["thicknessMm"], plate["upperZMm"] - plate["lowerZMm"])]
    # Keep separate meshes at the plate/casting contact: only bolt patches tie
    # the two bodies. A shared face would silently assume a bonded rear plate.
    gmsh.model.occ.synchronize()
    # Resolve the same physical bolt patches at both global mesh sizes, in both
    # cases. Their area must not grow merely to find three coarse-mesh nodes.
    fields = []
    for x in (-plate["widthMm"]/2+plate["boltEdgeMm"], plate["widthMm"]/2-plate["boltEdgeMm"]):
        for row in range(plate["boltRows"]):
            z = plate["lowerZMm"] + plate["boltEdgeMm"] + row * (
                plate["upperZMm"]-plate["lowerZMm"]-2*plate["boltEdgeMm"])/(plate["boltRows"]-1)
            field = gmsh.model.mesh.field.add("Ball")
            for option, value in {"XCenter":x, "YCenter":plate["rearYMm"], "ZCenter":z,
                                  "Radius":2*plate["patchRadiusMm"], "VIn":plate["patchRadiusMm"]/3, "VOut":size}.items():
                gmsh.model.mesh.field.setNumber(field, option, value)
            fields.append(field)
    background = gmsh.model.mesh.field.add("Min")
    gmsh.model.mesh.field.setNumbers(background, "FieldsList", fields)
    gmsh.model.mesh.field.setAsBackgroundMesh(background)
    gmsh.model.mesh.generate(3)
    tags, coordinates, _ = gmsh.model.mesh.getNodes()
    nodes = {int(n): tuple(float(v) for v in coordinates[3*i:3*i+3]) for i, n in enumerate(tags)}
    elements, member_nodes, minimum_jacobian, volume_errors = {}, {}, math.inf, {}
    order = tet10_order(gmsh)
    for member, volumes in bodies.items():
        rows, mesh_volume = [], 0.0
        for volume in volumes:
            types, tags, connections = gmsh.model.mesh.getElements(3, volume)
            if list(types) != [11]:
                raise ValueError("Expected quadratic tetrahedra")
            rows += [(int(tag), [int(connections[0][10*i+j]) for j in order]) for i, tag in enumerate(tags[0])]
            minimum_jacobian = min(minimum_jacobian, min(gmsh.model.mesh.getElementQualities(tags[0], "minDetJac")))
            mesh_volume += sum(gmsh.model.mesh.getElementQualities(tags[0], "volume"))
        exact_volume = sum(gmsh.model.occ.getMass(3, volume) for volume in volumes)
        volume_errors[member] = abs(mesh_volume/exact_volume-1)
        if volume_errors[member] > .01:
            raise ValueError(f"Column material-volume discretization error exceeds 1%: {member}")
        elements[member] = rows
        member_nodes[member] = {n for _, connectivity in rows for n in connectivity}
    if minimum_jacobian <= 0:
        raise ValueError("Inverted column element")
    lines = ["*HEADING", "Conditional column torsion; mm N tonne s"]
    equations = []

    def reference(ids, point):
        translation, rotation = max(nodes)+1, max(nodes)+2
        nodes[translation] = nodes[rotation] = point
        equations.extend(rigid_equations(nodes, sorted(ids), translation, rotation, point, (1, 2, 3)))
        return translation, rotation

    casting = member_nodes["CASTING"]
    bottom = {n for n in casting if abs(nodes[n][2]-source["boundsMm"]["minZ"]) < 1e-5}
    top = {n for n in casting if abs(nodes[n][2]-source["boundsMm"]["maxZ"]) < 1e-5}
    if len(bottom) < 3 or len(top) < 3:
        raise ValueError("Missing clamped sole/top diaphragm")
    root, root_rotation = max(nodes)+1, max(nodes)+2
    nodes[root] = nodes[root_rotation] = (0, 0, source["boundsMm"]["minZ"])
    tip, tip_rotation = reference(top, (0, 0, source["boundsMm"]["maxZ"]))
    patch_counts, used = [], set()
    if plated:
        for x in (-plate["widthMm"]/2+plate["boltEdgeMm"], plate["widthMm"]/2-plate["boltEdgeMm"]):
            for row in range(plate["boltRows"]):
                z = plate["lowerZMm"] + plate["boltEdgeMm"] + row * (
                    plate["upperZMm"]-plate["lowerZMm"]-2*plate["boltEdgeMm"])/(plate["boltRows"]-1)
                point = (x, plate["rearYMm"], z)
                patches = []
                for member in ("CASTING", "PLATE"):
                    surface = {n for n in member_nodes[member] if abs(nodes[n][1]-point[1]) < 1e-5}
                    ids = {n for n in surface if math.hypot(nodes[n][0]-x, nodes[n][2]-z) <= plate["patchRadiusMm"]}
                    if len(ids) < 3:
                        raise ValueError(f"Resolve the declared bolt patch with a finer mesh: {member}, {point}")
                    if used & ids:
                        raise ValueError("Bolt patches overlap")
                    used.update(ids)
                    patches.append(ids)
                if patches[0] & patches[1]:
                    raise ValueError("Rear plate accidentally shares bonded nodes")
                reference(patches[0] | patches[1], point)
                patch_counts.append([len(ids) for ids in patches])
    lines += node_lines(nodes)
    for member, rows in elements.items():
        material = config["materials"]["mineral-cast" if member == "CASTING" else "machined-steel"]
        elastic = elastic_input(material["youngsPa"], material["poissonRatio"], material["densityKgPerM3"])
        lines += element_lines(rows, member)
        lines += [f"*MATERIAL,NAME={member}", "*ELASTIC", f"{elastic['youngsMPa']},{elastic['poissonRatio']}",
                  f"*SOLID SECTION,ELSET={member},MATERIAL={member}"]
    lines += equation_lines(equations)
    lines += nset_lines("SOLE", bottom)
    lines += [f"*RIGID BODY,NSET=SOLE,REF NODE={root},ROT NODE={root_rotation}"]
    lines += nset_lines("OBS", [tip_rotation]) + nset_lines("SUPPORT", [root_rotation])
    lines += ["*BOUNDARY", f"{root},1,3", f"{root_rotation},1,3", "*STEP", "*STATIC",
              "*CLOAD", f"{tip_rotation},3,1", "*NODE PRINT,NSET=OBS", "U", "*NODE PRINT,NSET=SUPPORT", "RF"]
    for member in elements:
        lines += [f"*EL PRINT,ELSET={member},TOTALS=ONLY", "ELSE"]
    lines += ["*END STEP"]
    output.mkdir(parents=True, exist_ok=True)
    deck = output / "frame.inp"
    deck.write_text("\n".join(lines)+"\n")
    run_solver(ccx, output, 900)
    text = (output / "frame.dat").read_text()
    theta = read_table(text, "U")[tip_rotation][2]
    reaction = read_table(text, "RF")[root_rotation][2]
    energy = sum(read_energies(text, 1, list(elements)).values())
    if theta <= 0 or abs(reaction+1) > 1e-4 or abs(2*energy/theta-1) > 1e-4:
        raise ValueError(f"Column torque/energy balance failed: theta={theta}, reaction={reaction}, energy={energy}")
    return {"meshSizeMm": size, "stiffnessNmmPerRad": 1/theta, "rotationRadPerNmm": theta,
            "reactionNmm": reaction, "strainEnergyNmm": energy, "minimumJacobian": minimum_jacobian,
            "elements": sum(map(len, elements.values())), "materialVolumeRelativeErrors": volume_errors, "boltPatchNodes": patch_counts,
            "deckSha256": digest(deck), "resultSha256": digest(output / "frame.dat")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("packet", type=Path)
    parser.add_argument("--ccx", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sizes", type=float, nargs="+", default=[24, 18])
    args = parser.parse_args()
    import gmsh
    config = json.loads((Path(__file__).parent / "frame-study.json").read_text())
    packet = json.loads(args.packet.read_text())
    results = {}
    gmsh.initialize()
    try:
        for plated in (False, True):
            name = "ideal-bolted-rear-plate" if plated else "open"
            results[name] = [solve(gmsh, args.packet, config, size, plated, args.ccx,
                                   args.output / name / str(size)) for size in args.sizes]
    finally:
        gmsh.finalize()
    for name, rows in results.items():
        if len(rows) < 2 or abs(rows[-1]["stiffnessNmmPerRad"] / rows[-2]["stiffnessNmmPerRad"]-1) > .05:
            raise ValueError(f"Column comparison needs further refinement: {name}")
    report = {"status": "conditional comparison; physical qualification Pending", "sourceSha256": packet["source"]["stepSha256"],
              "materials": config["materials"], "plate": packet["plate"], "boundary": packet["boundary"], "limitations": packet["limitations"],
              "results": results, "plateToOpenStiffnessRatio": results["ideal-bolted-rear-plate"][-1]["stiffnessNmmPerRad"] /
              results["open"][-1]["stiffnessNmmPerRad"], "productionSource": "open; ideal bolt patches are not physical evidence"}
    (args.output / "comparison.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
