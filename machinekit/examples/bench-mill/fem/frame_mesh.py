"""Conformal two-material frame mesh and named load/support faces for the static study."""

import math
from pathlib import Path

from packet import digest
from ccx_deck import tet10_order
from mesh_resolution import measure as measure_resolution, partitions

GMSH_VERSION = "4.12.2"


def xyz(point):
    return tuple(point[a] for a in "xyz")


def rectangle(gmsh, corners):
    points = [gmsh.model.occ.addPoint(*p) for p in corners]
    edges = [gmsh.model.occ.addLine(points[i], points[(i + 1) % 4]) for i in range(4)]
    return gmsh.model.occ.addPlaneSurface([gmsh.model.occ.addCurveLoop(edges)])


def build_mesh(gmsh, packet, manifest, config, size, output):
    gmsh.clear()
    gmsh.model.add("bonded-frame")
    gmsh.option.setNumber("General.NumThreads", 1)
    gmsh.option.setNumber("Mesh.MeshSizeMax", size)
    gmsh.option.setNumber("Mesh.MeshSizeMin", min(4, size))
    gmsh.option.setNumber("Mesh.ElementOrder", 2)
    gmsh.option.setNumber("Mesh.HighOrderOptimize", 1)
    gmsh.option.setNumber("Mesh.SaveAll", 0)
    parts = {p["id"]: p for p in packet["parts"]}
    solids = [p for p in packet["parts"] if p["id"].startswith(("frame/base/", "frame/column/"))]
    imported = []
    for part in solids:
        entities = gmsh.model.occ.importShapes(str(Path(manifest).resolve().parent / part["stepFile"]))
        volumes = [p for p in entities if p[0] == 3]
        if len(volumes) != 1:
            raise ValueError(f"Expected one volume for {part['id']}")
        imported.append(volumes[0])
    tools, patches = [], []
    for guide in packet["guides"]:
        if guide["axis"] == "x":
            continue  # X mechanism is inside the explicitly rigid XY assembly.
        x, y, z = xyz(guide["topMm"])
        width, length = guide["railWidthMm"], guide["blockLengthMm"]
        if guide["axis"] == "y":
            owner = "frame/base/railY" + ("Left" if x < 0 else "Right")
            z = parts[owner]["boundsMm"]["maxZ"]
            corners = [(x - width / 2, y - length / 2, z), (x + width / 2, y - length / 2, z),
                       (x + width / 2, y + length / 2, z), (x - width / 2, y + length / 2, z)]
        else:
            owner = "frame/column/railZ" + ("Left" if x < 0 else "Right")
            y = parts[owner]["boundsMm"]["minY"]
            corners = [(x - width / 2, y, z - length / 2), (x + width / 2, y, z - length / 2),
                       (x + width / 2, y, z + length / 2), (x - width / 2, y, z + length / 2)]
        tools.append((2, rectangle(gmsh, corners)))
        patches.append({"name": guide["member"], "owner": owner, "kind": "guide-" + guide["axis"]})
    # New frames use one strip per side; partition only the original 20 mm
    # upper bearing region, rather than applying drive load over the whole strip.
    for side in ("Left", "Right"):
        if "frame/column/driveUpper" + side in parts:
            continue  # Previously exported split-insert packets remain readable.
        owner = "frame/column/railZ" + side
        box = parts[owner]["boundsMm"]
        x0, x1, y, z1 = box["minX"], box["maxX"], box["minY"], box["maxZ"]
        tools.append((2, rectangle(gmsh, [(x0, y, z1 - 20), (x1, y, z1 - 20),
                                         (x1, y, z1), (x0, y, z1)])))
        patches.append({"name": "drive-z-" + side.lower(), "owner": owner, "kind": "drive-z"})
    for support in packet["supports"]:
        x, y, z = xyz(support["seatMm"])
        r = config["footContactRadiusMm"]
        tools.append((2, gmsh.model.occ.addDisk(x, y, z, r, r)))
        patches.append({"name": "foot-" + support["id"], "owner": "frame/base/foot" + support["id"], "kind": "support"})
    mesh_partitions=partitions(packet)
    tools += [(2,rectangle(gmsh,plane["cornersMm"])) for plane in mesh_partitions]
    _, mapping = gmsh.model.occ.fragment(imported, tools)
    gmsh.model.occ.synchronize()
    owners, volumes_by_part, faces_by_part = {}, {}, {}
    for part, mapped in zip(solids, mapping[:len(solids)]):
        ids = [t for d, t in mapped if d == 3]
        if not ids:
            raise ValueError(f"Fragmentation lost {part['id']}")
        for tag in ids:
            if tag in owners:
                raise ValueError("Overlapping material volumes in frame CAD")
            owners[tag] = part["id"]
        volume = sum(gmsh.model.occ.getMass(3, t) for t in ids)
        if not math.isclose(volume, part["volumeMm3"], rel_tol=1e-7):
            raise ValueError(f"Fragmentation changed material volume for {part['id']}")
        volumes_by_part[part["id"]] = ids
        faces_by_part[part["id"]] = {t for d, t in gmsh.model.getBoundary([(3, t) for t in ids], oriented=False) if d == 2}
        physical = gmsh.model.addPhysicalGroup(3, ids)
        gmsh.model.setPhysicalName(3, physical, part["id"])
    for patch, mapped in zip(patches, mapping[len(solids):]):
        patch["faces"] = sorted({t for d, t in mapped if d == 2} & faces_by_part[patch["owner"]])
    for owner, axis, side, name, kind in (
            ("frame/base/driveYFixed", 2, "maxZ", "drive-y", "drive-y"),
            ("frame/column/driveUpperLeft", 1, "minY", "drive-z-left", "drive-z"),
            ("frame/column/driveUpperRight", 1, "minY", "drive-z-right", "drive-z")):
        if owner not in parts:
            continue  # Continuous-strip drive patches were partitioned above.
        coordinate = parts[owner]["boundsMm"][side]
        faces = []
        for face in faces_by_part[owner]:
            box = gmsh.model.getBoundingBox(2, face)
            if abs(box[axis] - coordinate) < 1e-5 and abs(box[axis + 3] - coordinate) < 1e-5:
                faces.append(face)
        patches.append({"name": name, "owner": owner, "kind": kind, "faces": sorted(faces)})
    bonds = []
    graph = {p["id"]: set() for p in solids}
    for interface in packet["interfaces"]:
        a, b = interface["first"], interface["second"]
        common = faces_by_part[a] & faces_by_part[b]
        area = sum(gmsh.model.occ.getMass(2, face) for face in common)
        if area <= 1e-4:
            raise ValueError(f"No shared bonded face: {a} / {b}")
        graph[a].add(b)
        graph[b].add(a)
        bonds.append({"first": a, "second": b, "areaMm2": area, "faces": sorted(common)})
    visited, pending = set(), ["frame/base/casting"]
    while pending:
        node = pending.pop()
        if node not in visited:
            visited.add(node)
            pending.extend(graph[node] - visited)
    if visited != set(graph):
        raise ValueError(f"Disconnected frame solids: {set(graph) - visited}")
    if "frameLayout" not in packet:
        raise ValueError("Re-export packet with frameLayout for thickness checks")
    # Geometric mid-planes impose two normal layers independently of tangential
    # mesh size; this avoids uniformly refining every long thin casting member.
    gmsh.model.mesh.generate(3)
    node_tags, coord, _ = gmsh.model.mesh.getNodes()
    all_nodes = {int(n): tuple(float(v) for v in coord[3 * i:3 * i + 3]) for i, n in enumerate(node_tags)}
    elements, body_nodes, mass_centres = {}, {}, {}
    order = tet10_order(gmsh)
    minimum_jacobian, element_volume = math.inf, 0.0
    for part in solids:
        member, rows = part["id"], []
        for volume in volumes_by_part[member]:
            types, tags, connectivity = gmsh.model.mesh.getElements(3, volume)
            if list(types) != [11]:
                raise ValueError("Expected C3D10-compatible tetrahedra only")
            for i, tag in enumerate(tags[0]):
                raw = connectivity[0][10 * i:10 * i + 10]
                rows.append((int(tag), tuple(int(raw[j]) for j in order)))
            jac = gmsh.model.mesh.getElementQualities(tags[0], "minDetJac")
            if any(not math.isfinite(v) or v <= 0 for v in jac):
                raise ValueError("Inverted/degenerate frame element")
            minimum_jacobian = min(minimum_jacobian, min(jac))
            element_volume += sum(gmsh.model.mesh.getElementQualities(tags[0], "volume"))
        elements[member] = rows
        body_nodes[member] = {n for _, conn in rows for n in conn}
        mass_centres[member] = xyz(part["centreOfMassMm"])
    for patch in patches:
        nodes = set()
        for face in patch["faces"]:
            tags, _, _ = gmsh.model.mesh.getNodes(2, face, includeBoundary=True)
            nodes.update(int(n) for n in tags)
        nodes &= body_nodes[patch["owner"]]
        if len(nodes) < 6:
            raise ValueError(f"Missing/undersampled named interface: {patch['name']}")
        patch["nodes"] = sorted(nodes)
        patch["areaMm2"] = sum(gmsh.model.occ.getMass(2, f) for f in patch["faces"])
    active = set().union(*body_nodes.values())
    nodes = {n: all_nodes[n] for n in sorted(active)}
    volume = sum(p["volumeMm3"] for p in solids)
    if not math.isclose(element_volume, volume, rel_tol=1e-3):
        raise ValueError("Frame mesh/CAD total volume mismatch")
    resolution=measure_resolution(gmsh,packet,elements)
    output.mkdir(parents=True, exist_ok=True)
    path = output / "frame.msh"
    gmsh.write(str(path))
    return {"partitions":mesh_partitions,"throughThickness":resolution,"nodes": nodes, "elements": elements, "bodyNodes": body_nodes, "patches": patches,
            "bonds": bonds, "minimumJacobianDeterminant": float(minimum_jacobian),
            "volumeMm3": volume, "meshVolumeMm3": float(element_volume), "meshSha256": digest(path),
            "gmshVersion": gmsh.__version__, "gmshLibrarySha256": digest(Path(gmsh.libpath))}
