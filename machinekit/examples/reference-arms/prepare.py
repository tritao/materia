#!/usr/bin/env python3
"""Fetch pinned official descriptions and compile meshes/URDF into a local SI manifest.

No ROS installation or hardware driver is required. Vendor files stay in build/.
URDF origins, inertia rotations and mesh scales are applied without axis snapping.
"""
import argparse
import concurrent.futures
import hashlib
import json
import re
from pathlib import Path
import tarfile
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parent
SOURCES = json.loads((ROOT / "sources.json").read_text())


def wire_manifest(value):
    """Generate haxeon's typed JSON representation from the consumer's @:wire schema."""
    schemas = {}
    for path in (ROOT / "ReferenceRobot.hx", ROOT.parents[2] / "projectkit/src/materia/assembly/AssemblyRecord.hx"):
        for name, body in re.findall(r"@:wire typedef (\w+) = \{(.*?)\n\}", path.read_text(), re.S):
            schemas[name] = re.findall(r"@:id\((\d+)\)\s+(@:optional\s+)?var (\w+):([^;]+);", body)
    def convert(item, kind):
        if item is None:
            return None
        if kind.startswith("Null<"):
            return convert(item, kind[5:-1])
        if kind.startswith("Array<"):
            return [convert(v, kind[6:-1]) for v in item]
        if kind not in schemas:
            return item
        return [[f"{index}:{name}", convert(item[name], field_type)]
                for index, optional, name, field_type in schemas[kind] if not optional or name in item]
    return {"version": 1, "value": convert(value, "ReferenceManifest")}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def mesh_relative(filename, package, package_name):
    prefix = f"package://{package_name}/"
    if filename.startswith(prefix):
        path = package / filename[len(prefix):]
    elif filename.startswith("../meshes/"):
        path = package / "urdf" / filename
    else:
        raise ValueError(f"Unsupported mesh URI: {filename}")
    return str(path.resolve().relative_to(package.resolve()))


def fetch(url, path, expected=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        temporary = path.with_suffix(path.suffix + ".partial")
        with urllib.request.urlopen(url, timeout=120) as source, temporary.open("wb") as target:
            import shutil
            shutil.copyfileobj(source, target)
        temporary.replace(path)
    if expected and digest(path) != expected:
        raise ValueError(f"Source digest mismatch: {path}")
    return path


def numbers(text, count, default=None):
    values = list(map(float, text.split())) if text is not None else default
    if values is None or len(values) != count or not np.isfinite(values).all():
        raise ValueError(f"Expected {count} finite numbers: {text}")
    return values


def origin(element):
    if element is None:
        return np.eye(4)
    matrix = trimesh.transformations.euler_matrix(*numbers(element.get("rpy"), 3, [0, 0, 0]), axes="sxyz")
    matrix[:3, 3] = numbers(element.get("xyz"), 3, [0, 0, 0])
    return matrix


def frame(matrix):
    w, x, y, z = trimesh.transformations.quaternion_from_matrix(matrix)
    return dict(zip(["x", "y", "z", "qx", "qy", "qz", "qw"],
                    [*map(float, matrix[:3, 3]), float(x), float(y), float(z), float(w)]))


def ur_package():
    spec = SOURCES["ur"]
    archive = fetch(f"https://codeload.github.com/{spec['repository']}/tar.gz/{spec['revision']}",
                    ROOT / "build/sources/ur.tar.gz", spec["archiveSha256"])
    directory = ROOT / "build/sources/ur"
    if not (directory / "package.xml").exists():
        directory.mkdir(parents=True, exist_ok=True)
        with tarfile.open(archive) as source:
            for member in source.getmembers():
                relative = Path(*Path(member.name).parts[1:])
                if not member.isfile():
                    continue
                target = directory / relative
                if not target.resolve().is_relative_to(directory.resolve()):
                    raise ValueError("Unsafe archive path")
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(source.extractfile(member).read())
    return directory


def description(family, model):
    spec = SOURCES[family]
    used = {}
    if family == "ur":
        import xacro
        import xacro.substitution_args
        package = ur_package()
        # Resolve exactly this package, without rewriting vendor Xacro files.
        previous = xacro.substitution_args._eval_find
        def resolve(name):
            if name != "ur_description":
                raise ValueError(f"Unexpected Xacro package: {name}")
            return str(package)
        xacro.substitution_args._eval_find = resolve
        try:
            document = xacro.process_file(str(package / "urdf/ur.urdf.xacro"),
                                          mappings={"name": model, "ur_type": model})
        finally:
            xacro.substitution_args._eval_find = previous
        xml = document.toxml()
        for path in sorted(package.glob("urdf/**/*.xacro")):
            used[str(path.relative_to(package))] = digest(path)
        for path in sorted((package / "config" / model).glob("*.yaml")):
            used[str(path.relative_to(package))] = digest(path)
    else:
        package = ROOT / "build/sources/fairino_description"
        base = f"https://raw.githubusercontent.com/{spec['repository']}/{spec['revision']}/fairino_description/"
        path = fetch(base + f"urdf/{model}.urdf", package / f"urdf/{model}.urdf")
        fetch(base + "package.xml", package / "package.xml")
        used["package.xml"] = digest(package / "package.xml")
        used[f"urdf/{model}.urdf"] = digest(path)
        xml = path.read_text()
    robot = ET.fromstring(xml)
    paths = set()
    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename", "")
        relative = mesh_relative(filename, package, spec["package"])
        paths.add(relative)
    if family == "fairino":
        with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
            list(pool.map(lambda p: fetch(base + p, package / p), sorted(paths)))
    for relative in sorted(paths):
        used[relative] = digest(package / relative)
    return robot, xml, package, used


def geometry(element, package, package_name):
    node = element.find("geometry")
    if node is None or len(node) != 1:
        raise ValueError("Expected one geometry primitive")
    node = node[0]
    if node.tag == "mesh":
        relative = mesh_relative(node.attrib["filename"], package, package_name)
        loaded = trimesh.load(package / relative, force="scene", process=False)
        mesh = loaded.to_mesh()
        mesh.apply_scale(numbers(node.get("scale"), 3, [1, 1, 1]))
    elif node.tag == "box":
        mesh = trimesh.creation.box(numbers(node.get("size"), 3))
    elif node.tag == "cylinder":
        mesh = trimesh.creation.cylinder(float(node.attrib["radius"]), float(node.attrib["length"]), sections=32)
    elif node.tag == "sphere":
        mesh = trimesh.creation.icosphere(subdivisions=2, radius=float(node.attrib["radius"]))
    else:
        raise ValueError(f"Unsupported geometry {node.tag}")
    mesh.apply_transform(origin(element.find("origin")))
    if not len(mesh.faces) or not np.isfinite(mesh.vertices).all():
        raise ValueError("Empty/nonfinite mesh")
    return mesh


def compile_model(family, model):
    spec = SOURCES[family]
    robot, xml, package, used = description(family, model)
    lock = json.loads((ROOT / "source-lock.json").read_text())["models"].get(model)
    if lock is not None and (lock["revision"] != spec["revision"] or lock["files"] != used):
        raise ValueError(f"Pinned source file hashes changed: {model}")
    destination = ROOT / "build" / model
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "robot.urdf").write_text(xml)
    links, joints, source_warnings = [], [], []
    names = [e.attrib["name"] for e in robot.findall("link")]
    if len(names) != len(set(names)):
        raise ValueError("Duplicate link")
    for element in robot.findall("joint"):
        kind = element.attrib["type"]
        if kind not in ("fixed", "revolute", "continuous", "prismatic") or element.find("mimic") is not None:
            raise ValueError(f"Unsupported joint: {kind}")
        limit = element.find("limit")
        limits = {key: float(limit.get(key)) if limit is not None and limit.get(key) is not None else None
                  for key in ("lower", "upper", "velocity", "effort")}
        if kind == "revolute" and (limits["lower"] is None or limits["upper"] is None):
            raise ValueError("Bounded joint needs limits")
        axis_node = element.find("axis")
        axis = np.array(numbers(axis_node.get("xyz") if axis_node is not None else None, 3, [1, 0, 0]), dtype=float)
        if kind == "fixed":
            axis = np.array([1., 0., 0.])  # Fixed-joint axis has no physical meaning.
        if np.linalg.norm(axis) < 1e-12:
            raise ValueError("Zero joint axis")
        axis /= np.linalg.norm(axis)
        joints.append({"id": element.attrib["name"], "type": kind,
                       "parent": element.find("parent").attrib["link"],
                       "child": element.find("child").attrib["link"],
                       "origin": frame(origin(element.find("origin"))),
                       "axis": list(map(float, axis)), "limits": limits})
    roots = set(names) - {j["child"] for j in joints}
    unknown = {j[key] for j in joints for key in ("parent", "child")} - set(names)
    if unknown:
        raise ValueError(f"Source URDF references undeclared links: {sorted(unknown)}")
    if len(roots) != 1 or len({j["child"] for j in joints}) != len(joints):
        raise ValueError("Description must have a single tree root")
    root = roots.pop()
    poses = {root: np.eye(4)}
    pending = joints.copy()
    ordered = []
    while pending:
        eligible = [j for j in pending if j["parent"] in poses]
        if not eligible:
            raise ValueError("Disconnected/cyclic URDF")
        for joint in eligible:
            f = joint["origin"]
            matrix = trimesh.transformations.quaternion_matrix([f["qw"], f["qx"], f["qy"], f["qz"]])
            matrix[:3, 3] = [f["x"], f["y"], f["z"]]
            poses[joint["child"]] = poses[joint["parent"]] @ matrix
            ordered.append(joint)
            pending.remove(joint)
    if set(poses) != set(names):
        raise ValueError("Unreachable link")
    for element in robot.findall("link"):
        name = element.attrib["name"]
        inertial = element.find("inertial")
        mass, center, inertia = None, [0., 0., 0.], None
        if inertial is not None:
            mass = float(inertial.find("mass").attrib["value"])
            f = origin(inertial.find("origin"))
            center = f[:3, 3].tolist()
            a = inertial.find("inertia").attrib
            inertia = np.array([[float(a["i" + r + c]) if "i" + r + c in a else float(a["i" + c + r])
                                 for c in "xyz"] for r in "xyz"])
            inertia = f[:3, :3] @ inertia @ f[:3, :3].T
            if mass <= 0 or not np.isfinite(inertia).all() or np.linalg.eigvalsh(inertia).min() <= 0:
                raise ValueError(f"Invalid source inertia: {name}")
            inertia = inertia.flatten().tolist()
        visuals = element.findall("visual")
        # One part per link, preserving per-visual colors in the mesh records.
        meshes = []
        visual_low, visual_high = np.full(3, np.inf), np.full(3, -np.inf)
        for i, visual in enumerate(visuals):
            mesh = geometry(visual, package, spec["package"])
            visual_low = np.minimum(visual_low, mesh.vertices.min(0))
            visual_high = np.maximum(visual_high, mesh.vertices.max(0))
            stem = f"{name}-{i}"
            mesh.vertices.astype("<f8").tofile(destination / (stem + ".vertices"))
            normals = np.zeros_like(mesh.vertices)
            triangles = mesh.vertices[mesh.faces]
            face_normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
            for column in range(3):
                np.add.at(normals, mesh.faces[:, column], face_normals)
            normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-20)
            normals.astype("<f8").tofile(destination / (stem + ".normals"))
            mesh.faces.astype("<i4").tofile(destination / (stem + ".indices"))
            color_node = visual.find("material/color")
            color = numbers(color_node.get("rgba"), 4) if color_node is not None else [0.65, 0.7, 0.74, 1]
            meshes.append({"stem": stem, "vertexCount": len(mesh.vertices), "indexCount": mesh.faces.size,
                           "color": color, "volume": max(abs(float(mesh.volume)), 1e-12),
                           "sha256": {ext: digest(destination / (stem + "." + ext))
                                      for ext in ("vertices", "normals", "indices")}})
        if meshes and mass is not None and (np.any(np.array(center) < visual_low - 0.005) or
                                           np.any(np.array(center) > visual_high + 0.005)):
            source_warnings.append(f"{name}: source declared centre of mass is outside visual mesh bounds by more than 5 mm; review inertia/frame correspondence before dynamic use.")
        # Keep source collision meshes separate. Convex reduction is the host's responsibility.
        collisions = []
        for i, collision in enumerate(element.findall("collision")):
            mesh = geometry(collision, package, spec["package"])
            stem = f"{name}-collision-{i}"
            mesh.vertices.astype("<f8").tofile(destination / (stem + ".vertices"))
            mesh.faces.astype("<i4").tofile(destination / (stem + ".indices"))
            collisions.append({"stem": stem, "vertexCount": len(mesh.vertices), "indexCount": mesh.faces.size,
                               "sha256": {ext: digest(destination / (stem + "." + ext)) for ext in ("vertices", "indices")}})
        links.append({"id": name, "pose": frame(poses[name]), "massKg": mass,
                      "centerOfMass": center, "inertiaKgM2": inertia, "visuals": meshes, "collisions": collisions})
    moving = [j for j in ordered if j["type"] != "fixed"]
    if len(moving) != 6:
        raise ValueError(f"Expected six moving joints, got {len(moving)}")
    tip = "tool0" if family == "ur" else "tool_Link" if "tool_Link" in names else "wrist3_Link" if "wrist3_Link" in names else "wrist3_link"
    if tip not in names:
        raise ValueError("Missing expected tip frame")
    # Independent source-XML matrix FK oracle, including single-axis jogs.
    coordinates = [[0.] * 6]
    for axis in range(6):
        q = [0.] * 6
        q[axis] = 0.2
        coordinates.append(q)
    for sample in range(32):
        coordinates.append([(j["limits"]["lower"] if j["limits"]["lower"] is not None else -np.pi) +
                            (0.5 + 0.35 * np.sin((sample + 1) * (i + 1) * 1.61803398875)) *
                            ((j["limits"]["upper"] if j["limits"]["upper"] is not None else np.pi) -
                             (j["limits"]["lower"] if j["limits"]["lower"] is not None else -np.pi))
                            for i, j in enumerate(moving)])
    probes = []
    xml_joints = {e.attrib["name"]: e for e in robot.findall("joint")}
    for q in coordinates:
        joint_values = dict(zip([j["id"] for j in moving], q))
        world = {root: np.eye(4)}
        for joint in ordered:
            e = xml_joints[joint["id"]]
            motion = np.eye(4)
            if joint["type"] != "fixed":
                value = joint_values[joint["id"]]
                if joint["type"] == "prismatic":
                    motion[:3, 3] = np.array(joint["axis"]) * value
                else:
                    motion = trimesh.transformations.rotation_matrix(value, joint["axis"])
            world[joint["child"]] = world[joint["parent"]] @ origin(e.find("origin")) @ motion
        probes.append({"q": q, "tip": frame(world[tip]), "links": [{"id": key, "pose": frame(value)} for key, value in world.items()]})
    result = {"schema": 1, "id": model, "family": family, "root": root, "tip": tip,
              "tipStatus": "vendor tool0" if family == "ur" else f"vendor {tip} origin; flange alignment needs CAD verification",
              "metresPerUnit": 1, "source": spec, "sourceFiles": used,
              "urdfSha256": digest(destination / "robot.urdf"), "links": links, "joints": ordered,
              "probes": probes, "sourceWarnings": source_warnings}
    interfaces = json.loads((ROOT / "interfaces.json").read_text())
    if model in interfaces:
        interface = interfaces[model]
        transform = np.eye(4)
        transform[:3, 3] = interface["flangeOffsetMetres"]
        result["flangeOffset"] = frame(transform)
        result["tipStatus"] = interface["basis"] + " " + interface["meshDiscrepancy"]
        result["interfaceReview"] = interface
    temporary = destination / "manifest.json.partial"
    temporary.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    temporary.replace(destination / "manifest.json")
    (destination / "manifest.wire.json").write_text(json.dumps(wire_manifest(result), allow_nan=False) + "\n")
    print(f"{model}: {len(links)} links, six joints, {sum(len(l['visuals']) for l in links)} visual meshes", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("models", nargs="*", default=["ur5e", "fairino5_v6"])
    parser.add_argument("--all", action="store_true")
    args = parser.parse_args()
    location = ROOT / "build/assets/ReferenceAssetLocation.hx"
    location.parent.mkdir(parents=True, exist_ok=True)
    content = 'class ReferenceAssetLocation { public static inline var DIRECTORY:String = ' + json.dumps(str(ROOT / "build")) + '; }\n'
    if not location.exists() or location.read_text() != content:
        location.write_text(content)
    requested = [m for family in ("ur", "fairino") for m in SOURCES[family]["models"]] if args.all else args.models
    results = []
    for model in requested:
        families = [family for family in ("ur", "fairino") if model in SOURCES[family]["models"]]
        if len(families) != 1:
            raise ValueError(f"Unknown model {model}")
        try:
            compile_model(families[0], model)
            results.append({"model": model, "status": "prepared"})
        except Exception as error:
            if not args.all:
                raise
            results.append({"model": model, "status": "failed", "reason": str(error)})
            print(f"{model}: FAILED: {error}", flush=True)
        (ROOT / "build/preparation-report.json").write_text(json.dumps(results, indent=2) + "\n")
    if any(row["status"] == "failed" for row in results):
        raise SystemExit(1)
