"""Mesh each mineral casting independently; no machine contacts or solve are created."""

import argparse
import importlib
import json
import math
from pathlib import Path

from packet import digest, validate

GMSH_VERSION = "4.12.2"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--size", type=float, default=30, help="Maximum pilot element size in mm")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "build/fem/mesh-pilot")
    args = parser.parse_args()
    report = args.output / "report.json"
    report.unlink(missing_ok=True)
    gmsh = None
    try:
        validated = validate(args.manifest)
        if not math.isfinite(args.size) or args.size <= 0:
            raise ValueError("Mesh size must be finite and positive")
        gmsh = importlib.import_module("gmsh")
        if gmsh.__version__ != GMSH_VERSION:
            raise ValueError(f"Expected Gmsh {GMSH_VERSION}, got {gmsh.__version__}")
        gmsh.initialize()
        gmsh.option.setNumber("General.NumThreads", 1)
        gmsh.option.setNumber("Mesh.MeshSizeMax", args.size)
        gmsh.option.setNumber("Mesh.MeshSizeMin", min(4.0, args.size))
        gmsh.option.setNumber("Mesh.ElementOrder", 2)
        gmsh.option.setNumber("Mesh.HighOrderOptimize", 1)
        gmsh.option.setNumber("Mesh.SaveAll", 0)
        packet = json.loads(args.manifest.read_text())
        args.output.mkdir(parents=True, exist_ok=True)
        cases = []
        for name in ("base", "column"):
            part = next(p for p in packet["parts"] if p["id"] == f"frame/{name}/casting")
            gmsh.clear()
            gmsh.model.add(name)
            imported = gmsh.model.occ.importShapes(str(args.manifest.resolve().parent / part["stepFile"]))
            gmsh.model.occ.synchronize()
            volumes = [tag for dim, tag in imported if dim == 3]
            if len(volumes) != 1:
                raise ValueError(f"Expected one imported solid for {name}")
            tag = volumes[0]
            volume = gmsh.model.occ.getMass(3, tag)
            if not math.isclose(volume, part["volumeMm3"], rel_tol=1e-7):
                raise ValueError(f"CAD volume changed on STEP import: {name}")
            actual = gmsh.model.getBoundingBox(3, tag)
            expected = [part["boundsMm"][s + a] for s in ("min", "max") for a in "XYZ"]
            if any(abs(a - b) > 1e-4 for a, b in zip(actual, expected)):
                raise ValueError(f"Posed CAD bounds changed on STEP import: {name}")
            group = gmsh.model.addPhysicalGroup(3, volumes)
            gmsh.model.setPhysicalName(3, group, part["id"])
            gmsh.model.mesh.generate(3)
            types, element_tags, _ = gmsh.model.mesh.getElements(3)
            if list(types) != [11]:  # Gmsh type 11 is the ten-node tetrahedron.
                raise ValueError(f"Expected only quadratic tetrahedra: {name}")
            tags = element_tags[0]
            if not len(tags):
                raise ValueError(f"Empty volume mesh: {name}")
            quality = gmsh.model.mesh.getElementQualities(tags, "minDetJac")
            if any(not math.isfinite(q) or q <= 0 for q in quality):
                raise ValueError(f"Inverted or degenerate high-order element: {name}")
            mesh_volume = sum(gmsh.model.mesh.getElementQualities(tags, "volume"))
            if not math.isclose(mesh_volume, volume, rel_tol=1e-3):
                raise ValueError(f"Mesh/CAD volume discrepancy: {name}")
            path = args.output / f"{name}.msh"
            gmsh.write(str(path))
            cases.append({"member": part["id"], "stepSha256": part["stepSha256"],
                          "cadVolumeMm3": volume, "meshVolumeMm3": mesh_volume,
                          "elements": len(tags), "minimumJacobianDeterminant": float(min(quality)),
                          "meshFile": path.name, "meshSha256": digest(path)})
        result = {"status": "casting meshing pilot passed; castings are independent; mill not solved",
                  "gmshVersion": gmsh.__version__, "gmshApiSha256": digest(Path(gmsh.__file__)),
                  "gmshLibrarySha256": digest(Path(gmsh.libpath)),
                  "meshGeneratorSha256": digest(Path(__file__)), "units": "mm",
                  "manifestSha256": validated["manifestSha256"], "maxElementSizeMm": args.size,
                  "element": "ten-node tetrahedron", "cases": cases}
        report.write_text(json.dumps(result, indent=2) + "\n")
        print(f"Casting meshing pilot passed; report: {report}")
    except Exception as error:  # Gmsh's Python binding raises plain Exception for native errors.
        parser.exit(1, f"FEM mesh pilot failed: {error}\n")
    finally:
        if gmsh is not None and gmsh.isInitialized():
            gmsh.finalize()


if __name__ == "__main__":
    main()
