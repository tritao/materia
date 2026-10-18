"""Prepare source-backed electrical interface facts without changing vendor geometry."""
import hashlib
import json
from pathlib import Path
from reference_wire import encode


def prepare_catalog(root):
    root = Path(root)
    raw = (root / "wrist-interfaces.json").read_bytes()
    value = json.loads(raw)
    if value["schema"] != 1:
        raise ValueError("Unsupported wrist interface catalog")
    value["sourceSha256"] = hashlib.sha256(raw).hexdigest()
    seen = set()
    for profile in value["profiles"]:
        if profile["id"] in seen:
            raise ValueError("Duplicate wrist profile")
        seen.add(profile["id"])
        for port in profile["ports"]:
            if sorted(p["number"] for p in port["pins"]) != list(range(1, port["contactCount"] + 1)):
                raise ValueError("Incomplete or duplicate wrist pinout")
            binding = port.get("observedGeometry")
            if binding:
                step = root / "build" / binding["sourceStep"]
                # Catalog evidence can be prepared before optional CAD assets or
                # other robot models have been fetched. Audit any sources present;
                # native inspection and robot loading still verify required files.
                if step.exists() and hashlib.sha256(step.read_bytes()).hexdigest() != binding["sourceStepSha256"]:
                    raise ValueError("Changed STEP observation source")
                model = root / "build" / profile["modelId"]
                urdf = model / "robot.urdf"
                if urdf.exists() and hashlib.sha256(urdf.read_bytes()).hexdigest() != binding["parentUrdfSha256"]:
                    raise ValueError("Changed observation parent URDF")
                if binding["parentLink"] != port["parentLink"]:
                    raise ValueError("Observation parent mismatch")
            if port["matingFrame"] is not None and port["geometryStatus"] != "qualified":
                raise ValueError("Unqualified mating location")
    wire = encode(value, "WristInterfaceCatalog", [root / "ReferenceWristInterfaces.hx",
                  root.parents[2] / "projectkit/src/materia/assembly/AssemblyRecord.hx"])
    data = (json.dumps(wire, allow_nan=False) + "\n").encode()
    target = root / "build/assets/wrist-interfaces.wire.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    if not target.exists() or target.read_bytes() != data:
        target.write_bytes(data)
    return hashlib.sha256(data).hexdigest()


def stamp_prepared(root, models=("ur5e", "fairino5_v6")):
    root = Path(root)
    digest = prepare_catalog(root)
    for model in models:
        directory = root / "build" / model
        manifest = json.loads((directory / "manifest.json").read_text())
        urdf = (directory / "robot.urdf").read_bytes()
        if hashlib.sha256(urdf).hexdigest() != manifest["urdfSha256"]:
            raise ValueError("Source URDF changed before interface preparation")
        manifest["wristInterfaceCatalogSha256"] = digest
        wire = encode(manifest, "ReferenceManifest", [root / "ReferenceRobot.hx",
                      root.parents[2] / "projectkit/src/materia/assembly/AssemblyRecord.hx"])
        (directory / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
        (directory / "manifest.wire.json").write_text(json.dumps(wire, allow_nan=False) + "\n")
        print(f"{model}: added wrist interface provenance; source meshes and transforms unchanged")


if __name__ == "__main__":
    stamp_prepared(Path(__file__).resolve().parent)
