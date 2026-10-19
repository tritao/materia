"""Validate the version-1 CAD packet before a mesher consumes it (stdlib only)."""

import argparse
import hashlib
import json
import math
from pathlib import Path

CAD_UNITS = {"length": "mm", "force": "N", "mass": "kg", "time": "s",
             "modulus": "Pa", "density": "kg/m3"}
SOLVER_UNITS = {"length": "mm", "force": "N", "mass": "tonne", "time": "s",
                "modulus": "MPa", "density": "tonne/mm3"}


def elastic_input(youngs_pa, poisson_ratio, density_kg_m3):
    """Convert explicit SI material inputs to the mm/N/tonne/s solver system."""
    for value in (youngs_pa, poisson_ratio, density_kg_m3):
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError("Elastic inputs must be finite numbers; no defaults are assigned")
    if youngs_pa <= 0 or density_kg_m3 <= 0 or not -1 < poisson_ratio < 0.5:
        raise ValueError("Invalid isotropic elastic material")
    return {"youngsMPa": youngs_pa * 1e-6, "poissonRatio": poisson_ratio,
            "densityTonnePerMm3": density_kg_m3 * 1e-12}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite(value):
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("Packet contains a non-finite number")
    if isinstance(value, dict):
        for child in value.values():
            finite(child)
    elif isinstance(value, list):
        for child in value:
            finite(child)


def validate(manifest_path):
    manifest_path = Path(manifest_path).resolve()
    packet = json.loads(manifest_path.read_text())
    finite(packet)
    if packet["schemaVersion"] != 1 or packet["units"] != CAD_UNITS:
        raise ValueError("Unsupported packet schema or units")
    if packet["geometryFrame"] != "world" or packet["poseApplication"] != "already-applied":
        raise ValueError("Expected STEP geometry already placed in world coordinates")
    parts = packet["parts"]
    by_id = {p["id"]: p for p in parts}
    materials = {m["id"]: m for m in packet["materials"]}
    if len(by_id) != len(parts) or len(materials) != len(packet["materials"]):
        raise ValueError("Duplicate component or material ID")
    for required in ("frame/base/casting", "frame/column/casting", "xyStage/table",
                     "xyStage/saddle", "zAxis/head", "zAxis/carriage", "spindle/tool"):
        if required not in by_id:
            raise ValueError(f"Missing required component: {required}")
    solids = []
    for part in parts:
        if part["materialId"] not in materials or part["massKg"] <= 0:
            raise ValueError(f"Invalid material or mass: {part['id']}")
        pose = part["poseMm"]
        norm = sum(pose[k] ** 2 for k in ("qx", "qy", "qz", "qw"))
        if abs(norm - 1) > 1e-6:
            raise ValueError(f"Invalid pose quaternion: {part['id']}")
        if part["representation"] == "solid":
            file = Path(part["stepFile"])
            if file.name != str(file) or file.suffix != ".step":
                raise ValueError("STEP path must be a relative filename inside the packet")
            step = manifest_path.parent / file
            if step.resolve().parent != manifest_path.parent:
                raise ValueError("STEP file escapes the packet directory")
            if digest(step) != part["stepSha256"]:
                raise ValueError(f"STEP digest mismatch: {part['id']}")
            if part["volumeMm3"] <= 0 or part["boundsMm"] is None:
                raise ValueError(f"Missing solid volume/bounds: {part['id']}")
            for axis in "XYZ":
                if part["boundsMm"]["min" + axis] >= part["boundsMm"]["max" + axis]:
                    raise ValueError(f"Invalid solid bounds: {part['id']}")
            solids.append(part)
        elif part["representation"] != "mass-only" or part["stepFile"] is not None:
            raise ValueError(f"Unknown component representation: {part['id']}")
    for total, subset in (("totalComponentMassKg", parts), ("solidMassKg", solids)):
        if not math.isclose(packet[total], sum(p["massKg"] for p in subset), rel_tol=1e-9):
            raise ValueError(f"Mass sum does not match {total}")
    for name in ("base", "column"):
        part = by_id[f"frame/{name}/casting"]
        if not math.isclose(part["massKg"], packet["castingMassKg"][name], rel_tol=1e-9):
            raise ValueError(f"Casting mass mismatch: {name}")
        density = materials[part["materialId"]]["densityKgPerM3"]
        if not math.isclose(part["volumeMm3"] * 1e-9 * density, part["massKg"], rel_tol=1e-7):
            raise ValueError(f"Casting volume/density mismatch: {name}")
    for interface in packet["interfaces"]:
        if any(by_id[p]["representation"] != "solid" for p in (interface["first"], interface["second"])):
            raise ValueError("Interface does not reference exported solids")
        if interface["kind"] == "cast-in" and interface["overlapMm3"] > 1e-3:
            raise ValueError("Casting and insert overlap")
    if len(packet["supports"]) != 4 or len({s["id"] for s in packet["supports"]}) != 4:
        raise ValueError("Four distinct support stations are required")
    if len(packet["guides"]) != 12 or any(g["member"] not in by_id for g in packet["guides"]):
        raise ValueError("Twelve guide block datums are required")
    table, payload = packet["datums"]["tableTopMm"], packet["datums"]["workpieceReferenceMm"]
    if not (math.isclose(payload["x"], table["x"], abs_tol=1e-6) and
            math.isclose(payload["y"], table["y"], abs_tol=1e-6) and
            math.isclose(payload["z"] - table["z"], 50, abs_tol=1e-6)):
        raise ValueError("Payload reference must be 50 mm above the table in world Z")
    if any(s["member"] not in by_id for s in packet["supports"]):
        raise ValueError("Unknown support component")
    return {"status": "CAD packet validated; structural inputs remain unassigned",
            "components": len(parts), "solids": len(solids), "massOnly": len(parts) - len(solids),
            "totalComponentMassKg": packet["totalComponentMassKg"],
            "manifestSha256": digest(manifest_path), "pending": packet["pending"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(validate(args.manifest), indent=2))
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f"Invalid FEM packet: {error}\n")


if __name__ == "__main__":
    main()
