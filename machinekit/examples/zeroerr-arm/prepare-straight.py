#!/usr/bin/env python3
"""Derive the straight forearm as a separately cached variant of the 80T arm."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".partial")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def main():
    config_path = ROOT / "straight-link-config.json"
    config = json.loads(config_path.read_text())
    baseline_path = ROOT / config["baselineLayout"]
    baseline = json.loads(baseline_path.read_text())
    source_vendor = ROOT / "build/revised/vendor"
    source = source_vendor / (config["sourceAsset"] + ".STEP")
    if digest(source) != config["sourceSha256"]:
        raise ValueError("Supplier forearm source changed")
    # This kernel implements one reviewed prototype fastener/cover arrangement.
    # Reject unsupported parameter edits instead of silently ignoring them.
    fastening = config["fastening"]
    expected = {"screwsPerEnd": 12, "rowsFromShoulderMm": [8, 18],
                "rowAnglesDeg": [[0, 60, 120, 180, 240, 300], [30, 90, 150, 210, 270, 330]],
                "tubeClearanceHoleMm": 5.5, "adapterTapDrillMm": 4.2,
                "spotfaceDiameterMm": 10, "spotfaceDepthMm": 0.5,
                "dowelPerEnd": 1, "dowelDiameterMm": 5, "dowelLengthMm": 10,
                "dowelAngleDeg": 15, "dowelFromShoulderMm": 12.5}
    if config["schema"] != 1 or not fastening["screw"].startswith("M5 x 12 ") or any(fastening[key] != value for key, value in expected.items()):
        raise ValueError("Fastener edits need a matching reviewed native kernel")
    cover = config["cover"]
    if any(cover[key] != value for key, value in {"outerDiameterMm": 84, "innerDiameterMm": 80,
           "lengthMm": 30, "lipThicknessMm": 1.5, "tubeRadialClearanceMm": 0.2, "outerEdgeFilletMm": 0.75}.items()):
        raise ValueError("Cover edits need a matching reviewed native kernel")
    inputs = {"configSha256": digest(config_path), "baselineLayoutSha256": digest(baseline_path),
              "sourceSha256": digest(source), "generatorSha256": digest(ROOT / "straight-link.cpp"),
              "preparerSha256": digest(Path(__file__))}
    key = hashlib.sha256(json.dumps(inputs, sort_keys=True).encode()).hexdigest()[:20]
    build = ROOT / "build/straight"
    target = build / "prepared" / key
    target.mkdir(parents=True, exist_ok=True)
    vendor = target / "vendor"
    vendor.mkdir(exist_ok=True)
    publication = target / "preparation.json"
    if publication.exists():
        previous = json.loads(publication.read_text())
        if previous["inputs"] != inputs:
            raise ValueError("Derived cache inputs changed")
        for relative, expected_hash in previous["files"].items():
            if digest(target / relative) != expected_hash:
                raise ValueError("Derived cache changed: " + relative)
        write_json(build / "current.json", {"layoutPath": str(target / "layout.json"),
                   "vendorDirectory": str(vendor), "preparationPath": str(publication)})
        print("Reused verified straight-link variant:", target)
        return
    cache = ROOT / "build/host/native/cadkit/cmake-ninja/CMakeCache.txt"
    prefix = Path(re.search(r"^CADKIT_OCCT_DIR:PATH=(.+)$", cache.read_text(), re.M).group(1))
    binary = build / "straight-link"
    libraries = ["TKDESTEP", "TKXSBase", "TKShHealing", "TKBO", "TKBool", "TKPrim", "TKOffset",
                 "TKFillet", "TKGeomAlgo", "TKTopAlgo", "TKBRep", "TKGeomBase", "TKG3d", "TKG2d", "TKMath", "TKernel"]
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-I", str(prefix / "include/opencascade"),
                    str(ROOT / "straight-link.cpp"), "-L", str(prefix / "lib"), "-Wl,-rpath," + str(prefix / "lib"),
                    *["-l" + name for name in libraries], "-o", str(binary)], check=True)
    derived = target / "derived"
    derived.mkdir(exist_ok=True)
    dimensions = [config[key] for key in ("sourceLengthMm", "removedMm", "protectedEndLengthMm",
                  "transitionLengthMm", "spigotEngagementMm", "tubeOuterDiameterMm", "tubeWallMm",
                  "spigotDiametralClearanceMm", "adapterCableBoreMm")]
    with (target / "native.log").open("w") as log:
        subprocess.run([str(binary), str(source), str(derived), *map(str, dimensions)], check=True, stdout=log)
    audit = json.loads((derived / "native-audit.json").read_text())
    if audit["method"] != "protected-ends-straight-tube-radial-fasteners-v2" or audit["protectedEndsMaterialDifferenceSolids"] != 0 or not audit["joinedStructuralSolidValid"] or audit["structuralOverlapMm3"] > 1e-5:
        raise ValueError("Derived structural/interface audit failed")
    layout = json.loads(json.dumps(baseline))
    layout["assets"] = [a for a in layout["assets"] if a["id"] != config["sourceAsset"]]
    for asset in layout["assets"]:
        path = source_vendor / (asset["id"] + ".STEP")
        if digest(path) != asset["sha256"]:
            raise ValueError("Baseline asset changed: " + asset["id"])
        shutil.copyfile(path, vendor / path.name)
    repairs = json.loads((source_vendor / "repairs.json").read_text())
    shutil.copyfile(source_vendor / "repairs.json", vendor / "repairs.json")
    for repair in repairs:
        for path in source_vendor.glob(f"{repair['asset']}-solid{repair['solid']}*.STEP"):
            shutil.copyfile(path, vendor / path.name)
    forearm = next(o for o in layout["occurrences"] if o["id"] == "forearm")
    parts = []
    for index, part in enumerate(audit["parts"]):
        asset_id = f"straight{index}"
        path = vendor / (asset_id + ".STEP")
        shutil.copyfile(derived / part["file"], path)
        layout["assets"].append({"id": asset_id, "file": part["file"], "sha256": digest(path), "solidCount": 1})
        occurrence_id = "forearm" if part["id"] == "tube" else "forearm-" + part["id"]
        if occurrence_id == "forearm":
            forearm["asset"] = asset_id
            forearm.pop("sourceOccurrence", None)
            forearm.pop("sourcePackage", None)
        else:
            layout["occurrences"].append({"id": occurrence_id, "asset": asset_id,
                                          "matrix": forearm["matrix"].copy(), "parent": "forearm"})
        parts.append({**part, "occurrence": occurrence_id, "asset": asset_id, "sha256": digest(path)})
    layout["notes"] = [n for n in layout["notes"] if "40 mm hollow transition" not in n]
    layout["notes"].append("Straight-forearm prototype: retained supplier ends, 206 mm tube, separate adapters and service covers; fasteners/fits are preliminary, all joints remain passive.")
    layout["straightForearm"] = {"inputs": inputs, "parts": parts, "audit": audit, "config": config}
    write_json(target / "layout.json", layout)
    files = {str(path.relative_to(target)): digest(path) for path in vendor.glob("*") if path.is_file()}
    files["layout.json"] = digest(target / "layout.json")
    files["derived/native-audit.json"] = digest(derived / "native-audit.json")
    write_json(publication, {"inputs": inputs, "files": files, "parts": parts, "audit": audit})
    write_json(build / "current.json", {"layoutPath": str(target / "layout.json"),
               "vendorDirectory": str(vendor), "preparationPath": str(publication)})
    print(f"Prepared {len(parts)} valid derived parts; protected ends unchanged; {audit['tubeLengthMm']} mm straight tube")


if __name__ == "__main__":
    main()
