#!/usr/bin/env python3
"""Independently audit the STEP files actually consumed by the arm loader."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    current = json.loads((ROOT / "build/straight/current.json").read_text())
    record_path = Path(current["preparationPath"])
    record = json.loads(record_path.read_text())
    vendor = Path(current["vendorDirectory"])
    layout = json.loads(Path(current["layoutPath"]).read_text())
    config = layout["straightForearm"]["config"]
    cache = ROOT / "build/host/native/cadkit/cmake-ninja/CMakeCache.txt"
    prefix = Path(re.search(r"^CADKIT_OCCT_DIR:PATH=(.+)$", cache.read_text(), re.M).group(1))
    directory = ROOT / "build/straight"
    binary = directory / "audit-straight"
    libs = ["TKDESTEP", "TKXSBase", "TKBO", "TKBool", "TKPrim", "TKTopAlgo", "TKBRep", "TKGeomBase", "TKG3d", "TKG2d", "TKMath", "TKernel"]
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-I", str(prefix / "include/opencascade"),
                    str(ROOT / "audit-straight.cpp"), "-L", str(prefix / "lib"), "-Wl,-rpath," + str(prefix / "lib"),
                    *["-l"+name for name in libs], "-o", str(binary)], check=True)
    arguments = []
    hashes = {}
    for part in record["parts"]:
        path = vendor / (part["asset"] + ".STEP")
        if digest(path) != part["sha256"]:
            raise ValueError("Derived part changed before independent audit")
        hashes[part["id"]] = digest(path)
        arguments += [part["role"], str(path)]
    result_path = directory / "exported-parts-audit.json"
    subprocess.run([str(binary), str(config["adapterCableBoreMm"]),
                    str(-config["sourceLengthMm"]+config["removedMm"]-0.1), "0.1", str(result_path), *arguments], check=True)
    result = json.loads(result_path.read_text())
    result.update(preparationSha256=digest(record_path), layoutSha256=digest(Path(current["layoutPath"])),
                  auditSourceSha256=digest(ROOT / "audit-straight.cpp"), partSha256=hashes)
    result_path.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(f"Independently reimported {result['importedValidSolids']} valid solids; "
          f"{result['testedAxialDuctDiameterMm']} mm axial duct and cover clearances checked")


if __name__ == "__main__":
    main()
