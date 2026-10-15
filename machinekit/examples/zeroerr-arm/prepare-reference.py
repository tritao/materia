#!/usr/bin/env python3
"""Fetch pinned supplier CAD into ignored build/vendor; optionally reuse a local ZIP."""
import argparse
import hashlib
import json
import re
import os
import subprocess
from pathlib import Path
import urllib.request
import zipfile


def digest(data):
    return hashlib.sha256(data).hexdigest()


def verify_placements(data, occurrences):
    """Check the tracked placements against the original AP214 assembly graph."""
    text = data.decode("latin1")  # entity IDs and floating-point numbers are ASCII
    entities = {int(k): v for k, v in re.findall(r"#(\d+)\s*=\s*(.*?);", text, re.S)}

    def refs(value):
        return [int(n) for n in re.findall(r"#(\d+)", value)]

    def vector(number):
        values = re.search(r"\(\s*([^()]*)\)\s*\)", entities[number]).group(1)
        return [float(n) for n in values.split(",")]

    def axis(number):
        p, z, x = [vector(n) for n in refs(entities[number])]
        y = [z[1]*x[2]-z[2]*x[1], z[2]*x[0]-z[0]*x[2], z[0]*x[1]-z[1]*x[0]]
        return [[x[i], y[i], z[i], p[i]] for i in range(3)] + [[0, 0, 0, 1]]

    def inverse(m):
        return [[m[j][i] for j in range(3)] + [-sum(m[j][i]*m[j][3] for j in range(3))]
                for i in range(3)] + [[0, 0, 0, 1]]

    def multiply(a, b):
        return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

    placements = {}
    for value in entities.values():
        if not value.lstrip().startswith("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"):
            continue
        relationship, shape = refs(value)
        usage = refs(entities[shape])[-1]
        transform = int(re.search(r"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*#(\d+)",
                                  entities[relationship]).group(1))
        parent, child = refs(entities[transform])
        m = multiply(axis(parent), inverse(axis(child)))
        placements[usage] = [n for row in m[:3] for n in row]
    if set(placements) != {o["sourceOccurrence"] for o in occurrences}:
        raise ValueError("Reference occurrence set differs from supplier assembly")
    for occurrence in occurrences:
        if any(abs(a-b) > 1e-8 for a, b in zip(occurrence["matrix"], placements[occurrence["sourceOccurrence"]])):
            raise ValueError(f"Reference placement differs from supplier assembly: {occurrence['id']}")


def prepare():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path)
    parser.add_argument("--repair", action="store_true", help="derive validated fixes for invalid supplier solids after building")
    parser.add_argument("--occt-prefix", type=Path)
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    layout = json.loads((here / "reference-layout.json").read_text())
    build = here / "build"
    build.mkdir(exist_ok=True)
    archive = args.archive or build / "zeroerr-5kg.zip"
    if not archive.exists():
        if args.archive:
            raise SystemExit(f"Archive does not exist: {archive}")
        temporary = archive.with_suffix(".part")
        print("Downloading pinned ZeroErr 5 kg reference CAD", flush=True)
        urllib.request.urlretrieve(layout["source"]["url"], temporary)
        if digest(temporary.read_bytes()) != layout["source"]["sha256"]:
            temporary.unlink()
            raise SystemExit("Supplier archive changed; review it before updating the pin")
        temporary.replace(archive)
    if digest(archive.read_bytes()) != layout["source"]["sha256"]:
        raise SystemExit("Archive SHA-256 does not match reference-layout.json")
    vendor = build / "vendor"
    vendor.mkdir(exist_ok=True)
    with zipfile.ZipFile(archive) as source:
        for asset in layout["assets"]:
            data = source.read(asset["file"])
            if digest(data) != asset["sha256"]:
                raise SystemExit(f"CAD SHA-256 mismatch: {asset['file']}")
            (vendor / (asset["id"] + ".STEP")).write_bytes(data)
        data = source.read(layout["source"]["assembly"])
        if digest(data) != layout["source"]["assemblySha256"]:
            raise SystemExit("Assembly CAD SHA-256 mismatch")
        verify_placements(data, layout["occurrences"])
        (vendor / "assembly.STEP").write_bytes(data)
    print(f"Verified {len(layout['assets'])} component CAD files and assembly in {vendor}")
    if args.repair:
        prefix = args.occt_prefix or (Path(os.environ["CADKIT_OCCT_DIR"]) if os.environ.get("CADKIT_OCCT_DIR") else None)
        cache = build / "host/native/cadkit/cmake-ninja/CMakeCache.txt"
        if prefix is None and cache.exists():
            match = re.search(r"^CADKIT_OCCT_DIR:PATH=(.+)$", cache.read_text(), re.M)
            if match:
                prefix = Path(match.group(1))
        if prefix is None:
            raise SystemExit("Build the example first or specify --occt-prefix /path/to/occt/install")
        binary = build / "repair-reference"
        libraries = ["TKDESTEP"] if (prefix / "lib/libTKDESTEP.so").exists() else ["TKSTEP", "TKSTEPBase", "TKSTEPAttr"]
        libraries += ["TKXSBase", "TKShHealing", "TKBO", "TKBool", "TKPrim", "TKTopAlgo", "TKBRep", "TKGeomBase", "TKG3d", "TKG2d", "TKMath", "TKernel"]
        subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-I", str(prefix / "include/opencascade"),
                        str(here / "repair-reference.cpp"), "-L", str(prefix / "lib"),
                        "-Wl,-rpath," + str(prefix / "lib"), *["-l" + name for name in libraries], "-o", str(binary)], check=True)
        repairs = []
        for asset in layout["assets"]:
            audit = vendor / (asset["id"] + "-repair.jsonl")
            subprocess.run([str(binary), str(vendor / (asset["id"] + ".STEP")),
                            str(vendor / asset["id"]), str(audit),
                            *(["partition110"] if asset.get("partitionOutputSpigot") else [])], check=True, stdout=subprocess.DEVNULL)
            for line in audit.read_text().splitlines():
                record = json.loads(line)
                record.update(asset=asset["id"], sourceSha256=asset["sha256"])
                repaired = vendor / f"{asset['id']}-solid{record['solid']}.STEP"
                record["sha256"] = digest(repaired.read_bytes())
                if "partitionZ" in record:
                    for side in ["rotor", "stator"]:
                        record[side + "Sha256"] = digest((vendor / f"{asset['id']}-solid0-{side}.STEP").read_bytes())
                repairs.append(record)
        (vendor / "repairs.json").write_text(json.dumps(repairs, indent=2) + "\n")
        print(f"Derived {len(repairs)} repaired solids; original supplier files retained. Audit: {vendor / 'repairs.json'}")


if __name__ == "__main__":
    prepare()
