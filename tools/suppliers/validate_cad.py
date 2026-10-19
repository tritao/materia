#!/usr/bin/env python3
"""Validate inventoried STEP bytes using the local CadKit worker, one model per process."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import subprocess

from vallder import DEFAULT_OUTPUT, now, write_json

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--timeout", type=int, default=90)
    args = parser.parse_args()
    host = ROOT / "build/suppliers/cad-validator/host"
    worker = host / "main.hl"
    if not worker.exists():
        parser.error("Build worker: ./haxeon/scripts/haxeon run --project machinekit/examples/supplier-fasteners/haxeon.cad-check.json")
    worker_hash = hashlib.sha256(worker.read_bytes()).hexdigest()
    inventory = json.loads((args.output / "cad/inventory.json").read_text())
    assets = {a["id"]: a for d in inventory["downloads"] for a in d.get("assets", [])}
    directory = args.output / "cad/validation"
    directory.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    libraries = [ROOT / "haxeon/out", ROOT / "haxeon/.tools/hashlink",
                 host / "native/cadkit", host / "native/collisionkit-native"]
    env["LD_LIBRARY_PATH"] = ":".join(map(str, libraries)) + ":" + env.get("LD_LIBRARY_PATH", "")
    results = []
    for i, (digest, asset) in enumerate(assets.items()):
        result_path = directory / (digest + ".json")
        path = args.output / asset["file"]
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError("CAD asset hash mismatch: " + str(path))
        result = json.loads(result_path.read_text()) if result_path.exists() else {}
        retry = result.get("status") == "timeout" and result.get("timeout_seconds") != args.timeout
        if result.get("worker_sha256") != worker_hash or result.get("sha256") != digest or retry:
            env["MATERIA_CAD_FILE"] = str(path.resolve())
            pending = directory / (digest + ".pending.json")
            env["MATERIA_CAD_RESULT"] = str(pending.resolve())
            log = directory / (digest + ".log")
            try:
                with log.open("w") as stream:
                    process = subprocess.run([str(ROOT / "haxeon/.tools/hashlink/hl"), str(worker)],
                                             cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                             timeout=args.timeout)
                if process.returncode != 0:
                    result = {"status": "import_failed", "exit_code": process.returncode, "log": str(log)}
                else:
                    result = json.loads(pending.read_text())
            except subprocess.TimeoutExpired:
                result = {"status": "timeout", "timeout_seconds": args.timeout}
            result.update({"sha256": digest, "worker_sha256": worker_hash, "validated_at": now()})
            write_json(result_path, result)
            pending.unlink(missing_ok=True)
        results.append({"asset_id": digest, "file": asset["file"], **result})
        print(f"[{i+1}/{len(assets)}] {result['status']}: {asset['archive_member']}", flush=True)
        write_json(args.output / "cad/validation.json", {"schema_version": 1, "generated_at": now(),
            "inventory_complete": inventory["complete"], "complete": len(results) == len(assets),
            "summary": dict(Counter(r["status"] for r in results)), "assets": results})


if __name__ == "__main__":
    main()
