"""Capture root/submodule diffs and the actual loaded geometry-generation binaries."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess


def sha(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def git(root, *args):
    return subprocess.check_output(["git", "-C", str(root), *args])


def snapshot(repository, label, output):
    patch = git(repository, "diff", "--binary", "--submodule=short", "HEAD")
    name = label.replace("/", "__") + ".patch"
    (output / name).write_bytes(patch)
    untracked = []
    for raw in git(repository, "ls-files", "--others", "--exclude-standard", "-z").split(b"\0"):
        if not raw:
            continue
        relative = Path(raw.decode())
        source = repository / relative
        if not source.is_file() or source.suffix not in {".hx", ".py", ".json", ".hxi", ".hxmap", ".cpp", ".hpp", ".h", ".c", ".cmake", ".txt", ".md"}:
            continue
        if source.stat().st_size > 10 * 1024 * 1024:
            raise ValueError(f"Untracked source too large to snapshot: {source}")
        target = output / "untracked" / label / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        untracked.append({"path":str(relative),"file":str(target.relative_to(output.parent)),"sha256":sha(target)})
    return {"path":label,"revision":git(repository,"rev-parse","HEAD").decode().strip(),
            "trackedDiffSha256":hashlib.sha256(patch).hexdigest(),"diffFile":"diffs/"+name,
            "trackedDirty":bool(patch),"untrackedSources":untracked}


def capture_sources(repository, output):
    repository,output = Path(repository).resolve(),Path(output).resolve()
    diffs = output/"diffs"
    diffs.mkdir(parents=True,exist_ok=True)
    records = [snapshot(repository,"root",diffs)]
    for line in git(repository,"submodule","status","--recursive").decode().splitlines():
        if line.startswith("-"):
            continue
        path = line[1:].split()[1]
        records.append(snapshot(repository/path,path,diffs))
    return {"schemaVersion":1,"root":records[0],"submodules":records[1:],"dirtySourcesCaptured":True}


def capture(repository, runtime, output):
    repository,output = Path(repository).resolve(),Path(output).resolve()
    sources=capture_sources(repository,output)
    binaries = []
    paths = set(Path(p).resolve() for p in runtime)
    for name in ("haxeon/.tools/haxe/haxe","haxeon/out/haxeon-cli.hl","haxeon/out/haxeon_runtime.hdll"):
        path = repository/name
        if path.is_file():
            paths.add(path.resolve())
    for path in sorted(paths):
        if not path.is_file():
            raise ValueError(f"Loaded build artifact is unavailable: {path}")
        binaries.append({"path":str(path),"sha256":sha(path),"bytes":path.stat().st_size})
    caches = sorted((repository/"machinekit/examples/bench-mill/build/fem-export").glob("**/cadkit/**/CMakeCache.txt"))
    builds = []
    for path in caches:
        text = path.read_text()
        entry = {"cachePath":str(path),"cacheSha256":sha(path)}
        found = re.search(r"^CADKIT_OCCT_DIR:PATH=(.+)$",text,re.M)
        if found:
            occt = Path(found.group(1))
            versions = list(occt.glob("lib/cmake/opencascade/*Version*.cmake"))
            entry["occt"] = [{"path":str(v),"sha256":sha(v),"contents":v.read_text()} for v in versions]
        builds.append(entry)
    if not any("libcadkit" in p["path"] for p in binaries) or not builds:
        raise ValueError("Missing loaded CadKit/OCCT build identity")
    return {**sources,"loadedBuildArtifacts":binaries,
            "cadkitBuilds":builds,"dirtySourcesCaptured":True}


def validate(record, packet_directory):
    base = Path(packet_directory)
    if record.get("schemaVersion")!=1 or record.get("dirtySourcesCaptured") is not True:
        raise ValueError("Missing report-grade source/build provenance")
    for entry in [record["root"]]+record["submodules"]:
        path = base/"provenance"/entry["diffFile"]
        if sha(path)!=entry["trackedDiffSha256"]:
            raise ValueError("Source/submodule diff snapshot changed")
        for source in entry["untrackedSources"]:
            if sha(base/"provenance"/source["file"])!=source["sha256"]:
                raise ValueError("Untracked source snapshot changed")
    if not record["loadedBuildArtifacts"] or not record["cadkitBuilds"]:
        raise ValueError("Missing geometry build identities")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("repository",type=Path)
    parser.add_argument("runtime",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    result=capture(args.repository,json.loads(args.runtime.read_text()),args.output.parent)
    args.output.write_text(json.dumps(result,indent=2)+"\n")


if __name__=="__main__":
    main()
