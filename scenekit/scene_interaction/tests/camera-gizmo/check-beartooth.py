#!/usr/bin/env python3
"""Consumer check against Beartooth's accepted camera/render dependency boundary, without pin edits."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--haxeon', type=Path, required=True)
    parser.add_argument('--preview-project', type=Path, required=True)
    args = parser.parse_args()
    project = args.preview_project.resolve()
    base = project.parent
    fixture = Path(__file__).resolve().parent
    package = fixture.parents[1] / 'haxe/src'
    manifest = json.loads(project.read_text())
    if manifest['package']['name'] != 'beartooth-scene-preview':
        raise ValueError('Expected the Beartooth scene preview manifest')
    with tempfile.TemporaryDirectory(prefix='scenekit-camera-compat-') as directory:
        temporary = Path(directory)
        shutil.copyfile(fixture / 'CameraCompatibilityTests.hx', temporary / 'CameraCompatibilityTests.hx')
        manifest['package']['name'] = 'scenekit-gizmo-camera-compat'
        manifest['entry'] = 'CameraCompatibilityTests'
        # Test this checkout's core even after the consumer adopts a prepared package.
        manifest['dependencies'].pop('scenekit-gizmo', None)
        manifest['sourceRoots'] = [str((base / root).resolve()) for root in manifest['sourceRoots'] if root != 'src'] + [str(package), str(temporary)]
        manifest['outputDir'] = 'build'
        for dependency in manifest['dependencies'].values():
            dependency['path'] = str((base / dependency['path']).resolve())
        cmake = manifest['native']['cmake']
        cmake['source'] = os.path.relpath((base / cmake['source']).resolve(), temporary)
        cmake['inputs'] = [str((base / source).resolve()) for source in cmake['inputs']]
        for key in ['interfaces', 'projections']:
            if key in manifest['ffi']:
                manifest['ffi'][key] = [str((base / source).resolve()) for source in manifest['ffi'][key]]
        destination = temporary / 'haxeon.json'
        destination.write_text(json.dumps(manifest, indent=2))
        subprocess.run([str(args.haxeon.resolve()), 'run', '--project', str(destination)], check=True, timeout=180)


if __name__ == '__main__':
    main()
