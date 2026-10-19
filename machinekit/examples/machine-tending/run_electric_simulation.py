#!/usr/bin/env python3
"""Run the electric physics checks using Materia's native simulation provider."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

REPO = Path(__file__).resolve().parents[3]
EXAMPLE = REPO / 'machinekit/examples/machine-tending'


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def prepare(clearance_first=False):
    review_file = 'electric-cycle-review-clearance-first.json' if clearance_first else 'electric-cycle-review.json'
    review = json.loads((EXAMPLE / 'build' / review_file).read_text())
    inputs = EXAMPLE / 'build/electric-simulation-inputs'
    inputs.mkdir(parents=True, exist_ok=True)
    for name, key in [('arm', 'armSha256'), ('fixtures', 'fixtureSha256')]:
        target = inputs / f'{name}.mtrg'
        if target.exists() and sha(target) == review['provenance'][key]:
            continue
        for cached in (Path.home() / '.cache/materia/tending-geometry-v1').glob('*/*.scene'):
            packed = cached.read_bytes()
            body = packed[32:]
            if hashlib.sha256(body).hexdigest() == review['provenance'][key]:
                if packed[:32] != hashlib.sha256(body).digest():
                    raise RuntimeError(f'Invalid geometry cache: {cached}')
                target.write_bytes(body)
                break
        else:
            raise RuntimeError(f'Reviewed {name} geometry is unavailable; regenerate and review the access study')
    tool = REPO / 'machinekit/examples/dh-gripper/build/expert/dual-expert-tool.mtrg'
    if sha(tool) != review['provenance']['toolSha256']:
        raise RuntimeError('The DH tool changed after route validation')
    (inputs / 'tool.mtrg').write_bytes(tool.read_bytes())
    print('Using the exact geometry snapshot from the electric route review.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--gripper-check', action='store_true')
    mode.add_argument('--servo-check', action='store_true')
    mode.add_argument('--withdraw-smoke', action='store_true')
    mode.add_argument('--placement-smoke', action='store_true')
    mode.add_argument('--seat-settling-check', action='store_true')
    mode.add_argument('--seat-yaw-clamp-check', action='store_true')
    mode.add_argument('--locating-smoke', choices=['a', 'b'])
    mode.add_argument('--tray-placement-smoke', choices=['a', 'b'])
    mode.add_argument('--vise-pickup-smoke', choices=['a', 'b'])
    parser.add_argument('--deterministic', action='store_true')
    parser.add_argument('--clearance-first', action='store_true')
    parser.add_argument('--locating-offset-check', action='store_true')
    args = parser.parse_args()
    if (args.gripper_check or args.servo_check or args.withdraw_smoke or args.placement_smoke or args.seat_settling_check or args.seat_yaw_clamp_check or args.locating_smoke or args.tray_placement_smoke or args.vise_pickup_smoke) and args.deterministic:
        parser.error("the physical adapter checks require MuJoCo")
    if args.locating_offset_check and args.locating_smoke != 'b':
        parser.error("the grasp-offset regression requires --locating-smoke b")
    if (args.locating_smoke or args.seat_yaw_clamp_check or args.tray_placement_smoke or args.vise_pickup_smoke) and not args.clearance_first:
        parser.error("the locating and yaw/clamp checks require --clearance-first")
    native = REPO / 'app/build/host/native/app'
    required = ['libnativekit_scene.so', 'libnativekit_sim_core.so', 'libnativekit_sim_mujoco.so', 'librobotkit_runtime.so']
    if not all((native / name).exists() for name in required):
        raise RuntimeError('Build Materia native simulation libraries first with haxeon build --project=app/haxeon.json')
    build = native / 'cmake-ninja'
    if not build.exists():
        build = native / 'cmake'
    # Keep the new immutable-frame contact API in sync with this checkout.
    subprocess.run(['cmake', '--build', str(build), '--target', 'nativekit_sim_core', 'nativekit_sim_mujoco', 'robotkit_runtime', '-j', '4'], cwd=REPO, check=True)
    if not (args.gripper_check or args.servo_check):
        prepare(args.clearance_first)
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = str(native) + ':' + env.get('LD_LIBRARY_PATH', '')
    project = 'haxeon.electric-gripper-physics.json' if args.gripper_check else 'haxeon.electric-servo-physics.json' if args.servo_check else 'haxeon.electric-simulation.json'
    command = [str(REPO / 'haxeon/scripts/haxeon'), 'run', f'--project=machinekit/examples/machine-tending/{project}']
    app_args = []
    if args.clearance_first:
        app_args.append('--clearance-first')
    if args.withdraw_smoke:
        app_args.append('--withdraw-smoke')
    if args.placement_smoke:
        app_args.append('--placement-smoke')
    if args.seat_yaw_clamp_check:
        app_args.append('--seat-yaw-clamp-check')
    if args.seat_settling_check:
        app_args.append('--seat-settling-check')
    if args.vise_pickup_smoke:
        app_args.append(f'--vise-pickup-smoke={args.vise_pickup_smoke}')
    if args.tray_placement_smoke:
        app_args.append(f'--tray-placement-smoke={args.tray_placement_smoke}')
    if args.locating_offset_check:
        app_args.append('--locating-offset-check')
    if args.locating_smoke:
        app_args.append(f'--locating-smoke={args.locating_smoke}')
    if args.deterministic:
        app_args.append('--deterministic')
    if app_args:
        command += ['--', *app_args]
    return subprocess.run(command, cwd=REPO, env=env).returncode


if __name__ == '__main__':
    raise SystemExit(main())
