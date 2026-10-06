#!/usr/bin/env python3
"""Verify RobotKit's declared dependency boundaries without building optional stacks."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ('core', 'sim', 'remote', 'serial', 'recording', 'autonomy', 'inference', 'policy')
configs = {name: json.loads((ROOT / 'robotkit' / name / 'haxeon.json').read_text()) for name in PACKAGES}
assert set(configs['core']['dependencies']) == {'haxeon-platform', 'trajectorykit'}, 'Core dependency boundary changed'
assert all('simkit' not in config['dependencies'] for name, config in configs.items() if name != 'sim'), 'SimKit must enter through robotkit-sim'
assert all('visionkit' not in config['dependencies'] for name, config in configs.items() if name != 'autonomy'), 'VisionKit must enter through robotkit-autonomy'
umbrella = json.loads((ROOT / 'robotkit/haxeon.json').read_text())
assert not {'robotkit-inference', 'robotkit-policy'} & set(umbrella['dependencies']), 'ONNX packages must be explicit opt-ins'

seen = set()
def closure(manifest):
    manifest = manifest.resolve()
    if manifest in seen:
        return
    seen.add(manifest)
    config = json.loads(manifest.read_text())
    for dep in config.get('dependencies', {}).values():
        closure(manifest.parent / dep['path'] / 'haxeon.json')
closure(ROOT / 'robotkit/core/haxeon.json')
assert {json.loads(path.read_text())['package']['name'] for path in seen} == {'robotkit-core', 'haxeon-platform', 'trajectorykit'}, 'Core has an unexpected transitive dependency'
for path in (ROOT / 'robotkit/core/haxe').rglob('*.hx'):
    source = path.read_text()
    for forbidden in ('import visionkit.', 'import kinematicskit.', 'import nativekit.sim.', 'import robotkit.inference.', 'import robotkit.policy.'):
        assert forbidden not in source, f'{path}: forbidden core import {forbidden}'
print('RobotKit package boundaries passed: core -> Haxeon platform + TrajectoryKit; simulation, vision and ONNX are separate')
