"""Execute the pinned bd_warehouse NEMA23 oracle, independently of the native port.

uv venv build/suppliers/motor-reference-venv
uv pip install --python build/suppliers/motor-reference-venv/bin/python \
  'bd-warehouse @ git+https://github.com/gumyr/bd_warehouse.git@e6ae6e7bae74cb3a50cbde395930ba24a1051f88'
build/suppliers/motor-reference-venv/bin/python tools/suppliers/openbuilds_motor_reference.py
"""
import hashlib
import inspect
import json
from pathlib import Path
from bd_warehouse.open_builds import StepperMotor
from build123d import export_step

REVISION = 'e6ae6e7bae74cb3a50cbde395930ba24a1051f88'
SOURCE_SHA256 = '3f1d196b07d2adcc5c5cfa46a2ca50df24c4f920117d010933299f9b792a8fee'


def main():
    output = Path('build/suppliers/vallder')
    output.mkdir(parents=True, exist_ok=True)
    source = Path(inspect.getfile(StepperMotor))
    if hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise RuntimeError('Reference generator source does not match the pinned revision')
    motor = StepperMotor('Nema23', motor_length=56)
    solids = motor.solids()
    fused = solids[0].fuse(*solids[1:])
    if not motor.is_valid or not fused.is_valid:
        raise RuntimeError('Upstream motor geometry is invalid')
    export_step(motor, output / 'openbuilds-nema23-upstream.step')
    record = {'source_revision': REVISION, 'installed_source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'model': 'OpenBuilds Nema23', 'body_length_mm': 56,
              'valid': motor.is_valid, 'solid_count': len(solids),
              'fused_volume_mm3': fused.volume, 'shaft_tip_mm': motor.bounding_box().max.Z}
    (output / 'openbuilds-nema23-upstream.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record, indent=2))


if __name__ == '__main__':
    main()
