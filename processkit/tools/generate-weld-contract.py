#!/usr/bin/env python3
"""Generate the device sensor layout from ProcessKit's canonical channel contract."""
from pathlib import Path
import re
import sys
root = Path(__file__).resolve().parents[2]
source = root / 'processkit/haxe/processkit/tool/WeldContract.hx'
values = re.findall(r'public static inline var (\w+):Int = (\d+);', source.read_text())
if len(values) != 7:
    raise SystemExit('Unexpected canonical welding sensor layout')
output = '// Generated from processkit.tool.WeldContract; run processkit/tools/generate-weld-contract.py.\n'
output += ''.join(f'pub const {name}: usize = {value};\n' for name, value in values)
target = root / 'processkit/schema/weld_contract.rs'
if '--check' in sys.argv:
    if not target.exists() or target.read_text() != output:
        raise SystemExit('Device welding contract is stale')
else:
    target.write_text(output)
