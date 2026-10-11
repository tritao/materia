#!/usr/bin/env python3
"""Generate the screwdriver layout, phases and results from its canonical Haxe contract."""
from pathlib import Path
import re
import sys
root = Path(__file__).resolve().parents[2]
source = root / 'robotkit/core/haxe/robotkit/tool/ScrewdriverContract.hx'
contract = source.read_text()
values = re.findall(r'public static inline var (\w+):Int = (\d+);', contract)
if len(values) != 24:
    raise SystemExit('Unexpected canonical screwdriver contract')
output = '// Generated from robotkit.tool.ScrewdriverContract; run processkit/tools/generate-screwdriver-contract.py.\n'
kind = re.search(r'public static inline var SENSOR_KIND:String = "([^"]+)";', contract)
if kind is None:
    raise SystemExit('Missing canonical screwdriver sensor kind')
output += f'pub const SENSOR_KIND: &str = "{kind.group(1)}";\n'
output += ''.join(f'pub const {name}: usize = {value};\n' for name, value in values)
target = root / 'processkit/schema/screwdriver_contract.rs'
if '--check' in sys.argv:
    if not target.exists() or target.read_text() != output:
        raise SystemExit('Device screwdriver contract is stale')
else:
    target.write_text(output)
