#!/usr/bin/env python3
"""Keep immediate geometry authoring from spreading: parts are recorded as CadKit documents (MachineComponent.record).

Counts the immediate-API call sites (`Part.box(`, `Part.cylinder*(`, `Part.prism(`, ... , `Solids.*(`,
`PosedParts.moved(`) in every Haxe file outside CadKit, and compares them with the checked-in baseline
(tools/immediate-geometry-baseline.json). A file may lose call sites but never gain them, and no new file may start
using the API. When a file has fewer than its baseline, rerun with --write to tighten the baseline.

Usage: tools/check-immediate-geometry.py [--write]
"""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / 'tools' / 'immediate-geometry-baseline.json'
PATTERN = re.compile(r'\b(?:Part\.(?:box|cylinder\w*|prism|sphere|loft|cone|torus)|Solids\.\w+|PosedParts\.moved)\(')
# CadKit is the kernel the documents evaluate with; its own sources use the immediate API by design.
EXCLUDED_TOP = {'cadkit', 'haxeon', 'build', 'docs'}
EXCLUDED_PARTS = {'build', 'out', 'vendor', 'node_modules', '.git'}


def counts():
    result = {}
    for path in sorted(ROOT.rglob('*.hx')):
        relative = path.relative_to(ROOT)
        if relative.parts[0] in EXCLUDED_TOP or EXCLUDED_PARTS & set(relative.parts):
            continue
        found = len(PATTERN.findall(path.read_text(errors='ignore')))
        if found:
            result[relative.as_posix()] = found
    return result


def main():
    current = counts()
    if '--write' in sys.argv:
        BASELINE.write_text(json.dumps(current, indent=1, sort_keys=True) + '\n')
        print(f'Wrote {BASELINE.relative_to(ROOT)}: {sum(current.values())} call sites in {len(current)} files')
        return
    baseline = json.loads(BASELINE.read_text())
    grown = [f'{path}: {count} (baseline {baseline.get(path, 0)})' for path, count in current.items() if count > baseline.get(path, 0)]
    shrunk = [path for path, count in baseline.items() if current.get(path, 0) < count]
    if grown:
        print('Immediate geometry authoring grew; record the part through MachineComponent.record instead:')
        for line in grown:
            print('  ' + line)
        sys.exit(1)
    if shrunk:
        print(f'{len(shrunk)} files use less immediate geometry than the baseline: run with --write to tighten it.')
    print(f'Immediate geometry: {sum(current.values())} call sites in {len(current)} files (baseline {sum(baseline.values())})')


if __name__ == '__main__':
    main()
