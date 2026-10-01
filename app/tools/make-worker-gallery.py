#!/usr/bin/env python3
"""Writes app/examples/worker-gallery.materia: the cases the human worker is built and tested for, one lane each.

Every lane is a worker with a rack, a table and a part, running a short document job, side by side in one scene. The first
lane is the rack-to-table demo as it was; the rest use the Universal Animation Library character. Lanes are 6 m apart along Y
so their workers and parts never meet. Lane 0 keeps the demo's coordinates, so the demo's robot still stands where it did.

    python3 app/tools/make-worker-gallery.py            # rewrites the example
    python3 app/tools/make-worker-gallery.py --check    # fails if the example is not what this would write
"""

import argparse
import copy
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "app/examples/worker-rack-to-table.materia"
OUTPUT = ROOT / "app/examples/worker-gallery.materia"
BUNDLED = "animkit/assets/quaternius/worker.glb"
LIBRARY = "animkit/assets/quaternius-ual/ual-work.glb"
SPACING = 6.0

# name, character, top height, top size, kind
LANES = [
    ("Bundled worker, table height", BUNDLED, 1.06, 0.8, "standard"),
    ("Library worker, deep tops: bends over", LIBRARY, 1.06, 0.8, "standard"),
    ("Library worker, bench: crouches", LIBRARY, 0.6, 0.4, "standard"),
    ("Library worker, low shelf: kneels", LIBRARY, 0.3, 0.4, "standard"),
    ("Library worker, left hand: turns round, walks back", LIBRARY, 1.06, 0.4, "turn"),
    ("Library worker, both hands: long part, then a press", LIBRARY, 1.06, 0.4, "both"),
]


def slab(template, ident, label, x, y, top, size, colour):
    item = copy.deepcopy(template)
    item.update({"id": ident, "label": label, "x": round(x, 3), "y": round(y, 3), "z": round(top - 0.05, 3),
                 "width": size, "height": size})
    item["visualOverrides"] = {"baseColor": colour}
    return item


def part(template, ident, x, y, top, size):
    item = copy.deepcopy(template)
    width, height = size
    item.update({"id": ident, "label": "Part", "x": round(x, 3), "y": round(y, 3), "z": round(top + 0.04, 3),
                 "width": width, "height": height, "depth": 0.08})
    return item


def lane(index, spec, templates):
    name, character, top, size, kind = spec
    y0 = index * SPACING
    prefix = f"gallery-{index}"
    rack_at, table_at = (0.9, y0 - 0.2), (2.4, y0 + 1.8)
    hand, part_size = "right", (0.08, 0.08)
    if kind == "turn":
        hand, rack_at, table_at = "left", (0.9, y0 + 0.2), (-1.2, y0 + 0.2)
    if kind == "both":
        part_size, rack_at = (0.36, 0.16), (0.9, y0)
    rack, table, thing = f"{prefix}-rack", f"{prefix}-table", f"{prefix}-part"
    steps = [{"action": "pick", "object": thing, "hand": hand, "from": rack}]
    if kind == "turn":
        steps.append({"action": "place", "onto": table, "hand": hand, "retreat": "backward"})
    elif kind == "both":
        steps[0]["hand"] = "both"
        steps += [{"action": "walkTo", "target": {"point": [1.0, y0 + 1.1]}},
                  {"action": "place", "onto": table, "hand": "both", "retreat": "backward"},
                  {"action": "press", "target": {"point": [1.2, y0 + 3.2, top + 0.14]}, "hand": "right"}]
    else:
        steps += [{"action": "walkTo", "target": {"point": [1.0, y0 + 1.1]}},
                  {"action": "place", "onto": table, "hand": hand, "offset": [0.1, -0.05], "retreat": "backward"}]
    worker = copy.deepcopy(templates["worker"])
    worker.update({"id": f"{prefix}-worker", "label": name, "x": 0, "y": y0, "z": 0})
    worker["worker"] = {"asset": character, "job": json.dumps({"version": 1, "loop": False, "steps": steps}, separators=(",", ":")),
                        "migrationNote": None, "zones": [rack, table]}
    return [slab(templates["rack"], rack, f"Rack, {top} m", *rack_at, top, size, [0.55, 0.38, 0.2]),
            slab(templates["table"], table, f"Table, {top} m", *table_at, top, size, [0.2, 0.45, 0.65]),
            part(templates["part"], thing, rack_at[0], rack_at[1], top, part_size), worker]


def build():
    source = json.loads(SOURCE.read_text())
    by_id = {item["id"]: item for item in source["objects"]}
    templates = {"rack": by_id["worker-demo-rack"], "table": by_id["worker-demo-table"], "part": by_id["worker-demo-part"],
                 "worker": by_id["worker-demo"]}
    floor = copy.deepcopy(by_id["worker-demo-floor"])
    last = (len(LANES) - 1) * SPACING
    floor.update({"id": "gallery-floor", "label": "Floor", "x": 1.0, "y": last / 2 + 0.8, "width": 7.5, "height": last + 5.5})
    document = copy.deepcopy(source)
    document["objects"] = [floor] + [item for index, spec in enumerate(LANES) for item in lane(index, spec, templates)]
    return json.dumps(document, indent=1) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    text = build()
    if arguments.check:
        if not OUTPUT.exists() or OUTPUT.read_text() != text:
            sys.exit("app/examples/worker-gallery.materia is not what make-worker-gallery.py writes; run it")
        return
    OUTPUT.write_text(text)
    print(f"wrote {OUTPUT.relative_to(ROOT)}: {len(LANES)} lanes")


if __name__ == "__main__":
    main()
