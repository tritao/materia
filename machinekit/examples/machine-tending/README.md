
The isolated dual electric wrist can be inspected through
`materia.electric-tool.project.json` (the shared `DhDualToolPreview` in
`../dh-gripper`). It uses two supplier PGC references, CNC finger variants and a
prototype orthogonal adapter. Build/check instructions and assumptions are in
`../dh-gripper/README.md`. It is a tool-design reference; the active pneumatic
mission, electrical controls and tending reach certificate are not replaced by
opening it.


The two tray supports use the reusable `machinekit.structural.TSlotTable`
assembly with HFS5-4040 posts, upper and lower rails, drilled corner angles,
bolts and end-loaded slot plate nuts, threaded levelling feet with locknuts,
and a counterbored 10 mm steel top. The upright front row is inset 100 mm for
robot clearance. Tops still follow the tray layout, while independent low bases
are about 720 x 670 mm at the stored 1019 mm top height. Width extends away from
the robot's central passage; depth extends forward with the rear base edge flush
with the top. Rear diagonal straps brace both upright frames. These are geometric
layout constraints; full robot reach and structural load qualification remain pending.

The generator preserves semantic subassemblies in the scene: upper frame, lower
frame, base (when fitted), feet, worktop, and optional bracing. Each frame's
connections contain their angle and fasteners; each foot groups its adapter,
levelling foot, locknut and mounting hardware. The cell names its supports
`left-table` and `right-table`. `partPath(member)` resolves a manufacturing member
name to its grouped occurrence path; cut-list names remain manufacturing labels.
The editor displays authored assembly groups and applies visibility and selection
navigation to their descendants.

Detailing follows dimensions and the selected `TSlotTableHardware` specification:

- HFS5-2020 defaults to M4 slot fasteners and M5 feet; HFS5-4040 defaults to M5
  slot fasteners and M8 feet. A caller can supply another validated hardware spec.
- Sixteen angles have two clearance holes each. Their screws seat against the
  angle faces; each nut fits the slot undercut. Slot plates must be loaded from
  the open extrusion ends before closing the frame.
- Four top mounting holes follow the upper rails. Counterbore depth adapts to
  top thickness and an available nominal screw length, recessing the heads while
  retaining at least 2 mm of plate. Screws retain 0.2 mm of slot bottom clearance.
- Each leg has a drilled and tapped bottom bore matching the foot thread. Foot
  stems retain at least 1.5 diameters of engagement through the reported adjustment
  range; a separate hex nut locks each foot against the leg.
- `rearBrace: true` adds a drilled diagonal rear strap and its slot fasteners.
  Bracing is an explicit choice, not an automatic stiffness or load calculation.

The constructor retains its existing dimension arguments, followed by optional
hardware, rear-brace and base-footprint arguments. With no footprint the original
floor-standing post construction is retained. A footprint adds low side rails,
cross members under each post row, angle connections, and separate tapped foot
adapter plates; end cross rails are added where there is room between connections.
The footprint must contain the upper frame. Its optional offsets default to zero.
For example:

```haxe
var bench = new TSlotTable(1200, 700, 900, "HFS5-4040", 18, 0, 40, 200, null, true);
var compact = new TSlotTable(320, 240, 650, "HFS5-2020", 10, 0, 25, 150);
var wideBase = new TSlotTable(540, 330, 1020, "HFS5-4040", 10, 100, 30, 220, null, true,
    {width: 720, depth: 670, offsetX: 90, offsetY: -170});
```

`memberCuts()` exposes extrusion cuts; `cutList()` aggregates profile stock.
`topMounts()` returns the top drilling/counterbore pattern, `machining()` reports
drilling and tapping for the selected foot mount (post end or base adapter), and
`footAdjustment()` reports nominal, minimum and
maximum foot extension. `billOfMaterials()` includes every placed hardware part.
Incompatible slots, obstructed driver space at the angles, thin tops, insufficient
foot engagement, and dimensions too small for the connections are rejected.

These are nominal dimensional reference specifications, not verified supplier
SKUs or load ratings. Angles use specified machined thicknesses; slot plate nuts
have semantic threads and unmodeled thread teeth. Hardware fit uses the actual
modeled slot depth, including the existing 2020 profile's corner-web allowance,
rather than assuming the nominal catalog depth. Supplier compatibility, strength,
and complete assembly-tool paths still need qualification before fabrication.

Run the sizing, CAD hole, mating interference, BOM and collision checks:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.table-checks.json
```

Generated files under `machinekit/examples/machine-tending/build/`:

- `tslot-tables.mtrg`: both detailed tray supports.
- `tslot-table-sizing.mtrg`: six layouts, including both square profiles, asymmetric
  and centred wide bases, and optional bracing.
- `tslot-table-manufacturing.json`: upper/base dimensions, member cuts, top holes,
  foot-mount machining, adjustment ranges and hardware BOMs for those layouts.

The reach study uses separate conservative envelopes for every member and hardware
part, preserving their poses. The table revision is included in the certificate
fingerprint; the full tending reach certificate still requires regeneration.
