
The isolated dual electric wrist can be inspected through
`materia.electric-tool.project.json` (the shared `DhDualToolPreview` in
`../dh-gripper`). It uses two supplier PGC references, CNC finger variants and a
prototype orthogonal adapter. Build/check instructions and assumptions are in
`../dh-gripper/README.md`. It is a tool-design reference; the active pneumatic
mission, electrical controls and tending reach certificate are not replaced by
opening it.


The two tray supports use the reusable `machinekit.structural.TSlotTable`
assembly: HFS5-4040 posts, upper and lower perimeter rails, nominal angle
brackets, levelling feet, and a separate 10 mm steel top. The front support row
is inset 100 mm for robot clearance. Top dimensions and height still follow the
tray layout. `memberCuts()` exposes each extrusion cut; `cutList()` aggregates
stock by profile. Constructor options select a square catalog profile, top
thickness, front inset, foot height, and lower rail height (rail centre above floor).
Brackets and feet are simplified without bolt holes, fasteners, or a specified
adjustment range; the assembly is not a structural load rating.

Run the CAD/layout/collision checks and export a standalone table preview:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.table-checks.json
```

The preview is `machinekit/examples/machine-tending/build/tslot-tables.mtrg`.
The reach study uses individual conservative boxes for every extrusion, bracket
arm, foot and top; the table revision is included in the certificate fingerprint.
