# Made parts package (machining, ground stock and formed guards)

Builds the quote package for the hobber's made parts that go to a CNC shop or a ground-stock vendor: every member
with a `ManufacturingRoute` facet (FRAME_PLAN H8). The exporter builds the installed machine and exports its actual parts and joint-derived holes; `machined_package.py` makes the drawings, tables and checks from its output.

```sh
# Run from the repository root in your task worktree.
export HAXEON_COMPILER_SERVER=0; ulimit -s 65536
# the motor mounts' reference STEP (HZMotion) must be installed: build/suppliers/hzmotion, or MATERIA_HZMOTION_ROOT
HL=machinekit/examples/flexspline-hobber/supplier-packages/machined
OUT=$PWD/$HL/build                      # the exporter takes an absolute directory (haxeon run changes directory)
./haxeon/scripts/haxeon run --project=$HL/haxeon.json -- $OUT     # about 15 s once built
python3 $HL/machined_package.py $OUT
```

The exporter needs CadKit's native library at run time (the `haxeon run` flow sets it up). The script needs
Python 3 with numpy, and Chrome or Chromium on the PATH for the PDFs; `pdftotext` (poppler) is used for the read-back when present.
`build/` is git-ignored.

## Parts

| Part | Process | Stock |
|---|---|---|
| `HOBBER-GE-X-carriage` | milled, 2 setups, fine | block |
| `HOBBER-GE-Z-saddle` | milled, 2 setups, fine (pocket, bore and 16 screw holes for the eRob 110, R1) | block |
| `HOBBER-GE-X-drive-pad` | one ground flat bar, 15 mm, under both supports (ground together: coplanar seats), holes of both, guard threads in edge lands | 100 x 15 |
| `HOBBER-GE-Z-drive-pad` | one ground flat bar, 40 mm, under both supports, holes of both, cut by the mount's flange relief and the eRob housing envelope swept over the Z travel plus the 2 mm gap (`HobberZOffset.relief`) | 80 x 40 |
| `HOBBER-GE-Y-drive-fixed/-floating` | milled L bracket, 1 setup, fine | block |
| `HOBBER-GE-Y-nut-seat` | milled from 19 mm bar, 1 setup: the seat plane (16 mm) and two key ribs 3.5 x 3 mm either side of the nut block's 20 mm base (the block is located against the screw thrust in both directions) | block |
| `HOBBER-GE-X-scale-seat`, `HOBBER-GE-Z-scale-seat` | ground flat bar, 10 x 20, 2 M8 tapped through (the scale housing's screws) | 20 x 10 |
| `HOBBER-GE-Z-scale-arm` | milled arm for the Z scale head, 1 setup, fine | block |
| `HOBBER-GE-Y-saddle` | milled, 2 setups, fine (Ø116 forward cartridge-clearance channel; spine for the Y scale head) | block |
| `HOBBER-GE-outboard-carrier` | milled, 2 setups, fine; separate structural service interface, four derived M4 and two Ø4 H7 pins to the Y saddle | block |
| `HOBBER-GE-swivel-plate` | ground steel plate, 15 mm (not aluminium: it carries the steel Y rails and the spindle seat), sawn to size, two faces ground after cutting, holes (Y scale housing seat, 8 adapter-ring screws) | plate 15 |
| `HOBBER-GE-swivel-adapter` | turned ring, 2 setups, fine, aluminium (output flange to swivel plate) | bar |
| `HOBBER-GE-spindle-seat` | milled split-clamp block for the Ø80 g6 cartridge, 2 setups, tight | block |
| `HOBBER-GE-motor-cradle` | milled steel, 2 setups, fine: 12 mm floor (8 mm ahead of y 195, extended forward to y 125 for the front bolt row at y 135) on the spindle seat block's top (4 × M8, 2 dowels, slit over the seat's), an 80 × 55 mm rest block (top ground flat: the seat of the shim pack under the servo body, FE 2026-11-06), 8 mm wall carrying the spindle servo's flange (Ø70 pilot slot, 4 × M6 in vertical slots, ±3 mm travel; nuts on the rear), M8 jack screw lug under the flange, one windowed 6 mm rib | block |
| `HOB-ARBOR-BT30` | turned and ground in 2 setups, face key milled at the second setup; BT30 taper, Ø16 hob seat, M16x1 retaining thread, Ø15 journal; functional runout 0.002 (design requirement) | bar |
| `NUT-BLOCK-...` (X, Y, Z) | ground flat bar, 16 mm (Y 20 mm), bore, holes; X and Z (part number ends `-P`) also 2 reamed Ø6 H7 dowel bores in the base end (the Y block has none: the seat's key locates it) | 60 x 16 |

Quantities are one per machine. The list is the exporter's `DESIGNS` plus one nut block per axis; a new route on a
hobber member means adding it there.

The P2 export also includes the installed outboard housing, floating sleeve, M36x1 gland, rear cap and ground inner spacer. Their separate routes preserve the indicated housing interface, matched bearing retention and thermal float. Bolt and dowel tables are derived from joints; the gland's two blind pin-spanner holes are functional tool features. See `b-align-outboard` for indication and pinning.

The P4 export adds nine chip shields: eight cut and formed 2 mm steel sheets (inside R2) and one flat rear lid. Routes carry thickness, bend count and radius separately from CNC setups. The STEP is the finished formed shape, not an unfolded blank; the sheet shop must develop the blank using its tooling and bend allowance. No welding. The Z saddle has two 6.5 mm-deep counterbores for its nut-block screw heads, keeping them clear of the guards.

P5 installs the X nut block with its tapped flange face toward the motor; the carriage interface remains at X232.5..248.5 mm. The nut and screw withdraw rearward with the block and chip guards retained; see the generated `x-service-screw` procedure.

## What is written

- `step/<part>.step`: the part in its own frame (rotated so X is the longest side and Z the shortest, minimum corner at
  the origin; the map from the model's frame is in `parts.json` as `frame`), holes cut (tapped holes at the tap drill).
- `machined-package.json`: data for the script (route, stock, mass, holes probed in the solid, tessellation for the views,
  STEP re-import volume).
- `parts.json`: summary per part: id, route (kind, process, supplier, setups, tolerance class, stock row, machined faces),
  material, density, volume, mass, size, quantity, cut or raw stock size, hole table (type, thread, drill and nominal
  diameter, depth, through, mouth position, direction, position tolerance), checks.
- `PARTS_RFQ.md` and `parts-rfq.csv`: one row per part with process, who it goes to, stock, size, mass, holes, tolerances.
- `drawings/<part>.pdf` (A4 landscape, `.html` source): top, front and side views (third angle, hidden-line removal,
  hidden hole edges dashed, hole centre marks with their table ids), overall dimensions, part data, machined-face table,
  hole table, notes, title block. `drawings/<part>.dxf`: the three views 1:1 (R12, mm).

## Where each number comes from

The route gives kind, setups, tolerance class, stock row and machined faces with tolerance and Ra; `GroundStock` the
thickness tolerance, flatness, parallelism and grinding Ra; the geometry the size, volume and mass (density from
`MaterialLibrary`); `HobberSaddle.holes()`, `NutBlock.holes()` and `HobberDrives.seatRouteHoles()` the holes (kind, thread,
tap drill, dowel fit, depth, position), the same data the solids are cut from. A hole's position tolerance is the tolerance
of the routed face whose name says holes, bolts, dowels or bore; holes without such a face are general tolerance. General
tolerance is ISO 2768-m, or -f when the route's class is fine. Nothing is toleranced tighter than the route says.

What the model does not hold is in `ORDERING` in the script and printed on every sheet: material grade (the model has
generic steel), heat treatment, finish and protection, tap run-out below blind tapped depth, and reaming the dowel
holes at assembly (procedure H7).

## Checks

`machined_package.py` exits 1 and prints the findings when any of these fail:

- each STEP re-imported has the model solid's volume within 0.1 % (exporter), mass equals density times volume;
- every hole of every table is in the solid (exporter probes with cylinders: the bore is empty at 0.98 r, the wall
  length of a 1.02 r probe matches the hole depth, so a missing, shifted, wider or shallower hole fails) and its mouth lies
  on the part; through or blind is measured, not assumed;
- milled parts: at most 2 setups and at most 600 mm; ground stock: a side equals the stock thickness and the other
  two give a standard bar width;
- the drawings read back: part number, overall dimensions, material, every hole id and coordinate and every face name
  are on the sheet and (with `pdftotext`) in the PDF.

Finding the Z saddle's nut block holes cut 363 mm away from the part (the holes were in the work frame, the saddle in its
own) is the first thing these checks caught; `HobberSaddle.withBlockBolts` now takes the part frame's origin.

## Limits

- The scale housings' seat holes (the bars' and the swivel plate's tapped M8, `HobberScales.seatHoles`) are still a hand list (a 20 mm bar cannot be a drilled-to-suit host at the 1.5 d edge rule);
  the heads' clearance holes, the pad bars' screws to the cast inserts, the nut blocks' dowels and the carriers' counterbores come from the joint derivation and are in the hole tables.
- Machined faces are listed by name with their tolerances; the views do not mark which face is which.
- Mass is volume times the generic steel density; the model's own mass estimate for the carriage may differ by the
  amount of detail it carries.

## Stock tolerances

Ground stock is priced and toleranced as bought (`GroundStock`: DIN 59350-type flat steel, Blanchard-ground steel plate, MIC-6 type aluminium plate; thickness about +-0.03 to 0.1 mm, flatness 0.05 to 0.1 mm per 100 mm; **assumed** typical catalogue values, each row carries its source). A face that needs better is marked *ground after cutting* in the face table with its tolerance: the drive pad seat planes, the nut block base ends, the Y nut seat, the scale seat bars and the swivel plate's two seats.
