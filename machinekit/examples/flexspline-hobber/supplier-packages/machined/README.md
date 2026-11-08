# Made parts package (machining, ground stock and formed guards)

Builds the quote package for the hobber's made parts that go to a CNC shop or a ground-stock vendor: every member
with a `ManufacturingRoute` facet (FRAME_PLAN H8). The exporter builds the installed machine and exports its actual parts and joint-derived holes; `machined_package.py` makes the drawings, tables and checks from its output.

```sh
# Run from the repository root in your task worktree.
source /media/joao/DATA1/claude-hobber-tools/env-fe.sh
# the motor mounts' reference STEP (HZMotion) must be installed: build/suppliers/hzmotion, or MATERIA_HZMOTION_ROOT
HL=machinekit/examples/flexspline-hobber/supplier-packages/machined
OUT=$PWD/$HL/build                      # the exporter takes an absolute directory (haxeon run changes directory)
/media/joao/DATA1/claude-hobber-tools/run-fe.sh "$HL/haxeon.json" "$OUT"
python3 $HL/machined_package.py $OUT
```

The exporter needs CadKit's native library at run time (the `haxeon run` flow sets it up). The script needs
Python 3 with numpy, and Chrome or Chromium on the PATH for the PDFs; `pdftotext` (poppler) is used for the read-back when present.
`build/` is git-ignored.

## Current inventory

The installed assembly's `ManufacturingRoute` facets are the inventory; no parallel design list is maintained. Repeat parts aggregate by part number and physical machining revision. Bolts, counterbores and pins come from installed joints; functional arbor, gland and screw-clearance features remain explicit in their owning geometry.

| Group | Current route |
|---|---|
| X carriage | bought ground20 mm plate; joint drilling and profiled clearances |
| Four X ribs | sawn bright bar; the scale rib has its declared milled interface |
| Y module | profiled ground base/carriage, standard rail riser bars, nut carrier, scale seat/head carrier and stops |
| Head/Z interfaces | Z saddle, spindle split-clamp seat, servo cradle, outboard carrier and scale arm |
| X/Z drive seats | existing ground bars are still in the model; XA fixture-replicated replacements remain OPEN |
| Nut blocks and spindle/outboard | installed X/Z blocks, BT30 arbor, housing/sleeve/gland/cap/spacer |
| Guards | seven X sheets, Y module covers/retainers and six Z bellows mounts (two finished mounting designs) |

Cast-in inserts and sole pads retain their casting-package manufacturing records; bought bellows and scales are not fabricated by this package. Z bellows use a custom, ASSUMED thin-fabric RFQ envelope, not a confirmed vendor product. Ground-stock data is ASSUMED until a supplier row is chosen; inspect finished interfaces and replicate failing faces before custom grinding.

`assembly-tooling.csv` is a separate **Tooling** BOM category, excluded from machine quantities/mass. Each row carries an ASSUMED EUR price and the assembly steps that reference it. The exporter rejects an unknown tool reference. Tools include X/Y/Z drive arbors and the nut-bore height reference, replication kit, block-pad master, C/spindle mandrels, square/straightedge/level, indicators and bases, AF4 key>=400 mm, qualified low-profile keys, hoist/slings and bellows support straps. See `ALIGNMENT.md` and the generated X/Y/Z procedures.

This is a model RFQ package, **not a machine release**. The tilted Y-to-bed/tray fit, Y stiffness/accuracy qualification, XA replicated support seats/A6 qualification and Z screw ingress remain open in LINEAR_MODULE_PLAN.md. Part/drawing checks below validate the exported parts; they do not waive those machine-level gates.

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
- milled parts: at most 2 setups and at most 600 mm; ground stock: the finished profile fits its declared stock thickness and a standard bar width (or cut plate); routed removal allowances are listed; sawn bars contain the finished envelope;
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
