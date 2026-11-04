# Machined parts package (milled parts, ground bar pads and nut blocks)

Builds the quote package for the hobber's made parts that go to a CNC shop or a ground-stock vendor: every member
with a `ManufacturingRoute` facet (FRAME_PLAN H8). The exporter constructs the parts directly, with the same
constructors `FlexsplineHobber` installs (`HobberSaddle` designs, `HobberDrives.nutBlock`), rather than building the whole
hobber; `machined_package.py` makes the drawings, tables and checks from its output.

```sh
cd ~/dev/materia
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
| `HOBBER-GE-X-drive-pad` | one ground flat bar, 15 mm, under both supports (ground together: coplanar seats), holes of both | 80 x 15 |
| `HOBBER-GE-Z-drive-pad` | one ground flat bar, 40 mm, under both supports, holes of both, cut by the mount's flange relief and the eRob housing envelope swept over the Z travel plus the 2 mm gap (`HobberZOffset.relief`) | 80 x 40 |
| `HOBBER-GE-Y-drive-fixed/-floating` | milled L bracket, 1 setup, fine | block |
| `HOBBER-GE-Y-nut-seat` | ground flat bar, 20 mm | 60 x 20 |
| `HOBBER-GE-X-scale-seat`, `HOBBER-GE-Z-scale-seat` | ground flat bar, 10 x 20, 2 tapped M8 | 20 x 10 |
| `HOBBER-GE-Z-scale-arm` | milled arm for the Z scale head, 1 setup, fine | block |
| `HOBBER-GE-Y-saddle` | milled, 2 setups, fine (boss for the Y scale head) | block |
| `HOBBER-GE-swivel-plate` | ground steel plate, 15 mm (not aluminium: it carries the steel Y rails and the spindle seat), sawn to size, two faces ground after cutting, holes (Y scale housing seat, 8 adapter-ring screws) | plate 15 |
| `HOBBER-GE-swivel-adapter` | turned ring, 2 setups, fine, aluminium (output flange to swivel plate) | bar |
| `HOBBER-GE-spindle-seat` | milled split-clamp block for the Ø80 g6 cartridge, 2 setups, tight | block |
| `NUT-BLOCK-...` (X, Y, Z) | ground flat bar, 16 mm, bore, holes | 60 x 16 |

Quantities are one per machine. The list is the exporter's `DESIGNS` plus one nut block per axis; a new route on a
hobber member means adding it there.

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

- The nut seat `GE-Y-nut-seat` has no holes in the model: the Y nut block's two M6 bolts and the seat's own fixing to
  the Y saddle are not modelled, so the drawing lists none. Do not order it before they are.
- The X carriage and Z saddle list only the nut block bolts: their fixings to the rail blocks and the column are not
  modelled yet.
- Machined faces are listed by name with their tolerances; the views do not mark which face is which.
- Mass is volume times the generic steel density; the model's own mass estimate for the carriage may differ by the
  amount of detail it carries.

## Stock tolerances

Ground stock is priced and toleranced as bought (`GroundStock`: DIN 59350-type flat steel, Blanchard-ground steel plate, MIC-6 type aluminium plate; thickness about +-0.03 to 0.1 mm, flatness 0.05 to 0.1 mm per 100 mm; **assumed** typical catalogue values, each row carries its source). A face that needs better is marked *ground after cutting* in the face table with its tolerance: the drive pad seat planes, the nut block base ends, the Y nut seat, the scale seat bars and the swivel plate's two seats.
