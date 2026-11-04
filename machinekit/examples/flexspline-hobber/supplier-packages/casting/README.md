# Casting package (base and column)

Builds the quote package for the two mineral (epoxy granite) castings of `FlexsplineHobber` with
their steel inserts and fixing sockets cast in: the base (`MineralCastBed`) and the column
(`MineralCastColumn`), exactly as the machine installs them. The exporter depends on `machinekit`
and needs CadKit's native library at run time. Each casting takes several minutes to export
(the anchor sizing in `HobberFoundationSizing` is not cached); the column takes about a minute and the base about five.

```sh
cd ~/dev/materia
export HAXEON_COMPILER_SERVER=0; ulimit -s 65536
./haxeon/scripts/haxeon build --compiler-only \
  --project=machinekit/examples/flexspline-hobber/supplier-packages/casting/haxeon.json
N=$PWD/app/build/host/native
export LD_LIBRARY_PATH=$PWD/haxeon/.tools/hashlink:$PWD/haxeon/out:$N/cadkit:$N/app:$N/collisionkit-native:$N/kinematicskit-native:$N/stockkit:$N/motionkit:$N/trajectorykit
HL=machinekit/examples/flexspline-hobber/supplier-packages/casting
./haxeon/.tools/hashlink/hl $HL/build/host/main.hl /tmp/casting bed
./haxeon/.tools/hashlink/hl $HL/build/host/main.hl /tmp/casting column
python3 $HL/casting_package.py /tmp/casting
```

The exporter writes, per casting `<kind>` (`bed`, `column`):

- `<kind>-casting.step`: the casting body as cast (inserts and anchor bolts cut out).
- `<kind>-casting-with-inserts.step`: the body with every cast-in insert, fixing socket and anchor
  bolt in place, as separate solids (full). `<kind>-casting-with-inserts-light.step` is the same
  without the anchor bolts.
- `inserts/<kind>-insert-*.step`: one STEP per distinct insert, in the form supplied to the pour
  (stock size, with the holes that exist at the pour). Holes the model routes to after casting are
  not in these files; the schedule lists them.
- `<kind>-package.json`: the exact data for the script: bounds, volume, mass, density, every insert
  (pose, bounds, holes, anchors, finish, tolerance, datum face), cavities, lifting points and the
  sections of the as-cast body (world coordinates).

`casting_package.py` then writes `casting-summary.json` (both castings: outline, volume, mass and
density, inserts, fixing sockets, cores and cavities, coolant reservation, lifting points, datum faces,
tolerance table with its source, notes and open questions), `<kind>-drawings.dxf` (DXF R12, mm, 1:1:
face views, insert layout and sections, one layer per view and kind: `<VIEW>_OUTLINE`,
`_SECTION`, `_INSERTS`, `_DATUM`, `_DIMS`, `_TEXT`) and `<kind>-drawings.pdf` (A3 landscape). PDFs
need Chrome or Chromium. `casting_package.py <dir> bed` runs one casting.

The STEP files are large (the base is about 8 MB each as text, about 1 MB zipped); zip them for email.

## What the script checks

It stops if the numbers and the drawings disagree. `check_rows` asserts that the outline equals the
nominal size, the mass equals density times the exported volume, every insert's exported box follows
from its pose, every insert has its recess in the casting, every STEP exists and the quantities add
up. `check_layout` asserts the design facts the notes rely on (no datum scheme in the model, three
levelling-foot pockets in the base, four lifting points, the ground datum seats and no declared
precision interfaces). After the drawings are built, it reads them back: the mass and insert count
printed, every insert rectangle in the plan and elevation layout views against the JSON bounds, the
datum faces, the overall dimension values, and the DXF outlines, insert rectangles and section
contours against the same data.

## What is in the model and what is not

The base no longer has a cut that follows the machine's motion: the spindle servo stands above the cartridge since the belt-drive redesign (2026-11-04), so `HobberSpindleRelief` finds no relief (the earlier 179 x 47.5 mm pocket, 13 mm deep, is gone). The class stays: if the servo ever comes within the running gap again, the feature returns by itself.

Tolerances the model holds: the insert position budget (0.01 mm on finished steel seats, checked
by `FoundationIntegrity` as position error plus tilt times face span; 0.5 mm on the fixing
sockets) and the ground-after-casting finish. Everything else (flatness, parallelism, general
tolerance, surface finish, cure and stress relief, sealing and paint, mould draft, lifting
hardware) is printed as "to agree with the supplier". The mineral material values (density
2400 kg/m3, E 35 GPa, loss factor 0.02, expansion 14 um/m/K, conductivity 2 W/m/K) are assumed
bulk values in `MaterialLibrary`, not test results. The lifting points are placeholder M12 threads
in steel plates; `MACHINE_DESIGN.md` says lifting hardware is deferred. The 7.92 L coolant
reservation is a constant in `FlexsplineHobberBaseReport`, not a measured cavity.
