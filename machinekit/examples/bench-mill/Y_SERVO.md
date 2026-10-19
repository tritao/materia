# Selected Leadshine Y drive

This selectable `MillXYStageLayout(..., supplierY=true)` variant fits
ELM2H-0400LA60F and supplier M60D-BK12-C/BF12 geometry into the existing
mechanical mill frame and XY stage, with detailed pinned HGH15CA reference
carriages on both X and Y, preserving 160 mm Y travel and the X return
drive. The original generic layout remains the default for existing callers.

Open `materia.y-servo.project.json`, or run from the repository root:

```sh
python3 machinekit/examples/linear-rail-reference/prepare.py
MATERIA_ROOT="$PWD" python3 tools/suppliers/hzmotion_cad.py --offline
MATERIA_ROOT="$PWD" ./haxeon/scripts/haxeon run --project machinekit/examples/bench-mill/haxeon.y-servo.json
```

The run writes `build/y-servo/checks.json`, `xy-stage.assembly.json` and
`mill-y-servo.mtrg` in this example. The preview includes the actual frame and XY
stage with a fixed mating connection to the drive insert. Its BF12 and HGH15CA masses are
explicit visualization estimates and is excluded from engineering sizing.

The Y screw has the HZMotion drawing's 51 mm fixed end and a separate machining
requirement identity. Its length follows the mill datums. The motor and screw
shaft ends are 4 mm apart. The D25/L32 coupling with 14/10 mm bores remains a
reference requirement, with 14 mm shaft engagement per hub; no exact supplier
coupling SKU is claimed. Its two radial screw approaches face upward for service.

BK12 uses the corrected v2 drawing interface with the unchanged supplier STEP.
BF12 uses the supplier housing/bearing and the explicitly labelled nominal
installed DIN471 ring envelope. The unchanged supplier clip remains available
as `as-supplied`; its original groove interference is still tested. Ring elasticity,
preload and detailed installed shape remain unqualified.

Two saveable steel seat designs match the actual support bolt patterns. M6x35
mount bolts have 11 mm nominal receiving engagement; M6x40 floating-support bolts
have 9 mm; M6x25 seat anchors enter the frame inserts by 10 mm. M5x12 motor bolts
penetrate the bracket nominally 5.4 mm; effective motor thread depth is unpublished.
These are machining and joint design requirements, not load qualifications.

The supplier variant raises the rear bay roof by 12 mm for the motor connectors,
uses an 83 mm fixed seat/insert, and adds a 22x22 mm shaft passage through the
column web. The passage and bay require structural review. The checks cover
installed drive/frame clearances, sampled motion, the existing conservative
full-stroke XY/XYZ sweep audit and rear motor withdrawal. Straight 30 mm key
approaches are checked for coupling, support/seat and motor screws with the rear
hatch removed. They do not establish full hand clearance or cable routing.

Y motion uses an explicit signed 5 mm-lead kinematic relation. HZMotion does not
publish axial rigidity for this support, so the integration does not inherit the
generic BK12 compliance or claim cutting stiffness. Loads, mounting-joint strength,
bearing preload, coupling selection, cables, controls and thermal behavior remain
engineering work before manufacture. The existing preliminary axis sizing report
has not been requalified for this variant.

The eight X/Y carriages use the existing hash-pinned HIWIN HGH15CA STEP reference
hosted by Matis, including seals, mounting holes, rail channels and grease fittings.
This is a geometry reference, not confirmation of a purchased Vallder block’s
manufacturer. The selected Z layout accepts the same block for consistency.

The detailed-block running-fit proof uses the exact block against an undrilled,
uniform rail section covering its full length. This proves clearance continuously
along the extrusion without filling the carriage’s rail channel with a bounding
box. Other structural pairs retain the conservative sweep checks. A deliberate
1 mm seating error is also checked to ensure actual interference is detected.

The front structure is sized by `MillYTravel`, shared through `MillFrameLayout.yTravel`
with the XY stage. Defaults are 160 mm working stroke, 25 mm table-edge margin,
200 mm saddle depth and 2 mm overtravel reserve. Rail ends are rounded outwards
on a 10 mm grid after allowing for the paired carriages, end margins, floating
bearing, nut bracket and screw journal. The same front envelope covers both the
reference drive and the selected HZMotion drive. The base, inserts, bearing seat,
screw and feet then follow the resulting rail datums.

For another working stroke, pass e.g. `new MillYTravel(120)` to
`MillFramePreview.layout(reference, travel)`. Table depth follows
`stroke + 2 * tableEdgeMargin`; do not change joint limits separately. The compact
160 mm version has 410 mm Y rails and an 809 mm base, down from 508 and 909 mm.
The spindle and column retain their positions. A larger stroke with the existing
spindle reach is rejected with a clearance error: it needs a coordinated head/reach
change and another full motion, access and structural assessment.
