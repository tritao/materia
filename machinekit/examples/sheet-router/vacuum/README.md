# SR8 vacuum planning screen

The existing table has six isolated plenums, two columns by three rows. Each has
its own 50 mm valve and branch; the SR6 service layout keeps valves reachable
from the sides and services the plenum from above. Stock coordinates are metres
from the lower-left sheet corner, mapped to the centred CAD table datum. Zone
numbers advance left/right, then front/rear. Open only zones under the sheet;
mask uncovered areas within an open zone with impermeable blanks where practical.

| Row from front | Left zone, X=0..800 mm | Right zone, X=800..1600 mm | Y range |
|---|---|---|---|
| 1 | Z1 / valve 1 | Z2 / valve 2 | 0..L/3 |
| 2 | Z3 / valve 3 | Z4 / valve 4 | L/3..2L/3 |
| 3 | Z5 / valve 5 | Z6 / valve 6 | 2L/3..L |

`L` is 2600 or 3150 mm; rows are 866.667 or 1050 mm long and nominal zone
areas are 0.69333 or 0.84 m². The pressure model uses the full footprint and
the separate 0.8 effective-area factor, rather than claiming uniform suction
on every machined wall or groove.

The assumed 5.5 kW blower has 300 m³/h free flow and 25 kPa shutoff. These are
recipe assumptions, **not** a purchased pump curve. The calculation uses a
linear pressure/flow curve, Darcy flow through the CAD spoilboard (25 mm new) and the remaining
sheet thickness, quadratic branch/header losses and a 2 kPa filter loss at free
flow. Trunk: 75 mm ID x 4 m, K=6; branches: 50 mm ID x 3 m, K=4; friction factor
0.03. Air viscosity is 1.8e-5 Pa s and density 1.2 kg/m³. Both porous materials
start at permeability 1e-12 m². Impermeable sheet can be selected explicitly.
Replace these values with measured sheet/spoilboard leakage, fitted losses and
the actual blower curve before cutting acceptance. This is steady state; valve
transients, blocked filters and pump cooling are commissioning checks.

The StockKit component scan uses actual post-cut dexel intervals, four-neighbour
XY connectivity and positive Z overlap. A remaining skin connects the material;
a full-depth kerf splits it. Components retain volume, bounds and support area
in each zone. Floating layers receive no vacuum footprint. The scan streams two
rows, but its union labels grow with disconnected regions; it makes no SR9
sheet-scale memory guarantee. Call only at a completed cut checkpoint.

For each component, lateral capacity is
`0.5 friction x 0.8 effective area x sum(zone pressure x support area)`.
The required capacity is twice the supplied cutting-force allowance (300 N by
default). Every freed component is checked independently. A feature narrower
than ten grid cells is reported unresolved. Open kerf leakage is the uncovered
support area; the reported equivalent length divides it by the latest cutter
diameter. Porous remaining skins also leak through their actual thickness.
This model does not qualify overturning, the strength of a tab/skin, cutter
forces or a generic friction coefficient; SR13 and measured acceptance supply
those values.

## Native regression result

For a 400 x 400 x 18 mm sheet on one zone, at 1 mm dexel spacing, with the
**new 25 mm CAD spoilboard**:

| Bed Y | Active-zone vacuum | All six open | 50 mm freed-part lateral hold |
|---|---:|---:|---:|
| 2600 mm | 17.07 kPa | 6.25 kPa | 17.06 N |
| 3150 mm | 15.81 kPa | 5.36 kPa | 15.80 N |

The 50 x 50 mm part fails the 600 N factored requirement and emits
`freed_part_slipping` at final program line 43. The surrounding matrix holds
1065.9/987.0 N. A 0.5 mm onion skin remains connected and emits no freed-part
warning, but is explicitly marked `retainedWebNeedsForceQualification`.
A native 12 mm full-height tab also retains connectivity and is flagged for
strength qualification; full thickness alone does not qualify the connection.
Doubling spoilboard permeability lowers pressure; halving friction halves hold.
The default model reads thickness and permeability from the CAD spoilboard
recipe. An 18 mm sensitivity lowers active pressure to 15.19/13.79 kPa and
small-part capacity to 15.19/13.79 N; this is not an approved
surfacing or physical cutting allowance. These are assumed-material planning
results, not measured vacuum ratings.

The present 120 mm reference tool reaches the new spoilboard plane only at full
Z. Sacrificial overcut, a surfaced board datum and actual tool/holder gauge
lengths must be resolved in SR13 before cutting acceptance. The StockKit fixture's
1 mm through-cut proves material separation and hold detection; it does not
prove that an axis command can reach below the current machine datum.

## Freed-part policy

Do not through-cut a small unsupported part at the full cutting allowance.
Leave an onion skin or tabs and retain the part in the matrix. A final separation
pass requires a fresh component/force check at its actual reduced engagement,
or a separate qualified mechanical/adhesive fixture. For this test part the
allowable final force is only 6.9 N on the long bed in the 18 mm sensitivity after the factor of two;
there is no blanket promise that a light finish pass passes. Tab geometry and
remaining-web strength must be checked in SR13 before selecting their sizes.
Program ordering must preserve those connections until the finishing operation.

## Reproduction

Run `bash machinekit/examples/sheet-router/vacuum/check.sh` from the repository
root. It builds only Haxe code against existing StockKit/CAD native providers,
then runs the core component tests and both-bed vacuum tests under heavy-run.
`validation.json` retains the native report. The archive
`build/sheet-router-evidence/sr8-vacuum.tar.gz` retains source inputs and logs.
