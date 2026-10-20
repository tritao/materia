# CamKit

CamKit produces `ToolpathProgram` directly from CAD or sheet geometry. It
supports 2.5D outside and inside profiles, pocket clearing,
and drilled hole centres. Profiles and pockets use configurable depth steps.
Coordinates are metres; feeds are metres/second.
`CamJob.finish(?setup)` returns the program with its tool library and setups;
there is no separate CAM program wrapper.
`ToolpathMotion.lower(program, binding)` sends the program to MotionKit.
`CncWriter.write(program, controller, binding.travel)` validates optional
stock data and machine travel, then exports LinuxCNC millimetre G-code.
Each CAM operation carries
its operation ID and, when available, a CAD feature reference.

`Setup` requires an ID and work origin. Its optional `SetupStock` records the
stock rectangle, top and bottom, safe Z, and `Fixture` rectangular keep-outs.
All bounds are metres in the program's
work coordinates. With an envelope, export checks machine travel. When stock
data is present it checks depth below stock bottom,
lateral rapids below safe Z and fixture intersections along lines and arcs.
The fixture check conservatively expands each rectangular keep-out by the
cutter radius and includes the curved path between arc endpoints. It treats
space below a fixture as blocked because the shank may still pass through it;
it does not model holder dimensions. A setup whose safe Z does not clear
every fixture is rejected.

Input adapters:

- `CamContour.fromSketch(authored, solved)` accepts one closed cadkit sketch
  boundary of lines, arcs or a circle.
- `CamContour.fromEdges(edges)` and `CamContour.fromFace(face)` accept one
  horizontal cadkit boundary. Curves are sampled to a chord tolerance.
- `CamContour.fromFaceBoundaries(face)` returns the outer contour followed by
  inner contours, so a plate with holes can use an outside profile for its
  outer boundary and inside profiles for the holes. `fromFace` still requires
  a single boundary.
- `CamJob.profileFace(face, tool, depth, feed)` profiles every inner boundary
  before the outer boundary. It checks that the tool fits each hole before
  adding any cuts to the job.
- `CamJob.pocketFace(face, tool, depth, feed, stepOver)` clears around inner
  boundaries as islands, finishing each island edge before the outer edge.
- `CamSheetProfiles.fromPlan(plan, placementId)` turns a manufacturingkit
  rectangular sheet placement into a profile contour.

Inside and outside profiles accept simple concave contours. The cutter path
rounds exposed corners, trims recessed corners, and rejects offsets that
collapse a narrow feature or collide with another edge. Convex pockets use
inward offset rings. Concave pockets use horizontal passes inside the cutter's
clearance region, retract between disconnected passes, and finish the inside
boundary at each depth.
`pocket(contour, ...)` accepts one boundary; `pocketFace(face, ...)` accepts
multiple internal islands and rejects a tool that cannot clear between an
island and another boundary.

Pocket operations use a separate plunge feed, defaulting to one quarter of
the cutting feed. The optional final `plungeFeed` argument sets it explicitly.
Long entries ramp down at no more than a 10% slope, retrace the ramp at full
depth, then cut the pass. Short entries plunge vertically at `plungeFeed`.
Rapid approach stops above the stock surface.

Profiles also accept a final `plungeFeed` argument, with the same default.
Profile and drill XY rapids run at safe Z; the job raises vertically before
the first XY move and before a tool change. Drill `feed` is its vertical
cutting feed.

Example:

```haxe
var tool = new Tool(2, 0.0, 0.002);
var contour = CamSheetProfiles.fromPlan(sheetPlan, "gantry-bracket");
var setup = new Setup("1", new Point3(0, 0, 0),
  new SetupStock(0.0, 0.05, 0.0, 0.05, 0.0, -0.003, 0.005));
var program = new CamJob(0.005, 12000)
  .profile(contour, tool, -0.002, 0.01)
  .finish(setup);
var previewAndExecution = ToolpathMotion.lower(program, machineBinding);
var controller = new CncController(LinuxCnc, program.tools);
var linuxCnc = CncWriter.write(program, controller, machineBinding.travel);
```

## Additive: slicing

`camkit.additive.Slicer.slice(part, settings, profile, name)` turns a CadKit `Part` (millimetres) into a
`ToolpathProgram` of printing operations (metres) and a `SliceReport`. It slices the B-rep: each layer is
`PlanarRegion.section` of the solid at the layer's middle height, perimeters are offsets of it
(`(k + 1/2)` bead widths in, from the outside in), the infill region starts where the last perimeter's
share ends, top and bottom skins are what the `topLayers` above and `bottomLayers` below do not cover
(booleans between layer regions), and infill is `CamPocketPlanner.fill` scanlines (rectilinear, alternating
45 and 135 degrees on a lattice shared by all layers). Boundaries are read back exactly, so a circle is an
`Arc` operation with no polygonization and no arc fitting. A bead is a rectangle with semicircular sides:
`SlicerSettings.beadArea(height)`, and beads that fill a layer are `beadSpacing(height)` apart, so a solid
layer deposits exactly its volume. Travels retract when they are longer than `retractMinTravel` and leave the
part.

`PrinterProfile` holds the bed centre, the purge line and the home position; its start sequence heats the
bed and hotend and waits, travels to the purge line and primes, and its end sequence retracts, switches the
heaters and the fan off and parks. The skirt belongs to no object, so cancelling the part leaves it.
`TestParts` builds the calibration cube, the test tower and the small tab in code.

The test project creates its own rectangular, rounded and holed plate fixtures,
plus a generated clamp keep-out.
It checks CAD curve sampling, pocket coverage, cutter offsets, depth steps,
hole-before-outer ordering, concave profiles and pockets, island clearance,
ramps and narrow-pocket plunges, safe profile and drill travel,
direct lowering and
CAM IR → G-code → CncKit IR
round trips. The suite also
checks the generic manufacturingkit sheet placement adapter using locally
authored input; it does not depend on a MachineKit example. The island pocket
is also cut with StockKit and compared with the finished part: no gouge, no
rapid or shank through stock, and leftover only in the inside corners.
Haxeon builds StockKit core and CadKit core from their package manifests, and CadKit finds its prebuilt OCCT
automatically (see the CadKit README), so run:

```sh
./haxeon/scripts/haxeon run --project camkit/tests/haxeon.json
```


## Tooling panels

`CamToolingPanels.build(panel, settings)` validates a ManufacturingKit `PanelPlan`
and returns ordered setups with `ToolpathProgram`s. Each source operation makes
one profile, pocket (including open-side rebates), or drill call, retaining the
source ID in provenance. Named access pockets precede their deeper cuts;
non-release faces precede the setup containing the outside contour, which is cut
last. Every setup records its face and original setup ID. Negative-Z machining
reflects X about the finished bounding profile midline; the raw blank lower-left
origin includes the explicitly declared margin. Z0 is the active machining face.

Settings are metres/m/s: tool library, chord tolerance, safe Z, RPM, cutting and
plunge feeds, step-down, step-over fraction, blank margin, work origin, fixtures
and machine travel. Cutter diameters must exactly match the operation, including
relief tools. The bridge checks stock/fixtures/travel before returning programs;
`CncWriter` applies those checks again for LinuxCNC export. Stock clearance does
not qualify workholding or release a part after its final outside cut.

`CamContour` retains its polyline vertices and optionally the exact line/arc
boundary supplied by this adapter. `CamExactBoundary` compensates that exact
boundary in native CAD (mm), then samples the centreline to the declared metre
chord tolerance. This avoids treating a cutter-radius relief as an undersized
polygon. Regular offsets can discard isolated matched-radius relief centres:
CamJob machines them with separate vertical entries under the same operation,
after a CAD boolean proves the full cutter disc lies in the removable region.
A cutter-sized circular profile/pocket similarly uses its point centreline.
Callers without exact metadata retain the existing polyline offset behavior.
Machine-specific feeds, axial-plunge suitability, tool lengths, fixtures and
part retention remain explicit shop qualification inputs.
