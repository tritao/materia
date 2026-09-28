# CamKit

CamKit produces `cnckit.ir.CncOp` directly from CAD or sheet geometry. It
supports 2.5D outside and inside profiles, pocket clearing,
and drilled hole centres. Profiles and pockets use configurable depth steps.
Coordinates are metres; feeds are metres/second.
`CamProgram.lower(machine)` sends the same operations to MotionKit and returns
its source map. `CamGCodeWriter.write(program, setup, cncMachine)` validates
the setup and exports LinuxCNC millimetre G-code. Each CAM operation gets a synthetic source span whose line number is
the operation number, so editor highlighting works without G-code text.

`CamSetup` records the stock rectangle, top and bottom, safe Z and optional
`CamFixture` rectangular keep-outs. All bounds are metres in the program's
work coordinates. Export checks machine travel, depth below stock bottom,
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
var tool = new CncTool(2, 0.0, 0.002);
var contour = CamSheetProfiles.fromPlan(sheetPlan, "gantry-bracket");
var program = new CamJob(0.005, 12000)
  .profile(contour, tool, -0.002, 0.01)
  .finish();
var previewAndExecution = program.lower(cncMachine);
var setup = new CamSetup(0.0, 0.05, 0.0, 0.05, 0.0, -0.003, 0.005);
var linuxCnc = CamGCodeWriter.write(program, setup, cncMachine);
```

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
rapid or shank through stock, and leftover only in the inside corners. Build
StockKit core (see `stockkit/README.md`), then run with it and the cadkit
native build on `LD_LIBRARY_PATH`:

```sh
LD_LIBRARY_PATH=build/stockkit-core:/path/to/cadkit/build/debug/core:/path/to/cadkit/build/debug/lin64/gcc/libd \
  ./haxeon/scripts/haxeon run --project camkit/tests/haxeon.json
```
