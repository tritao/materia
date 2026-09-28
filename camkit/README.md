# CamKit

CamKit produces `cnckit.ir.CncOp` directly from CAD or sheet geometry. It
supports 2.5D outside and inside profiles, convex offset-cleared pockets,
and drilled hole centres. Profiles and pockets use configurable depth steps.
Coordinates are metres; feeds are metres/second.
`CamProgram.lower(machine)` sends the same operations to MotionKit and returns
its source map. `CamGCodeWriter.write(program)` exports LinuxCNC millimetre
G-code. Each CAM operation gets a synthetic source span whose line number is
the operation number, so editor highlighting works without G-code text.

Input adapters:

- `CamContour.fromSketch(authored, solved)` accepts one closed cadkit sketch
  boundary of lines, arcs or a circle.
- `CamContour.fromEdges(edges)` and `CamContour.fromFace(face)` accept one
  horizontal cadkit boundary. Curves are sampled to a chord tolerance.
  Faces with multiple boundaries are rejected until hole selection is added.
- `CamSheetProfiles.fromPlan(plan, placementId)` turns a manufacturingkit
  rectangular sheet placement into a profile contour.

Pocket clearing requires a convex contour. Inside and outside profile offsets
also require convexity; an `on` profile can follow a concave boundary. These
limits fail explicitly instead of producing an unsafe offset.

Example:

```haxe
var tool = new CncTool(2, 0.0, 0.002);
var contour = CamSheetProfiles.fromPlan(sheetPlan, "gantry-bracket");
var program = new CamJob(0.005, 12000)
  .profile(contour, tool, -0.002, 0.01)
  .finish();
var previewAndExecution = program.lower(cncMachine);
var linuxCnc = CamGCodeWriter.write(program);
```

The test project checks cadkit sketch and face inputs, manufacturingkit sheet
input, pocket coverage, round corner geometry, direct lowering and a full
CAM IR → G-code → CncKit IR round trip. Run with the existing cadkit native
build on `LD_LIBRARY_PATH`:

```sh
LD_LIBRARY_PATH=/path/to/cadkit/build/debug/core:/path/to/cadkit/build/debug/lin64/gcc/libd \
  ./haxeon/scripts/haxeon run --project camkit/tests/haxeon.json
```
