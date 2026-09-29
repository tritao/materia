# ToolpathKit

ToolpathKit is the dependency-free machining path format shared by CNC, CAM,
motion execution, and stock simulation. `ToolpathProgram` carries ordered
`ToolpathOp` values, a `ToolLibrary`, and at least one `Setup`. The first setup
is active at the start; `SetSetup(id)` selects another setup in that program.
Unknown and duplicate setup IDs are rejected.

Paths are authored in work coordinates, in metres. Every `Setup` has an ID
and a work origin that places those coordinates in machine space. Optional
`SetupStock` holds stock bounds, safe Z, and fixtures for CAM validation.
`Setup.validate` requires that stock data. `TravelEnvelope` holds machine
lower and upper points and `check(program)` reports violations with source
`Provenance`, including extrema within arcs and helices. It applies each
program setup to work-coordinate moves and leaves machine-coordinate moves
in machine space.

`GeometryTools` provides lengths, points, and unit tangents for lines, arcs,
and circular moves in XY, XZ, and YZ. `GeometryOffset.profile` produces
inside or outside paths for a planar closed contour; `translate` places a
geometry by an XYZ offset. Cutter shapes and tool lookup live in
`toolpathkit.tool`.

Run the standalone core suite with:

```sh
./haxeon/scripts/haxeon run --project=toolpathkit/tests/haxeon.json
```

The execution adapter is described in [motion/README.md](motion/README.md).
The format decision and work log are in [docs/ADR-001-toolpath-ir.md](docs/ADR-001-toolpath-ir.md)
and [docs/PLAN.md](docs/PLAN.md).
