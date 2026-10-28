# ToolpathKit

ToolpathKit is the dependency-free machining path format shared by CNC, CAM,
3D printing, motion execution, and stock simulation. `ToolpathProgram` carries
ordered `ToolpathOp` values, a `ToolLibrary`, and at least one `Setup`. The
first setup is active at the start; `SetSetup(id)` selects another setup in that
program. Unknown and duplicate setup IDs are rejected.

Paths are authored in work coordinates, in metres. Every `Setup` has an ID
and a work origin that places those coordinates in machine space. Optional
`SetupStock` holds stock bounds, safe Z, and fixtures for CAM validation.
`Setup.validate` requires that stock data. `TravelEnvelope` holds machine
lower and upper points and `check(program)` reports violations with source
`Provenance`, including extrema within arcs and helices. It applies each
program setup to work-coordinate moves and leaves machine-coordinate moves
in machine space.

## Layout

- `haxe/toolpathkit` is the format, with no dependencies:
  - `path` holds the program and its operations (`ToolpathProgram`, `ToolpathOp`, `Provenance`,
    `ToolpathFrame`, `MoveKind`), path geometry (`PathGeometry`, `Point3`, `ArcPlane`,
    `GeometryTools`, `GeometryOffset`, `ArcFitting`) and the printing helpers (`PrinterOps`,
    `PrintFeature`, `BeadSection`).
  - `setup` holds `Setup`, `SetupStock`, `Fixture` and `TravelEnvelope`.
  - `tool` holds cutter shapes (`CutterProfile`), `Tool`, `ToolLibrary` and the router planning
    helpers (`RouterBit`, `RouterCut`).
- `motion/` is the separate `toolpathkit-motion` package that lowers a program onto MotionKit; see
  [motion/README.md](motion/README.md).
- `tests/` is the standalone suite; `docs/` holds the decision record and plans.

## Operations

`ToolpathOp` covers milling and turning (moves, spindle, coolant, tool changes and length offsets),
auxiliary axes and electronic gearing for lathes (`PositionJoint`, `FeedPerRevolution`,
`ElectronicGear`, `WaitSignal`), and 3D printing (`Deposit`, `FilamentMove`, `Heat`, `Fan`, and
layer, feature and object markers). Printing stores the bead laid per distance, which is machine
independent; the filament diameter belongs to the material and extruder.

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
The format decision is in [docs/ADR-001-toolpath-ir.md](docs/ADR-001-toolpath-ir.md), the record of the
T0–T7 migration in [docs/HISTORY.md](docs/HISTORY.md), and the planned native program format in
[docs/NATIVE_PROGRAM_PLAN.md](docs/NATIVE_PROGRAM_PLAN.md).
